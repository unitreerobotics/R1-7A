"""Local camera capture for the real R1Arm recorder.

The real R1Arm workstation has its cameras plugged in directly. This module
supports two local backends:

``v4l2``
    Plain UVC cameras through ``cv2.VideoCapture`` (the "1080P USB Camera").
``realsense``
    Intel RealSense D405 through ``pyrealsense2``.  The D405's color stream is
    driven by the RealSense ASIC rather than by plain V4L2, so ``/dev/video4``
    negotiates a YUYV format but never delivers frames; librealsense is the
    only reliable path.

Both backends run one background thread per camera and publish the newest BGR
frame.  ``LocalCameraSubscriber.get`` returns a snapshot that is only valid when
every active camera contributed a frame, which is what the MCAP writer needs:
one sample must contain one image per ``camera_order`` entry.

MCAP color keys retain the historical head/wrist names used by existing
episodes. ``DEFAULT_CAMERAS`` uses the external view plus the wrist view.
"""

from __future__ import annotations

import os
import math
from collections import deque
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np


#: Stable color keys accepted by ``--camera key=...`` and stored in MCAP.
CAMERA_KEY_CHOICES = (
    "color_0_head_left",
    "color_1_head_right",
    "color_2_wrist_left",
    "color_3_wrist_right",
)

BACKEND_CHOICES = ("v4l2", "realsense")

#: Historical recording roles: D405 head view, JR0002 right wrist view.
DEFAULT_CAMERAS = (
    "key=color_0_head_left,backend=realsense,device=auto,size=640x480,fps=30,auto_exposure=false,exposure=8000,gain=16",
    "key=color_3_wrist_right,backend=v4l2,match=JR0002,"
    "size=640x480,fps=30,fourcc=MJPG",
)

#: JR0001 is the additional left-wrist device; verify its physical view on site.
DUAL_DEFAULT_CAMERAS = (
    DEFAULT_CAMERAS[0],
    "key=color_2_wrist_left,backend=v4l2,match=JR0001,"
    "size=640x480,fps=30,fourcc=MJPG",
    DEFAULT_CAMERAS[1],
)

#: RealSense pixel formats tried in order.  ``bgr8``/``rgb8`` need USB 3.x
#: bandwidth; on a USB 2.0 link only the raw ``yuyv`` stream gets through.
_REALSENSE_FORMATS = ("bgr8", "rgb8", "yuyv")


@dataclass
class CameraSpec:
    """One ``--camera`` entry: what to open and where to store its frames."""

    key: str
    backend: str = "v4l2"
    device: str = ""
    match: str = ""
    width: int = 640
    height: int = 480
    fps: float = 30.0
    fourcc: str = ""
    exposure: float | None = None
    auto_exposure: bool | None = None
    gain: float | None = None

    @property
    def size(self) -> tuple[int, int]:
        return self.width, self.height

    def label(self) -> str:
        where = self.device or (f"*{self.match}*" if self.match else "first")
        return (
            f"{self.key} [{self.backend} {where} "
            f"{self.width}x{self.height}@{self.fps:g}"
            f"{' ' + self.fourcc if self.fourcc else ''}]"
        )


def parse_camera_spec(text: str) -> CameraSpec:
    """Parse ``key=...,backend=...,device=...,size=WxH,fps=...,fourcc=...``."""
    fields: dict[str, str] = {}
    for chunk in text.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "=" not in chunk:
            raise ValueError(f"camera spec entry is not key=value: {chunk!r}")
        name, value = chunk.split("=", 1)
        fields[name.strip().lower()] = value.strip()

    unknown = set(fields) - {
        "key", "backend", "device", "match", "size", "fps", "fourcc",
        "exposure", "auto_exposure", "gain",
    }
    if unknown:
        raise ValueError(f"unknown camera spec field(s): {sorted(unknown)}")
    if "key" not in fields:
        raise ValueError(f"camera spec is missing key=: {text!r}")

    key = fields["key"]
    if key not in CAMERA_KEY_CHOICES:
        raise ValueError(f"unknown camera key {key!r}; pick from {CAMERA_KEY_CHOICES}")
    backend = fields.get("backend", "v4l2")
    if backend not in BACKEND_CHOICES:
        raise ValueError(f"unknown camera backend {backend!r}; pick from {BACKEND_CHOICES}")

    width, height = 640, 480
    if "size" in fields:
        parts = fields["size"].lower().replace(" ", "").split("x")
        if len(parts) != 2 or not all(p.isdigit() for p in parts):
            raise ValueError(f"camera size must be WxH: {fields['size']!r}")
        width, height = int(parts[0]), int(parts[1])

    auto = fields.get("auto_exposure")
    if auto is not None and auto.lower() not in ("true", "false", "1", "0"):
        raise ValueError("auto_exposure must be true/false or 1/0")
    if auto is not None and auto.lower() in ("true", "1") and "exposure" in fields:
        raise ValueError("manual exposure conflicts with auto_exposure=true")
    for name in ("fps", "exposure", "gain"):
        if name in fields and not math.isfinite(float(fields[name])):
            raise ValueError(f"{name} must be finite")
    if width <= 0 or height <= 0 or float(fields.get("fps", 30)) <= 0:
        raise ValueError("camera dimensions and fps must be positive")
    return CameraSpec(
        key=key,
        backend=backend,
        device=fields.get("device", "") if fields.get("device", "") != "auto" else "",
        match=fields.get("match", ""),
        width=width,
        height=height,
        fps=float(fields.get("fps", 30.0)),
        fourcc=fields.get("fourcc", "").upper(),
        exposure=float(fields["exposure"]) if "exposure" in fields else None,
        auto_exposure=None if auto is None else auto.lower() in ("true", "1"),
        gain=float(fields["gain"]) if "gain" in fields else None,
    )


def resolve_v4l2_device(spec: CameraSpec) -> str:
    """Turn a spec into a concrete ``/dev/videoN`` or ``by-id`` path.

    ``match`` looks for a substring in ``/dev/v4l/by-id`` so the default cameras
    survive re-enumeration (video indices change on every replug).
    """
    if spec.device:
        return os.path.realpath(spec.device)
    if spec.match:
        by_id = "/dev/v4l/by-id"
        try:
            for name in sorted(os.listdir(by_id)):
                if spec.match.lower() in name.lower() and name.endswith("video-index0"):
                    return os.path.realpath(f"{by_id}/{name}")
        except OSError:
            pass
        raise RuntimeError(
            f"no V4L2 camera matching {spec.match!r} under /dev/v4l/by-id; "
            "run scripts/verify_r1arm_right_cameras.py to list what is connected"
        )
    raise RuntimeError(
        f"camera {spec.key} needs device= or match=; "
        "run scripts/verify_r1arm_right_cameras.py to list what is connected"
    )


class _CameraWorker(threading.Thread):
    """Background thread publishing the newest decoded BGR frame."""

    #: Consecutive bit-identical frames before the stream counts as frozen
    #: (~2 s at 30 fps; a static scene still differs through sensor noise).
    _FROZEN_AFTER = 60

    def __init__(self, spec: CameraSpec) -> None:
        super().__init__(name=f"camera-{spec.key}", daemon=True)
        self.spec = spec
        self.status = "starting"
        self.detail = ""
        self.frames_seen = 0
        #: Cleared by ``resolve_active`` when the camera stops delivering.
        self.usable = True
        self._lock = threading.Lock()
        self._history = deque(maxlen=30)
        self._frame: np.ndarray | None = None
        self._received_at_ns = 0
        self._received_monotonic_ns = 0
        self._device_timestamp_ms: float | None = None
        self._stop_event = threading.Event()
        self._prev_bytes: bytes | None = None
        self._identical_streak = 0

    # -- public API -----------------------------------------------------
    def stop(self) -> None:
        self._stop_event.set()

    def snapshot(self, at_monotonic_ns=None) -> tuple[np.ndarray | None, int, int, int, float | None]:
        with self._lock:
            if at_monotonic_ns is not None:
                for frame, count, wall, mono, device in reversed(self._history):
                    if mono <= at_monotonic_ns:
                        return frame.copy(), count, wall, mono, device
                return None, -1, 0, 0, None
            if self._frame is None:
                return None, -1, 0, 0, None
            # Copy so the writer cannot race with the capture thread.
            return (
                self._frame.copy(), self.frames_seen, self._received_at_ns,
                self._received_monotonic_ns, self._device_timestamp_ms,
            )

    def has_frame(self) -> bool:
        with self._lock:
            return self._frame is not None

    def age(self, now_ns: int | None = None) -> float:
        """Seconds since the newest frame arrived (inf before the first one)."""
        with self._lock:
            stamp = self._received_at_ns
        if stamp == 0:
            return float("inf")
        if now_ns is None:
            now_ns = time.time_ns()
        return (now_ns - stamp) / 1e9

    def publish(self, frame: np.ndarray | None, device_timestamp_ms: float | None = None) -> None:
        if frame is None or frame.size == 0:
            return
        # A stalled driver can keep handing out the *same* frame; sensor noise
        # makes genuine consecutive frames differ, so bit-identical frames mark
        # a frozen stream (one recording held 13 copies of a single image).
        raw = frame.tobytes()
        if self._prev_bytes is not None and raw == self._prev_bytes:
            self._identical_streak += 1
        else:
            self._identical_streak = 0
            if self.status == "frozen":
                self.status = "open"
                self.detail = "image content resumed changing"
        self._prev_bytes = raw
        if self._identical_streak >= self._FROZEN_AFTER and self.status != "frozen":
            self.status = "frozen"
            self.detail = (
                f"image content unchanged for {self._identical_streak} "
                "consecutive frames"
            )
        with self._lock:
            self._frame = frame
            self._received_at_ns = time.time_ns()
            self._received_monotonic_ns = time.monotonic_ns()
            self._device_timestamp_ms = device_timestamp_ms
            self.frames_seen += 1
            self._history.append((frame,self.frames_seen,self._received_at_ns,
                                  self._received_monotonic_ns,device_timestamp_ms))

    def describe(self) -> str:
        return f"{self.spec.label()} status={self.status} frames={self.frames_seen}"

    def health(self) -> tuple[str, float]:
        """Return ``(status, age_seconds)`` for on-screen diagnostics."""
        return self.status, self.age()

    def run(self) -> None:  # pragma: no cover - exercised on hardware
        raise NotImplementedError


class V4L2Camera(_CameraWorker):
    """UVC camera read through OpenCV's V4L2 backend."""

    #: Reopen after this many consecutive failed reads.
    _REOPEN_AFTER = 15

    def __init__(self, spec: CameraSpec, device_path: str) -> None:
        super().__init__(spec)
        self.device_path = device_path
        self._capture: cv2.VideoCapture | None = None

    def _open(self) -> bool:
        capture = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            self.status = "open-failed"
            self.detail = f"cannot open {self.device_path}"
            return False
        # Order matters: V4L2 negotiates against the format, so FOURCC must be
        # set before the size on drivers that only expose one packed format.
        if self.spec.fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.spec.fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.spec.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.spec.height)
        capture.set(cv2.CAP_PROP_FPS, self.spec.fps)
        # JR0001/JR0002 lose frame throughput with a single queued V4L2 buffer.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        settings = []
        if self.spec.auto_exposure is not None or self.spec.exposure is not None:
            enabled = self.spec.auto_exposure if self.spec.auto_exposure is not None else False
            settings.append((cv2.CAP_PROP_AUTO_EXPOSURE, 3.0 if enabled else 1.0))
        if self.spec.exposure is not None:
            settings.append((cv2.CAP_PROP_EXPOSURE, self.spec.exposure))
        if self.spec.gain is not None:
            settings.append((cv2.CAP_PROP_GAIN, self.spec.gain))
        for prop, value in settings:
            if not capture.set(prop, value):
                capture.release()
                self.status = "settings-failed"
                self.detail = f"camera rejected property {prop}={value}"
                return False
        self._capture = capture
        self.status = "open"
        self.detail = self.device_path
        return True

    def _close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def run(self) -> None:  # pragma: no cover - exercised on hardware
        misses = 0
        while not self._stop_event.is_set():
            if self._capture is None and not self._open():
                if self._stop_event.wait(1.0):
                    break
                continue

            ok, frame = self._capture.read()
            if ok and frame is not None and frame.size:
                misses = 0
                self.publish(frame)
                continue

            misses += 1
            if misses >= self._REOPEN_AFTER:
                self.status = "no-frame"
                self.detail = f"{self.device_path} delivered no frame"
                self._close()
                misses = 0
                if self._stop_event.wait(1.0):
                    break
            else:
                time.sleep(0.05)
        self._close()


class RealSenseCamera(_CameraWorker):
    """RealSense color stream read through librealsense.

    The D405 on this workstation only starts streaming on the *second*
    ``pipeline.start()`` after a failed one, so opening always warms up with
    retries.  It also stalls mid-capture when the USB 2.0 link is saturated,
    hence the periodic re-open.
    """

    _OPEN_ATTEMPTS = 3
    #: Seconds to wait for each ``start`` to produce its first frame.
    _WARMUP_S = 2.5
    _FRAME_TIMEOUT_MS = 2000
    #: Restart the pipeline after this many consecutive frame timeouts.
    _RESTART_AFTER = 20

    def __init__(self, spec: CameraSpec, serial: str = "") -> None:
        super().__init__(spec)
        self.serial = serial
        self.pixel_format = ""
        self._pipeline = None
        self._rs = None

    def _build_config(self, rs_module, pixel_format):
        config = rs_module.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(
            rs_module.stream.color,
            self.spec.width,
            self.spec.height,
            getattr(rs_module.format, pixel_format),
            int(self.spec.fps),
        )
        return config

    def _set_exposure(self) -> None:
        settings = []
        rs = self._rs
        if self.spec.auto_exposure is not None or self.spec.exposure is not None:
            enabled = self.spec.auto_exposure if self.spec.auto_exposure is not None else False
            settings.append((rs.option.enable_auto_exposure, float(enabled)))
        if self.spec.exposure is not None:
            settings.append((rs.option.exposure, self.spec.exposure))
        if self.spec.gain is not None:
            settings.append((rs.option.gain, self.spec.gain))
        if not settings:
            return
        sensors = self._pipeline.get_active_profile().get_device().query_sensors()
        sensor = next((item for item in sensors if all(item.supports(option) for option, _ in settings)), None)
        if sensor is None:
            raise RuntimeError("No RealSense sensor supports the requested exposure controls")
        for option, value in settings:
            if not sensor.supports(option):
                raise RuntimeError(f"RealSense does not support {option}")
            limits = sensor.get_option_range(option)
            if not limits.min <= value <= limits.max:
                raise ValueError(f"{option}={value} outside {limits.min}..{limits.max}")
            sensor.set_option(option, float(value))

    def _open(self) -> bool:
        try:
            import pyrealsense2 as rs
        except ImportError:
            self.status = "no-librealsense"
            self.detail = "pyrealsense2 is not installed in this environment"
            return False
        self._rs = rs

        formats = (self.pixel_format,) if self.pixel_format else _REALSENSE_FORMATS
        for attempt in range(self._OPEN_ATTEMPTS):
            for pixel_format in formats:
                pipeline = rs.pipeline()
                # ``_convert`` and ``_warm_up`` both key off pixel_format, so it
                # has to be recorded before the first frame is decoded.
                self.pixel_format = pixel_format
                try:
                    pipeline.start(self._build_config(rs, pixel_format))
                except Exception as exc:  # noqa: BLE001 - driver raises broadly
                    self.detail = f"{pixel_format}: {type(exc).__name__}: {exc}"
                    continue
                if self._warm_up(pipeline):
                    self._pipeline = pipeline
                    try:
                        self._set_exposure()
                    except Exception as exc:  # noqa: BLE001 - report unsupported camera settings
                        self.detail = f"exposure setting failed: {exc}"
                        self._close()
                        continue
                    self.status = "open"
                    self.detail = (
                        f"{pixel_format} attempt {attempt + 1}"
                        + (f", exposure={self.spec.exposure:g}" if self.spec.exposure is not None else "")
                    )
                    return True
                try:
                    pipeline.stop()
                except Exception:  # noqa: BLE001
                    pass
                if self._stop_event.wait(0.3):
                    return False
        self.status = "no-frame"
        self.detail = self.detail or "RealSense color stream never started"
        return False

    def _warm_up(self, pipeline) -> bool:
        """Wait for the first frame; the D405's first ``start`` usually yields none.

        The frame is published rather than discarded: on a USB 2.0 link it is
        sometimes the only one that arrives.
        """
        deadline = time.monotonic() + self._WARMUP_S
        while time.monotonic() < deadline and not self._stop_event.is_set():
            try:
                frames = pipeline.wait_for_frames(timeout_ms=700)
            except Exception:  # noqa: BLE001 - timeout is expected while warming
                continue
            color = frames.get_color_frame()
            if color:
                self.publish(self._convert(color), float(color.get_timestamp()))
                return True
        return False

    def _convert(self, frame) -> np.ndarray:
        raw = np.asanyarray(frame.get_data())
        if self.pixel_format == "bgr8":
            return raw.copy()  # Retain ownership after librealsense recycles its frame buffer.
        if self.pixel_format == "rgb8":
            return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)

    def _close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
            self._pipeline = None

    def run(self) -> None:  # pragma: no cover - exercised on hardware
        misses = 0
        while not self._stop_event.is_set():
            if self._pipeline is None and not self._open():
                if self._stop_event.wait(1.0):
                    break
                continue

            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=self._FRAME_TIMEOUT_MS)
                color = frames.get_color_frame()
            except Exception:  # noqa: BLE001 - timeout raised as RuntimeError
                color = None

            if color is None:
                misses += 1
                if misses >= self._RESTART_AFTER:
                    self.status = "stalled"
                    self.detail = "RealSense stopped delivering frames; reopening"
                    self._close()
                    misses = 0
                    if self._stop_event.wait(1.0):
                        break
                continue

            misses = 0
            self.publish(self._convert(color), float(color.get_timestamp()))
        self._close()


class LocalCameraSubscriber:
    """Start the configured local cameras and serve coherent frame snapshots."""

    #: How long a camera has to keep delivering before it is trusted.
    _STABILITY_WINDOW_S = 1.5
    _STABILITY_MIN_FRAMES = 3

    def __init__(self, specs: tuple[CameraSpec, ...]) -> None:
        if not specs:
            raise ValueError("at least one camera is required")
        keys = [spec.key for spec in specs]
        if len(set(keys)) != len(keys):
            raise ValueError(f"duplicate camera keys: {keys}")
        self.specs = tuple(specs)
        self.workers: list[_CameraWorker] = []
        self.active_keys: tuple[str, ...] = ()
        self.inactive: list[tuple[str, str]] = []
        self._lock = threading.Lock()

    # -- setup ----------------------------------------------------------
    def _build_worker(self, spec: CameraSpec) -> _CameraWorker:
        if spec.backend == "v4l2":
            return V4L2Camera(spec, resolve_v4l2_device(spec))
        serial = "" if spec.device in ("", "auto") else spec.device
        return RealSenseCamera(spec, serial=serial)

    def start(self) -> None:
        for spec in self.specs:
            try:
                worker = self._build_worker(spec)
            except RuntimeError as exc:
                # Resolution failure is permanent for this run.
                worker = _MissingCamera(spec, str(exc))
            self.workers.append(worker)
            worker.start()

    def resolve_active(self, timeout: float) -> None:
        """Wait for frames, then lock in which cameras can actually record.

        A single frame is not enough to accept a camera: the RealSense D405 on a
        saturated USB link delivers one frame and then stalls, which would hold
        every recorded sample's timestamp hostage.  So once all cameras have
        produced a frame, each one has to keep producing for a short window.
        """
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if all(worker.has_frame() for worker in self.workers):
                break
            time.sleep(0.1)

        observed = {worker.spec.key: worker.frames_seen for worker in self.workers}
        time.sleep(self._STABILITY_WINDOW_S)
        for worker in self.workers:
            gained = worker.frames_seen - observed[worker.spec.key]
            if worker.has_frame() and gained < self._STABILITY_MIN_FRAMES:
                worker.usable = False
                worker.status = "stalled"
                worker.detail = (
                    f"{worker.detail} (only {gained} frames in "
                    f"{self._STABILITY_WINDOW_S:g}s)"
                )

        active: list[str] = []
        self.inactive = []
        for worker in self.workers:
            if worker.has_frame() and worker.usable:
                active.append(worker.spec.key)
            else:
                self.inactive.append((worker.spec.key, worker.describe()))
        with self._lock:
            self.active_keys = tuple(active)

    def _active_workers(self) -> list[_CameraWorker]:
        with self._lock:
            keys = self.active_keys or tuple(w.spec.key for w in self.workers)
        return [w for w in self.workers if w.spec.key in keys]

    # -- runtime --------------------------------------------------------
    def get(
        self,
    ) -> tuple[tuple[np.ndarray, ...] | None, int, int, tuple[str, ...]]:
        """Return ``(frames, frame_id, received_at_ns, source_streams)``.

        ``frames`` is ``None`` until every active camera has produced a frame.
        ``received_at_ns`` is the *oldest* contributing timestamp so a caller
        can treat it as "the whole sample is at most this stale".
        This compatibility view uses aggregate minimum IDs/timestamps. Use
        :meth:`get_atomic_snapshot` when frame identity must stay paired with pixels.
        """
        workers = self._active_workers()
        if not workers:
            return None, -1, 0, ()
        frames: list[np.ndarray] = []
        stamps: list[int] = []
        counters: list[int] = []
        sources: list[str] = []
        for worker in workers:
            frame, counter, received_at_ns, _, _ = worker.snapshot()
            if frame is None:
                return None, -1, 0, ()
            frames.append(frame)
            stamps.append(received_at_ns)
            counters.append(counter)
            sources.append(worker.describe())
        return tuple(frames), min(counters), min(stamps), tuple(sources)

    def get_atomic_snapshot(self, at_monotonic_ns=None) -> dict:
        """Return images and all per-camera identity/timing from one read per worker."""
        workers = self._active_workers()
        if not workers:
            return {"frames": None, "frame_ids": {}, "received_at_ns": {},
                    "received_monotonic_ns": {}, "device_timestamp_ms": {},
                    "source_streams": ()}
        frames, ids, wall, mono, device, sources = [], {}, {}, {}, {}, []
        for worker in workers:
            frame, frame_id, wall_ns, mono_ns, device_ms = worker.snapshot(at_monotonic_ns)
            if frame is None:
                return {"frames": None, "frame_ids": {}, "received_at_ns": {},
                        "received_monotonic_ns": {}, "device_timestamp_ms": {},
                        "source_streams": ()}
            key = worker.spec.key
            frames.append(frame)
            ids[key], wall[key], mono[key], device[key] = frame_id, wall_ns, mono_ns, device_ms
            sources.append(worker.describe())
        return {
            "frames": tuple(frames), "frame_ids": ids, "received_at_ns": wall,
            "received_monotonic_ns": mono, "device_timestamp_ms": device,
            "source_streams": tuple(sources),
        }

    def frame_times(self) -> dict[str, int]:
        """Per-camera arrival time of the newest frame (ns, 0 = never).

        P3 alignment: recording frames persist this per key so the image-vs-
        action offset is measurable per camera instead of assumed equal.
        Workers guard their own state in ``snapshot``; the active-worker list
        only changes during startup/shutdown, not while recording runs.
        """
        return {
            worker.spec.key: worker.snapshot()[2]
            for worker in self._active_workers()
        }

    def health(self) -> dict[str, tuple[str, float]]:
        """Per-requested-camera ``(status, frame_age_s)`` for diagnostics.

        Includes dropped cameras (status ``unavailable``/``stalled``) so callers
        can name the exact camera that is failing, not just "waiting".
        """
        now_ns = time.time_ns()
        return {worker.spec.key: worker.health() for worker in self.workers}

    def frame_ages(self) -> dict[str, float]:
        now_ns = time.time_ns()
        return {
            worker.spec.key: worker.age(now_ns) for worker in self._active_workers()
        }

    def wait_first_frame(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            if all(worker.has_frame() for worker in self.workers):
                return True
            time.sleep(0.05)
        return all(worker.has_frame() for worker in self.workers)

    def describe(self) -> str:
        return "; ".join(worker.describe() for worker in self.workers)

    def close(self) -> None:
        for worker in self.workers:
            worker.stop()
        for worker in self.workers:
            worker.join(timeout=2.0)


class _MissingCamera(_CameraWorker):
    """Placeholder for a camera whose device could not be resolved."""

    def __init__(self, spec: CameraSpec, reason: str) -> None:
        super().__init__(spec)
        self.status = "unavailable"
        self.detail = reason

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def join(self, timeout: float | None = None) -> None:
        pass

    def run(self) -> None:  # pragma: no cover - never a real thread
        raise RuntimeError("placeholder camera has no capture loop")


def parse_camera_specs(values: list[str]) -> tuple[CameraSpec, ...]:
    specs = tuple(parse_camera_spec(value) for value in values)
    keys = [spec.key for spec in specs]
    if len(set(keys)) != len(keys):
        raise ValueError(f"duplicate camera keys: {keys}")
    return specs
