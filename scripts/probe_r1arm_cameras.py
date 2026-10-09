#!/usr/bin/env python3
"""List the cameras attached to the R1Arm workstation and test which deliver frames.

Video node numbers change on every replug, so run this after moving a camera and
copy the printed ``--camera`` specs into ``record_r1arm_real.py``.

    python scripts/probe_r1arm_cameras.py
    python scripts/probe_r1arm_cameras.py --timeout 4
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402


#: RealSense devices hide several nodes behind one USB device; only the color
#: node is useful here, and it is driven by librealsense rather than V4L2.
REALSENSE_MATCH = "RealSense"


def device_name(index: int) -> str:
    try:
        return Path(f"/sys/class/video4linux/video{index}/name").read_text().strip()
    except OSError:
        return "?"


def usb_id(index: int) -> str:
    link = os.path.realpath(f"/sys/class/video4linux/video{index}/device")
    return Path(link).name


def by_id_paths() -> dict[int, str]:
    mapping: dict[int, str] = {}
    by_id = Path("/dev/v4l/by-id")
    if not by_id.is_dir():
        return mapping
    for entry in sorted(by_id.iterdir()):
        target = os.path.realpath(entry)
        if not target.startswith("/dev/video"):
            continue
        try:
            mapping[int(target.removeprefix("/dev/video"))] = str(entry)
        except ValueError:
            continue
    return mapping


def grab_with_timeout(
    device: str, fourcc: str, width: int, height: int, timeout: float
) -> tuple[np.ndarray | None, str]:
    """Try one frame; ``cv2.read`` blocks, so run it in a thread."""
    result: dict[str, object] = {"frame": None, "note": ""}

    def work() -> None:
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            result["note"] = "cannot open"
            return
        if fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = capture.read()
        if ok and frame is not None and frame.size:
            result["frame"] = frame
            result["note"] = f"{fourcc or 'auto'} {frame.shape[1]}x{frame.shape[0]}"
        else:
            result["note"] = f"{fourcc or 'auto'} no frame"
        capture.release()

    thread = threading.Thread(target=work, daemon=True)
    thread.start()
    thread.join(timeout)
    if thread.is_alive():
        return None, "timeout"
    return result["frame"], str(result["note"])  # type: ignore[return-value]


def probe_v4l2(limit: int, timeout: float) -> list[str]:
    paths = by_id_paths()
    working: list[str] = []
    print("\n== V4L2 nodes ==")
    for index in range(limit):
        device = f"/dev/video{index}"
        if not os.path.exists(device):
            continue
        if not os.access(device, os.R_OK | os.W_OK):
            print(f"{device}: no permission (add the user to the 'video' group)")
            continue
        name = device_name(index)
        note = ""
        if REALSENSE_MATCH in name:
            note = "  <- RealSense; use backend=realsense, not v4l2"
        for fourcc, width, height in (("MJPG", 1280, 720), ("MJPG", 640, 480), ("YUYV", 640, 480)):
            frame, detail = grab_with_timeout(device, fourcc, width, height, timeout)
            if frame is not None:
                print(f"{device}: OK  {detail}  [{name}]{note}")
                if note == "":
                    working.append(
                        f"key=REPLACE_ME,backend=v4l2,device={paths.get(index, device)},"
                        f"size={frame.shape[1]}x{frame.shape[0]},fps=30"
                        + (f",fourcc={fourcc}" if fourcc else "")
                    )
                break
        else:
            print(f"{device}: no frame via V4L2  [{name}]{note}")
    return working


def probe_realsense(timeout: float) -> list[str]:
    print("\n== RealSense (librealsense) ==")
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("pyrealsense2 is not installed: conda run -n r1_teleop pip install pyrealsense2")
        return ["key=REPLACE_ME,backend=realsense,device=auto,size=640x480,fps=30"]

    devices = list(rs.context().query_devices())
    if not devices:
        print("no RealSense device enumerated")
        return []

    specs: list[str] = []
    for device in devices:
        serial = device.get_info(rs.camera_info.serial_number)
        usb = device.get_info(rs.camera_info.usb_type_descriptor)
        print(f"{device.get_info(rs.camera_info.name)} sn={serial} usb={usb}")
        if usb.startswith("2"):
            print("  warning: linked at USB 2.x; the D405 color stream is unreliable")
            print("           there. Move it to a blue USB 3.x port if frames stall.")
        for pixel_format in ("bgr8", "yuyv"):
            if _try_realsense(rs, serial, pixel_format, timeout):
                specs.append(
                    f"key=REPLACE_ME,backend=realsense,device={serial},size=640x480,fps=30"
                )
                break
        else:
            print(f"  no color frame from {serial} (try a different USB port, then replug)")
    return specs


def _try_realsense(rs, serial: str, pixel_format: str, timeout: float) -> bool:
    attempts = 4
    for attempt in range(attempts):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        try:
            config.enable_stream(
                rs.stream.color, 640, 480, getattr(rs.format, pixel_format), 30
            )
            pipeline.start(config)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                frames = pipeline.wait_for_frames(timeout_ms=700)
                color = frames.get_color_frame()
                if color:
                    array = color.get_data()
                    print(
                        f"  OK {pixel_format} {color.get_width()}x{color.get_height()} "
                        f"(attempt {attempt + 1})"
                    )
                    return True
        except Exception as exc:  # noqa: BLE001 - timeout is the normal failure
            pass
        finally:
            try:
                pipeline.stop()
            except Exception:  # noqa: BLE001
                pass
    print(f"  {pixel_format}: no frame after {attempts} attempts")
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=int, default=12, help="Highest /dev/videoN to test")
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds per format attempt")
    parser.add_argument("--skip-realsense", action="store_true")
    args = parser.parse_args()

    print(f"python: {sys.executable}")
    specs = probe_v4l2(args.limit, args.timeout)
    if not args.skip_realsense:
        specs += probe_realsense(args.timeout)

    print("\n== Suggested --camera arguments ==")
    if not specs:
        print("(none; no camera delivered a frame)")
    for index, spec in enumerate(specs):
        key = "color_0_head_left" if index == 0 else "color_3_wrist_right"
        print(f"  --camera {spec.replace('REPLACE_ME', key)}")


if __name__ == "__main__":
    main()
