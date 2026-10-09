#!/usr/bin/env python3
"""Record one real right R1Arm (7 joints + optional Dex1) to compact MCAP.

The MCAP topics and frame dictionaries remain compatible with the existing
R1-A7/R1Arm recorder.  Only the controlled vector is compacted to the real
right arm: J1..J7 plus one logical Dex1 position (8 values total).

Cameras are read from the devices wired to this workstation instead of the
R1-A7 ZMQ image server.  ``--camera`` selects them; the defaults are the UVC
scene camera and the Intel RealSense D405 mounted on the arm.  Frames are
stored under the R1-A7 ``color_*`` keys so episodes stay readable by
``scripts/replay_r1arm_sim.py``.

A camera that never delivers a frame is dropped *before* the episode opens and
recorded in the metadata, because one sample must carry one image per camera.
At least one working camera is required; run ``scripts/probe_r1arm_cameras.py``
if a device went missing.

Pico controls (X/Y live on the LEFT controller, A/B on the RIGHT controller):
  X: start/stop and save an episode (--record-button, default)
  B: pause/resume recording and XR tracking (--pause-button, default)
  A: soft-reset the arm to home (requires --enable-a-reset)
  right grip/trigger: arm tracking and Dex1 control

Single right-arm teleop only needs the right controller; if the left controller
is not paired/awake, X never fires. Pass ``--record-button A`` (or ``B``) to
toggle recording from the right controller instead.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from mcap.reader import make_reader


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.r1arm_config import ARM_JOINTS, DEX1_SIM_CLOSE, DEX1_SIM_OPEN, R1ArmConfig
from robot.r1arm_sdk import (
    DEFAULT_BRIDGE_LIBRARY,
    DEFAULT_SDK_ROOT,
    as_gain_array,
    format_gain,
    parse_gain,
    sdk_to_model_q,
    validate_calibration_file,
)
from teleop.local_camera_client import (
    CAMERA_KEY_CHOICES,
    DEFAULT_CAMERAS,
    CameraSpec,
    LocalCameraSubscriber,
    parse_camera_specs,
)
from teleop.mcap_writer import FourCameraMcapRecorder
from teleop.r1arm_dataset import R1ArmDatasetAdapter
from teleop.r1arm_real_controller import R1ArmRealTeleopController


PREVIEW_WINDOW = "R1Arm real record: head / wrist"

# Controller button names accepted by --record-button / --pause-button,
# with which physical PICO controller each one lives on.
_BUTTON_CONTROLLER = {
    "X": "left",
    "Y": "left",
    "A": "right",
    "B": "right",
    "left_menu_button": "left",
    "right_menu_button": "right",
    "left_axis_click": "left",
    "right_axis_click": "right",
}
RECORD_BUTTON_CHOICES = tuple(_BUTTON_CONTROLLER)
# Pause may be disabled entirely (e.g. when only the right controller is used
# and its two buttons are needed for record + reset).
PAUSE_BUTTON_CHOICES = RECORD_BUTTON_CHOICES + ("none",)


def _dex1_open_ratio_to_sim(open_ratio: float) -> float:
    open_ratio = float(np.clip(open_ratio, 0.0, 1.0))
    return DEX1_SIM_CLOSE + open_ratio * (DEX1_SIM_OPEN - DEX1_SIM_CLOSE)


def _labeled(frame: np.ndarray, label: str) -> np.ndarray:
    image = frame.copy()
    cv2.putText(
        image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
        (0, 0, 0), 4, cv2.LINE_AA,
    )
    cv2.putText(
        image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
        (0, 255, 0), 2, cv2.LINE_AA,
    )
    return image


def _horizontal_preview(frames: tuple[np.ndarray, ...], labels: list[str]) -> np.ndarray:
    images = [_labeled(frame, label) for frame, label in zip(frames, labels)]
    if not images:
        raise ValueError("no camera frames to preview")
    common_height = min(image.shape[0] for image in images)
    resized = []
    for image in images:
        if image.shape[0] == common_height:
            resized.append(image)
            continue
        scale = common_height / image.shape[0]
        resized.append(
            cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        )
    return np.hstack(resized)


_PREVIEW_TILE_SIZE = (480, 640)  # (h, w) for placeholder tiles


def _placeholder_tile(health: tuple[str, float], view: str) -> np.ndarray:
    """Black tile naming a camera that currently contributes no frame."""
    status, age = health
    age_text = f"{age:.1f}s ago" if np.isfinite(age) else "no frame yet"
    tile = np.zeros((*_PREVIEW_TILE_SIZE, 3), dtype=np.uint8)
    lines = [
        "NO SIGNAL",
        view,
        f"status: {status}",
        age_text,
    ]
    for line_no, text in enumerate(lines):
        color = (0, 0, 255) if line_no == 0 else (200, 200, 200)
        cv2.putText(
            tile, text, (40, 120 + line_no * 48), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, color, 2, cv2.LINE_AA,
        )
    return tile


def _frozen_overlay(frame: np.ndarray) -> np.ndarray:
    """Stamp a frozen-image warning over the (still shown) last frame."""
    image = frame.copy()
    cv2.putText(
        image, "FROZEN IMAGE", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
        (0, 0, 0), 4, cv2.LINE_AA,
    )
    cv2.putText(
        image, "FROZEN IMAGE", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
        (0, 0, 255), 2, cv2.LINE_AA,
    )
    return image


def _stale_camera_summary(camera: LocalCameraSubscriber, max_age: float) -> str:
    """Name the exact camera blocking recording (not just 'waiting...')."""
    if camera is None:
        return "waiting for a fresh camera packet"
    parts = []
    for key, (status, age) in camera.health().items():
        if np.isfinite(age) and age <= max_age and status != "frozen":
            continue
        if np.isfinite(age):
            parts.append(f"{key} ({status}, no change for {age:.1f}s)")
        else:
            parts.append(f"{key} ({status}, no frame yet)")
    if not parts:
        return "waiting for a fresh selected-camera packet"
    return "stale camera: " + "; ".join(parts) + " — samples are being skipped"


class R1ArmRealMcapRecord:
    """Run the real-arm hold/teleop loop and asynchronously write MCAP frames."""

    def __init__(self, args: argparse.Namespace, calibration: Path) -> None:
        self.args = args
        self.running = True
        self.started = False
        # Teleoperation is available immediately; X is reserved for recording.
        self.teleop_active = True
        self.is_recording = False
        self.record_paused = False
        self.record_frame_count = 0
        self.record_start_monotonic = 0.0
        self.record_pause_started_monotonic = 0.0
        self.record_paused_duration = 0.0
        self.next_record_tick = 0.0
        self.active_episode_path: str | None = None
        self.previous_buttons = {args.record_button: False}
        if args.pause_button != "none":
            self.previous_buttons[args.pause_button] = False
        self.writer_lock = threading.Lock()
        self.record_stop_event = threading.Event()
        self.record_thread: threading.Thread | None = None
        self.record_error: BaseException | None = None
        self.preview_created = False
        self.preview_broken = False
        self._preview_last_render = 0.0
        # P2 修复抖动：预览彻底移出控制线程。控制循环只无锁读取共享的
        # 最近帧引用（原子替换），渲染（拼图/imshow/waitKey/Qt 事件泵）
        # 全在独立 preview 线程——GIL/周期争抢不再进入 20ms 控制预算。
        self._preview_state: dict | None = None  # 渲染线程读，控制线程写
        self._preview_quit = False
        self.preview_thread: threading.Thread | None = None
        self.last_stale_warning = 0.0

        config = R1ArmConfig(
            control_side="right",
            scale_factor=args.scale_factor,
            reset_blend_frames=args.reset_blend_frames,
        )
        if args.urdf:
            config.urdf_path = str(Path(args.urdf).expanduser().resolve())
        self.config = config
        self.controller = R1ArmRealTeleopController(
            config=config,
            port=args.port,
            baudrate=args.baud,
            calibration_path=calibration,
            bridge_library=args.bridge_lib,
            motion_mode=args.motion_mode,
            enable_dex1=args.enable_dex1,
            dex1_interface=args.dex1_net,
            dex1_domain=args.dex1_domain,
            grip_torque_limit=args.grip_torque_limit,
            kp=args.kp,
            kd=args.kd,
            rate=args.rate,
            max_joint_speed=args.max_joint_speed,
            max_command_lead=args.max_command_lead,
            feedback_timeout=args.feedback_timeout,
            foc_timeout=args.foc_timeout,
            home_on_start=args.home_on_start,
            mock_xr=args.mock_xr,
            debug=args.debug,
            scale_factor=args.scale_factor,
            gravity_ff=args.gravity_ff,
            tau_clip=args.tau_clip,
            ik_dt=args.ik_dt,
        )
        self.controller.enable_a_button_reset = bool(args.enable_a_reset)
        self.adapter = R1ArmDatasetAdapter(dual_arm=False, control_side="right")

        if args.no_cameras:
            self.camera_specs = ()
            self.camera: LocalCameraSubscriber | None = None
            self.camera_keys: tuple[str, ...] = ()
        else:
            self.camera_specs = parse_camera_specs(list(args.camera))
            self.camera = LocalCameraSubscriber(self.camera_specs)
            self.camera_keys = tuple(spec.key for spec in self.camera_specs)
        # Filled in by run() once the live cameras are known.
        self.writer: FourCameraMcapRecorder | None = None
        self.task_dir = self._task_dir()
        print(f"[Record] MCAP episodes go to {self.task_dir}")

    def _task_dir(self) -> Path:
        if self.args.output_dir:
            return Path(self.args.output_dir).expanduser().resolve()
        date = dt.datetime.now().strftime("%Y_%m_%d")
        return (
            Path(self.args.data_root).expanduser()
            / self.args.task_dir_name
            / f"{date}_robot_{self.args.robot_id}_dex1"
        )

    def _camera_snapshot(
        self,
    ) -> tuple[tuple[np.ndarray, ...] | None, int, int, tuple[str, ...]]:
        if self.camera is None:
            # Empty tuple = "no cameras" (valid, state-only recording);
            # None means "cameras exist but no coherent frame yet".
            return (), -1, 0, ()
        return self.camera.get()

    def _camera_is_fresh(self, received_at_ns: int) -> bool:
        if self.camera is None or self.args.max_image_age <= 0:
            return True
        return (time.time_ns() - received_at_ns) / 1e9 <= self.args.max_image_age

    def _episode_metadata(self) -> dict:
        _, _, _, source_streams = self._camera_snapshot()
        inactive = list(self.camera.inactive) if self.camera is not None else []
        return {
            "robot": "R1Arm",
            "eef": "dex1",
            "simulator": None,
            "motion_mode": bool(self.args.motion_mode),
            "teleop_stack": "R1ArmRealTeleopController",
            "dual_arm": False,
            "control_side": "right",
            "urdf_path": str(self.config.urdf_path),
            "serial_port": self.args.port if self.args.motion_mode else "",
            "camera_keys": list(self.camera_keys),
            "camera_count": len(self.camera_keys),
            "camera_source_streams": list(source_streams),
            "camera_inactive": [key for key, _ in inactive],
            "camera_devices": self.camera.describe() if self.camera is not None else "",
            "mcap_color_keys": list(self.camera_keys),
            "joint_names": list(self.adapter.joint_names),
            "state_dim": self.adapter.vector_size,
            "action_dim": self.adapter.vector_size,
            "gripper_dof_per_arm": 1,
            "state_layout": "right_arm(7)+right_gripper(1)",
            "sim_q_layout": "right_arm(7)+right_hand_fingers(2)",
            # Teleop-side output shaping, so replay can reproduce the same
            # command stream instead of raw IK targets (which contain
            # activation transients the arm never physically executed).
            "teleop_rate_hz": float(self.controller.rate_hz),
            "teleop_max_joint_speed": float(
                self.controller.max_delta * self.controller.rate_hz
            ),
            "teleop_max_command_lead": float(self.controller.max_command_lead),
            "teleop_kp": float(self.controller.kp) if np.isscalar(self.controller.kp) else None,
            "teleop_kd": float(self.controller.kd) if np.isscalar(self.controller.kd) else None,
            "teleop_kp_list": [float(v) for v in as_gain_array(self.controller.kp)],
            "teleop_kd_list": [float(v) for v in as_gain_array(self.controller.kd)],
            "teleop_gravity_ff": bool(self.controller.gravity_ff),
            "teleop_tau_clip": float(self.controller.tau_clip),
            # P2 整形器参数属于部署契约：整形后的 action 语义由它们定义，
            # 部署端要用同一组参数消费策略输出。
            "ik_max_acc_per_s2": float(self.controller.config.ik_max_acc_per_s2),
            "ik_max_step_per_cycle": (
                None if self.controller.config.ik_max_step_per_cycle is None
                else float(self.controller.config.ik_max_step_per_cycle)
            ),
            "ik_qp_velocity_limit": self.controller.config.ik_qp_velocity_limit,
            # P3 字段契约（训练转换据此选字段，不必读代码）：
            #   actions.action  = 训练主字段（ik_target 语义）：整形后的 IK
            #                     关节目标，模型坐标系，J4 取负（URDF 符号），
            #                     [J1..J7, gripper] 8 维；gripper 单位为 sim
            #                     值（+0.0245=开, -0.02=闭合），非 0/1。
            #   states.joint_state = 电机实测（measured_q 语义），同坐标系。
            #   actions.robot_q_desired / hand_cmd 等为同值冗余副本（兼容读）。
            #   每帧 camera_frame_times_ns = 各路图像到达时刻（对齐测量用）。
            "field_contract": {
                "train_action": "actions.action",
                "train_state": "states.joint_state",
                "diagnostics": "frame.camera_frame_times_ns",
                "frame": "model",
                "j4_sign": "negative (URDF convention)",
                "gripper_unit": "sim value +0.0245 open / -0.02 closed",
            },
        }

    def _active_duration(self, now: float | None = None) -> float:
        if self.record_start_monotonic <= 0:
            return 0.0
        now = time.monotonic() if now is None else now
        paused = self.record_paused_duration
        if self.record_paused and self.record_pause_started_monotonic > 0:
            paused += now - self.record_pause_started_monotonic
        return max(0.0, now - self.record_start_monotonic - paused)

    def _start_episode(self) -> None:
        if self.is_recording or self.writer is None:
            return
        frames, _, received_at_ns, _ = self._camera_snapshot()
        if frames is None:
            print(
                "[Record] cannot start: "
                f"{_stale_camera_summary(self.camera, self.args.max_image_age)}"
            )
            return
        if not self._camera_is_fresh(received_at_ns):
            print(
                "[Record] cannot start: "
                f"{_stale_camera_summary(self.camera, self.args.max_image_age)}"
            )
            return
        with self.writer_lock:
            path = self.writer.start_episode(self._episode_metadata())
        self.active_episode_path = path
        self.is_recording = True
        self.record_paused = False
        self.record_frame_count = 0
        self._action_samples: list[np.ndarray] = []
        self._gripper_samples: list[float] = []
        self._breakpoints: list[tuple[int, float]] = []  # P3: (帧数, 单调钟)
        self.record_start_monotonic = time.monotonic()
        self.record_pause_started_monotonic = 0.0
        self.record_paused_duration = 0.0
        self.next_record_tick = self.record_start_monotonic
        print(f"[Record] START episode {self.writer.episode_id}: {path}")

    def _stop_episode(self) -> None:
        if self.writer is None:
            return
        if not self.is_recording and self.writer.writer is None:
            return
        duration = max(self._active_duration(), 1e-9)
        self.is_recording = False
        self.record_paused = False
        self.record_pause_started_monotonic = 0.0
        with self.writer_lock:
            self.writer.stop_episode()
        fps = self.record_frame_count / duration
        print(
            f"[Record] SAVED {self.active_episode_path} | "
            f"frames={self.record_frame_count} duration={duration:.2f}s avg_fps={fps:.1f}"
        )
        if self.record_frame_count == 0:
            print(
                "[Record] WARNING: this episode has NO frames at all — the "
                "camera stream was dead the whole time; do not use it"
            )
        elif fps < self.args.frequency * 0.9:
            slowest = self._slowest_camera()
            reason = f" (slowest camera: {slowest})" if slowest else ""
            print(
                f"[Record] WARNING: effective {fps:.1f} Hz is below 90% of "
                f"requested {self.args.frequency:.1f} Hz{reason}"
            )
        if self.camera is not None and self.camera.inactive:
            print(
                "[Record] WARNING: cameras dropped this run: "
                f"{[key for key, _ in self.camera.inactive]} — episodes lack those views"
            )
        frozen = self._frozen_cameras()
        if frozen:
            print(
                "[Record] WARNING: cameras currently streaming frozen content: "
                f"{frozen} — their images repeat and carry no information"
            )
        self._report_action_smoothness()
        self._report_gripper_channel()
        self._report_breakpoints()
        self._report_camera_alignment()

    def _report_breakpoints(self) -> None:
        """P3: where pauses split the frame sequence (chunking boundaries).

        A pause-resume pair inside one episode is a real time discontinuity:
        the gap in ``sample_time_ns`` grows to the pause length while ``idx``
        stays contiguous. Training conversions should cut action chunks at
        these indices instead of silently spanning the break.
        """
        bps = getattr(self, "_breakpoints", None) or []
        if not bps:
            return
        pairs = [bps[i:i + 2] for i in range(0, len(bps), 2)]
        print(f"[Record] pause breakpoints ({len(pairs)} 处，切 chunk 边界):")
        for i, (a, b) in enumerate(pairs):
            span = b[1] - a[1]
            print(f"  #{i + 1}: 帧 {a[0]} 暂停 {span:.1f}s 后继续（chunk 应在 "
                  f"{a[0]} 与 {b[0]} 处切开）")

    def _report_camera_alignment(self) -> None:
        """P3: image-vs-sample offset stats from camera_frame_times_ns.

        Reads the just-written episode's last frame's per-camera times and
        reports the offset each camera's image sat at relative to the sample
        timestamp. Full per-episode stats belong to the offline audit script;
        this is the in-recording sanity line.
        """
        if self.camera is None:
            return
        try:
            with open(self.active_episode_path, "rb") as f:
                last = None
                for _, _, message in make_reader(f).iter_messages():
                    msg = json.loads(message.data.decode())
                    if "camera_frame_times_ns" in msg:
                        last = msg
                if last is None or not last["camera_frame_times_ns"]:
                    return
                st = int(last["sample_time_ns"])
                print("[Record] camera alignment（末帧）: "
                      + ", ".join(
                          f"{k} {max(0.0, (st - int(v)) / 1e9) * 1000:.0f}ms"
                          for k, v in last["camera_frame_times_ns"].items()))
        except Exception as exc:
            if self.args.debug:
                print(f"[Record] alignment report skipped: {exc}")

    def _report_gripper_channel(self) -> None:
        """Warn when the gripper dimension carries no information.

        Two failure modes produce a constant gripper action: Dex1 not enabled
        at all (fallback = the sim open constant 0.0245) and Dex1 enabled but
        the trigger never pressed. Both mean the 8th action dim teaches the
        policy nothing; they must not pass silently.
        """
        grip_samples = getattr(self, "_gripper_samples", None)
        if not grip_samples or len(grip_samples) < 5:
            return
        grip = np.asarray(grip_samples, dtype=float)
        span = float(grip.max() - grip.min())
        unique = len(np.unique(np.round(grip, 4)))
        note = (
            "Dex1 NOT enabled — the 0.0245 constant is the sim fallback value, "
            "not hardware feedback; add --enable-dex1 --dex1-net <nic>"
            if self.controller.hand_ctrl is None
            else "Dex1 enabled but trigger never moved — check grip wiring"
        )
        if span < 1e-4:
            print(
                f"[Record] WARNING: gripper action is constant "
                f"({grip[0]:.4f}, {unique} unique value) — {note}"
            )
        else:
            print(f"[Record] gripper action: span {span:.3f} rad, {unique} unique values OK")

    def _report_action_smoothness(self) -> None:
        """Per-episode action quality gate: velocity/acceleration p95 by joint.

        The recording rate (--frequency) defines the sample interval; spikes
        here mean the recorded action stream carries jerk a VLA would have to
        reproduce — or a teleop/integration fault worth inspecting.
        """
        samples = getattr(self, "_action_samples", None)
        if not samples or len(samples) < 5:
            return
        q = np.asarray(samples, dtype=float)[:, :7]
        hz = max(float(self.args.frequency), 1e-6)
        names = ARM_JOINTS
        v = np.abs(np.diff(q, axis=0)) * hz
        a = np.abs(np.diff(v, axis=0)) * hz
        print(f"[Record] action smoothness (q→v→a, {len(q)} samples @ {hz:.0f} Hz):")
        print(f"  {'joint':<6}{'|v|p95':>9}{'|a|p95':>10}{'|a|max':>9}")
        for j in range(min(7, q.shape[1])):
            print(f"  {names[j]:<6}{np.percentile(v[:, j], 95):>9.3f}"
                  f"{np.percentile(a[:, j], 95):>10.2f}{a[:, j].max():>9.2f}")
        worst = float(np.percentile(a, 95))
        if worst > 25.0:
            print(
                f"[Record] WARNING: action acceleration p95 = {worst:.1f} rad/s² "
                f"(>25) — this episode is jerky; check the shaper (ik_max_acc) and "
                "inspect before including in training"
            )

    def _slowest_camera(self) -> str | None:
        if self.camera is None:
            return None
        worst = None
        worst_age = -1.0
        for key, (_, age) in self.camera.health().items():
            if np.isfinite(age) and age > worst_age:
                worst, worst_age = key, age
        return worst

    def _frozen_cameras(self) -> list[str]:
        if self.camera is None:
            return []
        return [
            key for key, (status, _) in self.camera.health().items()
            if status == "frozen"
        ]

    def _toggle_pause(self) -> None:
        if not self.is_recording:
            print("[Record] B ignored: no active episode")
            return
        now = time.monotonic()
        self.record_paused = not self.record_paused
        if self.record_paused:
            self.record_pause_started_monotonic = now
            # P3 断点：记录暂停发生处——训练转换时按这些 idx 切 chunk
            #（暂停两侧的 action chunk 跨越真实时间断点，直接拼接会产生
            # 隐式不连续）。恢复侧同样记（chunk 在恢复点开始新段）。
            self._breakpoints.append((self.record_frame_count, now))
            print("[Record] PAUSED (recording and XR tracking paused; motor hold remains active)")
        else:
            self.record_paused_duration += now - self.record_pause_started_monotonic
            self.record_pause_started_monotonic = 0.0
            self.next_record_tick = now
            self._breakpoints.append((self.record_frame_count, now))
            for track in self.controller.track.values():
                track.clear()
            print("[Record] RESUMED")

    def _button_edges(self) -> dict[str, bool]:
        edges = {}
        for name in self.previous_buttons:
            pressed = self.controller.get_button_state(name)
            edges[name] = pressed and not self.previous_buttons[name]
            self.previous_buttons[name] = pressed
        return edges

    def _handle_buttons(self) -> None:
        edges = self._button_edges()
        if edges[self.args.record_button]:
            if self.is_recording:
                self._stop_episode()
            else:
                self._start_episode()
        if self.args.pause_button != "none" and edges[self.args.pause_button]:
            self._toggle_pause()

    def _gripper_state_command(self) -> tuple[float, float]:
        desired = float(np.mean(self.controller.dex1_q("right")))
        hand = self.controller.hand_ctrl
        if hand is None:
            return desired, desired
        try:
            record = hand.get_record_data()
            measured = _dex1_open_ratio_to_sim(float(record["hand_state"][0]))
            commanded = _dex1_open_ratio_to_sim(float(record["hand_cmd"][0]))
            return measured, commanded
        except Exception as exc:
            if self.args.debug:
                print(f"[Record] Dex1 feedback fallback to command: {exc}")
            return desired, desired

    def _state_action(self) -> tuple[dict, dict]:
        commanded_arm = self.controller.arm_q("right").copy()
        state = self.controller.last_state
        if self.args.motion_mode and state is not None:
            measured_arm = sdk_to_model_q(np.asarray(state.q, dtype=float).copy())
        else:
            measured_arm = commanded_arm.copy()
        measured_gripper, commanded_gripper = self._gripper_state_command()
        measured_sim_q = np.hstack(
            [measured_arm, measured_gripper, measured_gripper]
        )
        commanded_sim_q = np.hstack(
            [commanded_arm, commanded_gripper, commanded_gripper]
        )
        # P2/P3 数据质检缓冲：记录写盘的 action 前七个关节（模型系），
        # 保存时算加速度 p95 —— 挑出毛刺 episode，不进正式训练集。
        # 第 8 维（夹爪）单独收：常数夹爪是无信息维度，保存时点名。
        self._action_samples.append(commanded_arm)
        self._gripper_samples.append(commanded_sim_q[7])
        return self.adapter.frame_data(measured_sim_q, commanded_sim_q)

    def _record_loop(self) -> None:
        interval = 1.0 / max(float(self.args.frequency), 1.0)
        while not self.record_stop_event.is_set():
            now = time.monotonic()
            if not self.is_recording or self.record_paused:
                time.sleep(0.005)
                continue
            if now < self.next_record_tick:
                time.sleep(min(self.next_record_tick - now, 0.005))
                continue
            while self.next_record_tick <= now:
                self.next_record_tick += interval

            frames, frame_id, received_at_ns, _ = self._camera_snapshot()
            if frames is None or not self._camera_is_fresh(received_at_ns):
                if now - self.last_stale_warning >= 1.0:
                    print(f"[Record] {_stale_camera_summary(self.camera, self.args.max_image_age)}")
                    self.last_stale_warning = now
                continue
            try:
                states, actions = self._state_action()
                # P3 对齐：每路相机的帧到达时刻与样本时刻一起落盘，
                # 图像-action 偏移从"假设"变为可测量（质检统计用）。
                cam_times = self.camera.frame_times() if self.camera is not None else {}
                with self.writer_lock:
                    if self.is_recording and not self.record_paused and self.writer is not None:
                        self.writer.add_frame(
                            frames,
                            states,
                            actions,
                            image_source_frame_id=frame_id,
                            sample_time_ns=time.time_ns(),
                            camera_frame_times_ns=cam_times,
                        )
                        self.record_frame_count += 1
            except BaseException as exc:
                self.record_error = exc
                self.running = False
                return

    def _start_preview_thread(self) -> None:
        """Render the preview in a dedicated thread (P2 fix for teleop jitter).

        Rendering in the control loop (even rate-limited) competes for the GIL
        with IK/serial/record threads and pumps Qt events on the control
        thread's schedule — measured as control-cycle jitter that the arm
        renders as audible click/jerk. The preview thread owns all cv2 GUI
        work; the control loop never touches cv2.
        """
        if not self.args.image_show or self.camera is None:
            return

        def _loop() -> None:
            period = 1.0 / max(float(self.args.preview_hz), 0.5)
            next_render = time.monotonic()
            while self.running and not self._preview_quit:
                now = time.monotonic()
                if now < next_render:
                    time.sleep(min(next_render - now, 0.01))
                    # pump GUI events while waiting so the window stays live
                    if self.preview_created and not self.preview_broken:
                        try:
                            if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                                self._preview_quit = True
                                self.running = False
                                break
                        except cv2.error:
                            self.preview_broken = True
                    continue
                next_render = now + period
                state = self._preview_state  # atomic read
                if state is None:
                    continue
                try:
                    if not self.preview_created:
                        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
                        self.preview_created = True
                    cv2.imshow(PREVIEW_WINDOW, state["preview"])
                except cv2.error:
                    self.preview_broken = True
                    print(
                        "[Record] camera preview disabled (GUI unavailable); "
                        "recording continues"
                    )
                    continue

        self.preview_thread = threading.Thread(
            target=_loop, name="r1arm-preview", daemon=True
        )
        self.preview_thread.start()

    def _publish_preview_state(self) -> None:
        """Control-thread side: build (cheap) and publish the preview frame.

        Only the tile concatenation + resize happen here (sub-millisecond on
        640x480 pairs); imshow/waitKey/Qt stay in the preview thread. GUI
        errors are detected in the render thread, not here.
        """
        if not self.args.image_show or self.camera is None:
            return
        if self.preview_broken or self._preview_quit:
            return
        now = time.monotonic()
        preview_period = 1.0 / max(float(self.args.preview_hz), 0.5)
        if now - self._preview_last_render < preview_period:
            return
        self._preview_last_render = now
        frames, frame_id, _, _ = self._camera_snapshot()
        active = self.camera.active_keys or tuple(spec.key for spec in self.camera_specs)
        health = self.camera.health()
        status = "PAUSED" if self.record_paused else (
            "RECORDING" if self.is_recording else "LIVE"
        )
        images: list[np.ndarray] = []
        labels: list[str] = []
        if frames is None:
            for view in active:
                images.append(_placeholder_tile(
                    health.get(view, ("unknown", float("inf"))), view
                ))
                labels.append(f"{view} | {status}")
        else:
            for view, frame in zip(active, frames):
                cam_status, cam_age = health.get(view, ("unknown", float("inf")))
                if cam_status == "frozen":
                    images.append(_frozen_overlay(frame))
                else:
                    images.append(frame)
                labels.append(
                    f"{view} | {status} | packet {frame_id} | {cam_status}"
                    + (f" {cam_age:.1f}s" if np.isfinite(cam_age) else "")
                )
        preview = _horizontal_preview(tuple(images), labels)
        scale = max(float(self.args.image_preview_scale), 0.1)
        if scale != 1.0:
            preview = cv2.resize(
                preview, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        # Atomic publish; the render thread may skip a frame if we overlap.
        self._preview_state = {"preview": preview}

    def _start_cameras(self) -> None:
        """Start capture, drop dead cameras, and open the episode writer."""
        if self.camera is None:
            # --no-cameras: record state/action only, no image frames.
            self.writer = FourCameraMcapRecorder(
                task_dir=str(self.task_dir),
                task_goal=self.args.task_description,
                frequency=self.args.frequency,
                format_name="r1arm_real_multicamera_mcap_json_v1",
                camera_order=(),
            )
            print("[Camera] recording state/action only (no cameras)")
            return
        self.camera.start()
        self.camera.resolve_active(self.args.camera_wait)
        for key, detail in self.camera.inactive:
            print(f"[Camera] UNAVAILABLE {key}: {detail}")
            spec = next((s for s in self.camera_specs if s.key == key), None)
            if spec is not None and spec.backend == "realsense":
                print(
                    "[Camera]   hint: a RealSense color stream stalls when it is linked "
                    "at USB 2.0; move the camera to a USB 3.x port, replug it, then "
                    "re-run scripts/probe_r1arm_cameras.py"
                )
        if not self.camera.active_keys:
            raise SystemExit(
                "[Camera] no camera delivered a frame; "
                "run scripts/probe_r1arm_cameras.py and pass --camera"
            )
        if self.camera.inactive and self.args.require_all_cameras:
            raise SystemExit(
                "[Camera] --require-all-cameras was set but "
                f"{[key for key, _ in self.camera.inactive]} produced no frame"
            )
        # Only live cameras reach camera_order: the MCAP writer requires one
        # image per key on every recorded sample.
        self.camera_keys = tuple(self.camera.active_keys)
        for spec in self.camera_specs:
            if spec.key in self.camera_keys:
                print(f"[Camera] {spec.label()}")
        self.writer = FourCameraMcapRecorder(
            task_dir=str(self.task_dir),
            task_goal=self.args.task_description,
            frequency=self.args.frequency,
            format_name="r1arm_real_multicamera_mcap_json_v1",
            camera_order=self.camera_keys,
        )

    def run(self) -> None:
        try:
            self._start_cameras()
            self.controller._on_start()
            self.started = True
            self.controller.last_gripper_update = time.monotonic()
            self.record_thread = threading.Thread(
                target=self._record_loop, name="r1arm-real-mcap-record", daemon=True
            )
            self.record_thread.start()

            print("\n[R1Arm right-arm MCAP Record] ready")
            print("  Teleoperation is active immediately")
            print(f"  Pico {self.args.record_button} ({_BUTTON_CONTROLLER[self.args.record_button]}): "
                  "start/stop and save an episode")
            if self.args.pause_button != "none":
                print(
                    f"  Pico {self.args.pause_button} "
                    f"({_BUTTON_CONTROLLER[self.args.pause_button]}): "
                    "pause/resume recording and XR tracking"
                )
            if self.args.enable_a_reset:
                print("  Pico A (right): soft-reset the arm to home "
                      "(motion is recorded as part of the episode)")
            print("  Right grip: arm tracking; right trigger: Dex1")
            if self.camera is None:
                print("  Cameras: none (state/action only)")
            else:
                requested = [spec.key for spec in self.camera_specs]
                dropped = [key for key, _ in self.camera.inactive]
                print(f"  Cameras requested: {requested}")
                print(f"  Cameras active:    {list(self.camera_keys)}")
                if dropped:
                    print(
                        f"  Cameras DROPPED:   {dropped} "
                        "(this episode records WITHOUT them; see camera_inactive in metadata)"
                    )
            if self.args.auto_start:
                self._start_episode()
            self._start_preview_thread()

            control_dt = 1.0 / max(float(self.args.rate), 1.0)
            while self.running:
                tick = time.monotonic()
                self._handle_buttons()
                if (
                    (self.teleop_active and not self.record_paused)
                    or self.controller._reset_blending
                ):
                    self.controller.step(control_dt)
                else:
                    # In real mode this keeps polling feedback and refreshing
                    # the position hold, including while collection is paused.
                    self.controller._output_command()

                if self.record_error is not None:
                    raise RuntimeError("MCAP recording worker failed") from self.record_error
                now = time.monotonic()
                if self.args.duration > 0 and self.is_recording:
                    if self._active_duration(now) >= self.args.duration:
                        self._stop_episode()
                        if self.args.exit_after_duration:
                            self.running = False
                # 预览：控制线程只做轻量拼图+原子发布；imshow/Qt 在渲染线程
                self._publish_preview_state()
                remaining = control_dt - (time.monotonic() - tick)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self.running = False
        self._preview_quit = True
        self._stop_episode()
        self.record_stop_event.set()
        if self.record_thread is not None:
            self.record_thread.join(timeout=3.0)
            self.record_thread = None
        if self.preview_thread is not None:
            self.preview_thread.join(timeout=2.0)
            self.preview_thread = None
        if self.started:
            self.controller._on_stop()
            self.started = False
        else:
            # Constructor may have opened XR even if arm startup failed.
            self.controller._on_stop()
        if self.camera is not None:
            self.camera.close()
        if self.preview_created:
            try:
                cv2.destroyWindow(PREVIEW_WINDOW)
            except cv2.error:
                pass
        print("[R1Arm right-arm MCAP Record] stopped")


def confirm_motion(args: argparse.Namespace, calibration: Path) -> None:
    print("\nREAL R1Arm MOTION AND RECORDING ARE ABOUT TO BE ENABLED")
    print(f"  serial:       {args.port} @ {args.baud}")
    print(f"  calibration:  {calibration}")
    print(f"  gains:        kp={format_gain(args.kp)}, kd={format_gain(args.kd)}, "
          f"gravity_ff={'on' if args.gravity_ff else 'off'}")
    print(f"  max speed:    {args.max_joint_speed} rad/s")
    # FOC 三项合计（kp·lead + kd·|dq| + tau_clip）的保守上界 vs 电机峰值
    kp_arr = as_gain_array(args.kp)
    kd_arr = as_gain_array(args.kd)
    torque_cap = float(np.max(kp_arr * args.max_command_lead
                              + kd_arr * args.max_joint_speed
                              + (args.tau_clip if args.gravity_ff else 0.0)))
    print(f"  max lead:     {args.max_command_lead} rad → 力矩上界 {torque_cap:.1f} Nm"
          f"（三项合计；电机峰值 34.4 Nm 的 {torque_cap / 34.4 * 100:.0f}%）")
    print("  controller:   right PICO controller")
    print("Keep the emergency stop reachable and support the arm during startup probing.")
    if input("Type MOVE to continue: ").strip() != "MOVE":
        raise SystemExit("Cancelled; no real motor command was sent")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    teleop = parser.add_argument_group("right R1Arm real teleoperation")
    teleop.add_argument("--motion-mode", action="store_true")
    teleop.add_argument("--port", default="/dev/ttyUSB0")
    teleop.add_argument("--baud", type=int, default=6_000_000)
    teleop.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    teleop.add_argument("--calib", type=Path)
    teleop.add_argument("--bridge-lib", type=Path, default=DEFAULT_BRIDGE_LIBRARY)
    teleop.add_argument("--urdf", type=Path)
    teleop.add_argument("--scale-factor", type=float, default=1.0)
    teleop.add_argument("--rate", type=float, default=30.0,
                        help="控制环 Hz。默认 30（全链 30Hz 统一：每拍一条 action 一条 state）")
    teleop.add_argument(
        "--ik-dt", type=float, default=0.0,
        help="IK 积分步长 s；0 = 跟随 1/--rate（对齐）",
    )
    teleop.add_argument(
        "--gravity-ff", dest="gravity_ff", action="store_true", default=True,
        help="下发重力前馈力矩（指令姿态处计算，经 v2 tau 通道）；"
             "默认开（P0 三重实测：J4 悬垂误差 −70%）。--no-gravity-ff 关闭",
    )
    teleop.add_argument(
        "--no-gravity-ff", dest="gravity_ff", action="store_false",
        help="关闭重力前馈（回到 kp 硬扛悬垂的旧行为）",
    )
    teleop.add_argument(
        "--tau-clip", type=float, default=8.0,
        help="重力前馈单关节力矩限幅 Nm（默认 8）",
    )
    teleop.add_argument("--kp", type=parse_gain, default=80.0,
                        help="位置环增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    teleop.add_argument("--kd", type=parse_gain, default=2,
                        help="阻尼增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    teleop.add_argument("--max-joint-speed", type=float, default=1.0)
    teleop.add_argument(
        "--max-command-lead", type=float, default=0.15,
        help="命令相对实测的最大超前量 rad（默认 0.15）：异常单拍位移 ≤8.6°，"
             "力矩三项合计 ≈ 峰值一半；正常跟踪不触发",
    )
    teleop.add_argument("--feedback-timeout", type=float, default=3.0)
    teleop.add_argument("--foc-timeout", type=float, default=1.0)
    teleop.add_argument("--enable-dex1", action="store_true")
    teleop.add_argument("--dex1-net", default="")
    teleop.add_argument("--dex1-domain", type=int, default=0)
    teleop.add_argument("--grip-torque-limit", type=float, default=-1.0)
    teleop.add_argument("--enable-a-reset", action="store_true", help="Allow A to move toward ARM_INIT")
    teleop.add_argument(
        "--home-on-start",
        action="store_true",
        help="After startup FOC hold, soft-blend to ARM_INIT before enabling tracking",
    )
    teleop.add_argument(
        "--reset-blend-frames",
        type=int,
        default=60,
        help="A/home reset 时长（blend 按 wall-clock frames/50Hz 计秒；默认 60 ≈ 1.2s，旧 2s 传 100）",
    )
    teleop.add_argument("--mock-xr", action="store_true")
    teleop.add_argument("--debug", action="store_true")
    teleop.add_argument("--no-confirm", action="store_true")

    image = parser.add_argument_group("local cameras")
    image.add_argument(
        "--no-cameras",
        action="store_true",
        help="Record state/action only; skip all camera capture",
    )
    image.add_argument(
        "--camera",
        action="append",
        metavar="SPEC",
        default=None,
        help=(
            "Camera to record, repeatable; order sets the MCAP color order. "
            "Fields: key=<" + "|".join(CAMERA_KEY_CHOICES) + ">, "
            "backend=v4l2|realsense, device=<path|serial|auto>, "
            "match=<substring of /dev/v4l/by-id>, size=WxH, fps=N, fourcc=MJPG. "
            "Default: the UVC scene camera plus the D405 on the arm"
        ),
    )
    image.add_argument(
        "--camera-wait",
        type=float,
        default=8.0,
        help="Seconds to wait for every camera before dropping the dead ones",
    )
    image.add_argument(
        "--require-all-cameras",
        action="store_true",
        help="Abort instead of recording with a subset of the requested cameras",
    )
    image.add_argument("--max-image-age", type=float, default=0.5)
    image.add_argument(
        "--image-show",
        dest="image_show",
        action="store_true",
        default=None,
        help="Show a live camera preview while recording (default: on when $DISPLAY is set)",
    )
    image.add_argument(
        "--no-image-show",
        dest="image_show",
        action="store_false",
        help="Disable the camera preview even when a display is available",
    )
    image.add_argument("--image-preview-scale", type=float, default=0.75)
    image.add_argument(
        "--preview-hz", type=float, default=12.0,
        help="预览渲染频率 Hz（默认 12）。渲染在控制循环内执行，50Hz 满频会"
             "挤占控制周期预算导致遥操抖动；人眼监控 12 Hz 足够",
    )

    record = parser.add_argument_group("compact R1Arm MCAP recording")
    record.add_argument(
        "--record-button",
        choices=RECORD_BUTTON_CHOICES,
        default="X",
        help=(
            "Controller button that toggles episode start/stop. "
            "X/Y are on the left controller, A/B on the right controller. "
            "Use A or B if only the right controller is paired."
        ),
    )
    record.add_argument(
        "--pause-button",
        choices=PAUSE_BUTTON_CHOICES,
        default="B",
        help="Controller button that pauses/resumes recording; 'none' disables pause",
    )
    record.add_argument("--frequency", type=float, default=30.0)
    record.add_argument("--output-dir", default="")
    record.add_argument("--data-root", default=str(PROJECT_ROOT / "data"))
    record.add_argument("--task-dir-name", default="r1arm_real_task")
    record.add_argument("--task-description", default="R1Arm right-arm teleoperation")
    record.add_argument("--robot-id", default="r1arm_right")
    record.add_argument("--auto-start", action="store_true")
    record.add_argument("--duration", type=float, default=0.0)
    record.add_argument("--exit-after-duration", action="store_true")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.motion_mode and args.mock_xr:
        parser.error("--mock-xr cannot be combined with --motion-mode")
    if args.rate <= 0 or args.frequency <= 0:
        parser.error("--rate and --frequency must be positive")
    if args.no_cameras and args.camera:
        parser.error("--no-cameras cannot be combined with --camera")
    if args.pause_button != "none" and args.record_button == args.pause_button:
        parser.error("--record-button and --pause-button must be different buttons")
    if args.enable_a_reset and "A" in (args.record_button, args.pause_button):
        parser.error(
            "--enable-a-reset reserves A for soft-reset; A cannot also be "
            "--record-button or --pause-button"
        )
    if args.image_show is None:
        # Preview is on by default where a GUI exists: "the cameras recorded
        # nothing" used to be invisible during collection.
        args.image_show = bool(os.environ.get("DISPLAY"))
    if not args.no_cameras and not args.camera:
        args.camera = list(DEFAULT_CAMERAS)
    if not args.no_cameras:
        try:
            parse_camera_specs(list(args.camera))
        except ValueError as exc:
            parser.error(str(exc))
    if args.motion_mode and not args.bridge_lib.expanduser().is_file():
        parser.error(
            f"bridge library not found: {args.bridge_lib}; "
            "run python scripts/build_r1arm_sdk_bridge.py first"
        )
    if args.motion_mode and not Path(args.port).exists():
        parser.error(f"serial port not found: {args.port}")

    calibration = args.calib or args.sdk_root / "config/joint_calib.txt"
    if args.motion_mode:
        try:
            calibration = validate_calibration_file(calibration)
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
        if not args.no_confirm:
            confirm_motion(args, calibration)

    recorder = R1ArmRealMcapRecord(args, Path(calibration).expanduser().resolve())

    def stop(_signum, _frame) -> None:
        recorder.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    recorder.run()


if __name__ == "__main__":
    main()
