#!/usr/bin/env python3
"""Record R1-A7 + Dex1 teleoperation as four-camera MCAP episodes.

Uses the same real-robot teleop stack as ``scripts/teleop_real.py``
(``teleop.real_controller.RealTeleopController`` / ``r1a7_controller_base``).
A thin bridge runs that controller on a background thread and exposes the
``pc`` / ``pt`` buffers the recorder samples each frame.

Camera order is fixed for every saved frame:
  color_0_head_left, color_1_head_right, color_2_wrist_left, color_3_wrist_right

Pico controls:
  X: enter/leave teleoperation mode
  Y: start/stop and save an episode while teleoperation is active
  B: pause/resume writing the active episode (also pauses arm output)
  A: delete the most recently saved episode (idle only, once)

Threads:
  teleop         -- IK, gripper and DDS output (RealTeleopController.step)
  state          -- ~100 Hz hardware feedback into ``pc``
  record         -- JPEG encode + MCAP write at ``--frequency``
  main           -- buttons, episode state machine, optional preview

Usage:
    # Dry run (no hardware, no DDS publish); needs a reachable image server
    python scripts/record_real.py --mock-xr

    # Real robot
    python scripts/record_real.py --motion-mode --enable-dex1 --net enp5s0

Note: ``--image-show`` is a debugging aid. For real sessions leave it off and
verify recordings afterwards with ``scripts/replay_mcap.py``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.config import (
    DEX1_SIM_CLOSE,
    DEX1_SIM_OPEN,
    LEFT_DEX1_JOINTS,
    RIGHT_DEX1_JOINTS,
    R1A7Config,
)
from teleop.image_client import CAMERA_NAMES, FourCameraSubscriber
from teleop.mcap_writer import FourCameraMcapRecorder
from teleop.real_controller import RealTeleopController


MCAP_COLOR_KEYS = FourCameraMcapRecorder.CAMERA_ORDER
PREVIEW_WINDOW = "R1-A7 record: head L/R, wrist L/R"


def draw_label(frame: np.ndarray, label: str) -> np.ndarray:
    image = frame.copy()
    cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2, cv2.LINE_AA)
    return image


def _dex1_ratio_to_sim(ratio: float) -> float:
    """Map Dex1 0..1 open-normalized state to sim joint space."""
    open_ratio = float(np.clip(ratio, 0.0, 1.0))
    return DEX1_SIM_CLOSE + open_ratio * (DEX1_SIM_OPEN - DEX1_SIM_CLOSE)


class RealTeleopRecordBridge:
    """Background wrapper around ``RealTeleopController`` for MCAP recording.

    Same control stack as ``scripts/teleop_real.py``, plus the ``pc`` / ``pt``
    / ``can_tele_flag`` surface the recorder already expects.
    """

    def __init__(
        self,
        config: R1A7Config,
        *,
        net: str,
        motion_mode: bool,
        enable_dex1: bool,
        mock_xr: bool,
        visualize: bool,
        debug: bool,
        scale_factor: float,
        rate: float,
        waist_joystick_gain: float,
        waist_joystick_deadband: float,
        teleop_enabled: bool = False,
    ) -> None:
        self.rate = float(rate)
        self.can_tele_flag = bool(teleop_enabled)
        self.collect_start = False
        self.state_lock = threading.Lock()
        self._stop_event = threading.Event()

        self.controller = RealTeleopController(
            config=config,
            interface=net,
            motion_mode=motion_mode,
            enable_real_hand=bool(enable_dex1),
            mock_xr=mock_xr,
            visualize=visualize,
            debug=debug,
            scale_factor=scale_factor,
        )
        # A is "delete last episode" in the recorder.
        self.controller.enable_a_button_reset = False
        self.controller.waist_joystick_gain = float(waist_joystick_gain)
        self.controller.waist_joystick_deadband = float(waist_joystick_deadband)

        self.pc = {
            "body": np.zeros(4, dtype=float),
            "arms": np.zeros(14, dtype=float),
            "gripper": np.zeros(4, dtype=float),
        }
        self.pt = {
            "body": np.zeros(4, dtype=float),
            "arms": np.zeros(14, dtype=float),
            "gripper": np.zeros(4, dtype=float),
        }

        self.thread = threading.Thread(target=self._run_loop, name="r1-a7-record-teleop", daemon=True)
        self.state_thread = threading.Thread(
            target=self._update_state_loop, name="r1-a7-record-state", daemon=True
        )
        self.thread.start()
        self.state_thread.start()

    def _commanded_arms(self) -> np.ndarray:
        q = self.controller.q
        return np.hstack(
            [
                q[self.controller.left_arm_q_indices],
                q[self.controller.right_arm_q_indices],
            ]
        ).astype(float)

    def _commanded_gripper(self) -> np.ndarray:
        q = self.controller.q
        left = [
            float(q[self.controller.joint_indices[name]])
            for name in LEFT_DEX1_JOINTS
            if name in self.controller.joint_indices
        ]
        right = [
            float(q[self.controller.joint_indices[name]])
            for name in RIGHT_DEX1_JOINTS
            if name in self.controller.joint_indices
        ]
        while len(left) < 2:
            left.append(left[-1] if left else 0.0)
        while len(right) < 2:
            right.append(right[-1] if right else 0.0)
        return np.asarray(left[:2] + right[:2], dtype=float)

    def _measured_gripper(self) -> np.ndarray:
        hand = self.controller.hand_ctrl
        if hand is not None:
            try:
                left_state, right_state = hand.get_hand_states()
                left = _dex1_ratio_to_sim(float(left_state[0]))
                right = _dex1_ratio_to_sim(float(right_state[0]))
                return np.array([left, left, right, right], dtype=float)
            except Exception:
                pass
        return self._commanded_gripper()

    def _publish_pt(self) -> None:
        arms = self._commanded_arms()
        gripper = self._commanded_gripper()
        waist = float(self.controller.waist_command)
        with self.state_lock:
            self.pt["arms"] = arms
            self.pt["gripper"] = gripper
            self.pt["body"][0] = waist

    def _run_loop(self) -> None:
        dt = 1.0 / max(self.rate, 1.0)
        self.controller._on_start()
        try:
            while not self._stop_event.is_set() and self.controller.running:
                loop_start = time.time()
                if self.can_tele_flag:
                    self.controller.step(dt)
                    self._publish_pt()
                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            self.controller.running = False

    def _update_state_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                arms = np.asarray(
                    self.controller.arm_ctrl.get_current_dual_arm_q(), dtype=float
                ).reshape(-1)[:14]
            except Exception:
                arms = self._commanded_arms()
            gripper = self._measured_gripper()
            waist = float(self.controller.waist_command)
            with self.state_lock:
                self.pc["arms"] = arms.copy()
                self.pc["gripper"] = gripper
                self.pc["body"][0] = waist
            time.sleep(0.01)

    def reset_robot(self) -> None:
        """Return both arms to the initial pose (same intent as TeleopRunner)."""
        self.controller.reset()
        arms = self._commanded_arms()
        waist = float(self.controller.waist_command)
        try:
            current = np.asarray(
                self.controller.arm_ctrl.get_current_dual_arm_q(), dtype=float
            ).reshape(-1)[:14]
        except Exception:
            current = arms.copy()
        steps = 100
        for i in range(steps):
            if self._stop_event.is_set():
                break
            alpha = (i + 1) / steps
            interpolated = current * (1.0 - alpha) + arms * alpha
            self.controller.arm_ctrl.ctrl_dual_arm(np.hstack([interpolated, waist]))
            time.sleep(0.01)
        self.controller._output_command()
        self._publish_pt()

    def stop(self) -> None:
        self.can_tele_flag = False
        self._stop_event.set()
        self.controller.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.state_thread.is_alive():
            self.state_thread.join(timeout=1.0)
        self.controller._on_stop()


class R1A7McapRecord:
    """Drive ``RealTeleopController`` and record its state/action stream to MCAP."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.running = True
        self.is_recording = False
        self.record_paused = False
        self.teleop_active = False
        self.previous_x_button = False
        self.previous_y_button = False
        self.previous_b_button = False
        self.previous_a_button = False
        self.record_frame_count = 0
        self.record_start_monotonic = 0.0
        self.record_pause_started_monotonic = 0.0
        self.record_paused_duration = 0.0
        self.preview_created = False
        self.writer_lock = threading.Lock()
        self.record_stop_event = threading.Event()
        self.record_thread: Optional[threading.Thread] = None
        self.active_episode_id: Optional[int] = None
        self.active_episode_path: Optional[str] = None
        self.delete_available = False

        task_dir = self._task_dir()
        self.recorder = FourCameraMcapRecorder(
            task_dir=str(task_dir),
            task_goal=args.task_description,
            frequency=args.frequency,
        )
        print(f"[Record] episodes go to {task_dir}")

        self.image_subscriber = FourCameraSubscriber(args.image_server_address, args.image_server_port)

        config = R1A7Config(
            scale_factor=args.scale_factor,
            elbow_lower_override=args.elbow_lower_override,
        )
        if args.robot_urdf_path:
            config.urdf_path = args.robot_urdf_path
        if args.head_pose_file:
            config.head_pose_file = args.head_pose_file

        # Idle until X enables teleop, same as before.
        self.runner = RealTeleopRecordBridge(
            config=config,
            net=args.net,
            motion_mode=args.motion_mode,
            enable_dex1=args.enable_dex1,
            mock_xr=args.mock_xr,
            visualize=args.vis,
            debug=args.debug,
            scale_factor=args.scale_factor,
            rate=args.rate,
            waist_joystick_gain=args.waist_joystick_gain,
            waist_joystick_deadband=args.waist_joystick_deadband,
            teleop_enabled=False,
        )

    def _task_dir(self) -> Path:
        date = dt.datetime.now().strftime("%Y_%m_%d")
        return (
            Path(self.args.data_root).expanduser()
            / self.args.task_dir_name
            / f"{date}_robot_{self.args.robot_id}_dex1"
        )

    def start(self) -> None:
        self.image_subscriber.start()
        if not self.image_subscriber.wait_first_frame(self.args.image_wait):
            print("[Image] no four-camera frame received. Recording cannot start until PC2 images arrive.")
        self.record_thread = threading.Thread(
            target=self._record_loop, name="r1-a7-mcap-record", daemon=True
        )
        self.record_thread.start()
        self._main_loop()

    def _episode_metadata(self) -> dict:
        return {
            "robot": "R1-A7",
            "eef": "dex1",
            "image_server": self.image_subscriber.connect,
            "image_order": list(CAMERA_NAMES),
            "mcap_color_keys": list(MCAP_COLOR_KEYS),
            "motion_mode": bool(self.args.motion_mode),
            "net": self.args.net,
            "teleop_stack": "RealTeleopController",
        }

    def _start_episode(self) -> None:
        if self.is_recording:
            return
        frames, _, _ = self.image_subscriber.get()
        if frames is None:
            print("[Record] cannot start: waiting for PC2 four-camera image stream.")
            return
        with self.writer_lock:
            path = self.recorder.start_episode(self._episode_metadata())
            episode_id = self.recorder.episode_id
            self.active_episode_id = episode_id
            self.active_episode_path = path
            self.delete_available = False
            self.is_recording = True
            self.record_paused = False
            self.record_frame_count = 0
            self.record_start_monotonic = time.monotonic()
            self.record_pause_started_monotonic = 0.0
            self.record_paused_duration = 0.0
        print(f"[Record] START 第 {episode_id + 1} 条数据: {path}")

    def _record_active_duration(self, now: Optional[float] = None) -> float:
        if self.record_start_monotonic <= 0.0:
            return 0.0
        now = time.monotonic() if now is None else now
        paused_duration = self.record_paused_duration
        if self.record_paused and self.record_pause_started_monotonic > 0.0:
            paused_duration += now - self.record_pause_started_monotonic
        return max(now - self.record_start_monotonic - paused_duration, 0.0)

    def _toggle_record_pause(self) -> None:
        if not self.is_recording:
            print("[Record] B ignored: no active recording")
            return
        now = time.monotonic()
        with self.writer_lock:
            if not self.is_recording:
                return
            self.record_paused = not self.record_paused
            if self.record_paused:
                self.record_pause_started_monotonic = now
            else:
                self.record_paused_duration += now - self.record_pause_started_monotonic
                self.record_pause_started_monotonic = 0.0
            self.runner.can_tele_flag = self.teleop_active and not self.record_paused
            episode_id = self.active_episode_id
        action = "PAUSED" if self.record_paused else "RESUMED"
        detail = (
            "teleop output and MCAP writing paused"
            if self.record_paused
            else "teleop output and MCAP writing resumed"
        )
        print(f"[Record] {action} 第 {episode_id + 1} 条数据 ({detail})")

    def _stop_episode(self) -> None:
        if not self.is_recording:
            return
        self.runner.collect_start = False
        now = time.monotonic()
        with self.writer_lock:
            duration = max(self._record_active_duration(now), 1e-9)
            frame_count = self.record_frame_count
            self.is_recording = False
            self.record_paused = False
            self.record_pause_started_monotonic = 0.0
            self.recorder.stop_episode()
        self.runner.can_tele_flag = self.teleop_active
        episode_id = self.active_episode_id
        episode_path = self.active_episode_path
        self.delete_available = episode_id is not None and episode_path is not None
        print(
            f"[Record] SAVED 第 {episode_id + 1} 条数据至 {episode_path} | "
            f"frames={frame_count} duration={duration:.2f}s avg_fps={frame_count / duration:.1f}"
        )

    def _delete_last_episode(self) -> None:
        if self.is_recording:
            print("[Record] DELETE ignored: stop recording before deleting")
            return
        if not self.delete_available:
            print("[Record] DELETE ignored: no newly saved episode is available")
            return
        with self.writer_lock:
            deleted = self.recorder.delete_last_episode()
        if deleted is None:
            print("[Record] DELETE failed: latest episode file was not found")
            self.delete_available = False
            return
        deleted_id, deleted_path = deleted
        self.delete_available = False
        self.active_episode_id = None
        self.active_episode_path = None
        print(f"[Record] DELETED 第 {deleted_id + 1} 条数据: {deleted_path}")

    @staticmethod
    def _array(data: dict, name: str, size: int) -> np.ndarray:
        raw = np.asarray(data.get(name, np.zeros(size)), dtype=float).reshape(-1)
        result = np.zeros(size, dtype=float)
        result[: min(size, raw.size)] = raw[:size]
        return result

    def _state_action(self) -> tuple[dict, dict]:
        """Build the ``states`` / ``actions`` dicts written into each frame."""
        with self.runner.state_lock:
            pc = {key: value.copy() for key, value in self.runner.pc.items()}
            pt = {key: value.copy() for key, value in self.runner.pt.items()}
        state_body = self._array(pc, "body", 4)
        state_arms = self._array(pc, "arms", 14)
        state_hand = self._array(pc, "gripper", 4)
        action_body = self._array(pt, "body", 4)
        action_arms = self._array(pt, "arms", 14)
        action_hand = self._array(pt, "gripper", 4)
        return (
            {
                "robot_q_current": np.hstack((state_body, state_arms, state_hand)).tolist(),
                "body": state_body.tolist(),
                "arms": state_arms.tolist(),
                "hand_state": state_hand.tolist(),
            },
            {
                "robot_q_desired": np.hstack((action_body, action_arms, action_hand)).tolist(),
                "body": action_body.tolist(),
                "arms": action_arms.tolist(),
                "hand_cmd": action_hand.tolist(),
            },
        )

    def _record_loop(self) -> None:
        interval = 1.0 / max(float(self.args.frequency), 1.0)
        next_tick = time.perf_counter()
        while not self.record_stop_event.is_set():
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(next_tick - now)
                continue
            next_tick += interval
            if not self.is_recording or self.record_paused:
                continue
            frames, image_frame_id, _ = self.image_subscriber.get()
            if frames is None:
                continue
            states, actions = self._state_action()
            try:
                with self.writer_lock:
                    if self.is_recording and not self.record_paused:
                        self.recorder.add_frame(frames, states, actions, image_frame_id)
                        self.record_frame_count += 1
            except Exception as exc:
                print(f"[Record] write frame failed: {exc}")

    def _show_preview(self) -> bool:
        if not self.args.image_show:
            return False
        frames, image_frame_id, _ = self.image_subscriber.get()
        if frames is None:
            return False
        if self.is_recording:
            status = "PAUSED" if self.record_paused else "RECORDING"
        else:
            status = "TELEOP" if self.teleop_active else "NORMAL"
        labeled = [
            draw_label(frame, f"{name} | {status} | image {image_frame_id}")
            for name, frame in zip(CAMERA_NAMES, frames)
        ]
        preview = np.vstack((np.hstack(labeled[:2]), np.hstack(labeled[2:])))
        scale = max(float(self.args.image_preview_scale), 0.1)
        if scale != 1.0:
            preview = cv2.resize(preview, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        if not self.preview_created:
            cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(PREVIEW_WINDOW, preview.shape[1], preview.shape[0])
            self.preview_created = True
        cv2.imshow(PREVIEW_WINDOW, preview)
        return (cv2.waitKey(1) & 0xFF) in (27, ord("q"))

    def _enter_teleop(self) -> None:
        self.teleop_active = True
        self.runner.can_tele_flag = True
        print("[Mode] X: teleoperation started")

    def _leave_teleop(self) -> None:
        self.teleop_active = False
        self.runner.can_tele_flag = False
        print("[Mode] X: returning to normal mode; moving both arms to initial pose")
        try:
            self.runner.reset_robot()
            print("[Mode] X: returned to normal mode; both arms at initial pose")
        except Exception as exc:
            print(f"[Mode] X: failed to restore initial arm pose: {exc}")

    def _main_loop(self) -> None:
        print("\n[R1-A7 MCAP Record] ready (RealTeleopController stack)")
        print("  Pico X: enter/leave teleoperation mode")
        print("  Pico Y: start/stop and save MCAP recording (teleoperation mode only)")
        print("  Pico B: pause/resume writing the active episode")
        print("  Pico A: delete the most recently saved MCAP episode (idle only, once)")
        print("  image order: head_left, head_right, wrist_left, wrist_right")
        if self.args.auto_start:
            self._enter_teleop()
            print("[Mode] AUTO: teleoperation started")
            self._start_episode()
        try:
            while self.running:
                x_button = self.runner.controller.get_button_state("X")
                y_button = self.runner.controller.get_button_state("Y")
                b_button = self.runner.controller.get_button_state("B")
                a_button = self.runner.controller.get_button_state("A")

                if x_button and not self.previous_x_button:
                    if self.is_recording:
                        print("[Mode] X ignored: press Y to stop and save the active recording first")
                    elif self.teleop_active:
                        self._leave_teleop()
                    else:
                        self._enter_teleop()
                if y_button and not self.previous_y_button:
                    if not self.teleop_active:
                        print("[Record] Y ignored: press X to start teleoperation first")
                    elif self.is_recording:
                        self._stop_episode()
                    else:
                        self._start_episode()
                if b_button and not self.previous_b_button:
                    self._toggle_record_pause()
                if a_button and not self.previous_a_button and not self.is_recording:
                    self._delete_last_episode()
                self.previous_x_button = x_button
                self.previous_y_button = y_button
                self.previous_b_button = b_button
                self.previous_a_button = a_button

                if self.args.duration > 0 and self.is_recording:
                    if self._record_active_duration() >= self.args.duration:
                        self._stop_episode()

                if self._show_preview():
                    self.running = False
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self.running = False
        self.teleop_active = False
        self.runner.can_tele_flag = False
        self._stop_episode()
        self.record_stop_event.set()
        if self.record_thread is not None:
            self.record_thread.join(timeout=3.0)
        self.image_subscriber.close()
        if self.preview_created:
            cv2.destroyWindow(PREVIEW_WINDOW)
        self.runner.stop()
        print("[R1-A7 MCAP Record] stopped")


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )

    teleop = parser.add_argument_group("same control options as teleop_real.py")
    teleop.add_argument("--robot-urdf-path", default="")
    teleop.add_argument("--scale-factor", type=float, default=1.0)
    teleop.add_argument("--motion-mode", action="store_true",
                        help="Publish to the real robot via DDS (off = dry run)")
    teleop.add_argument("--enable-dex1", action="store_true", help="Enable the real Dex1 gripper")
    teleop.add_argument("--net", "--interface", dest="net", default="enp5s0")
    teleop.add_argument("--mock-xr", action="store_true", help="Use mock XR input (no PICO needed)")
    teleop.set_defaults(vis=False)
    teleop.add_argument("--vis", dest="vis", action="store_true")
    teleop.add_argument("--no-vis", dest="vis", action="store_false")
    teleop.add_argument("--debug", action="store_true")
    teleop.add_argument("--rate", type=float, default=50.0, help="Control loop rate (Hz)")
    teleop.add_argument("--waist-joystick", choices=["left", "right"], default="right")
    teleop.add_argument("--waist-joystick-axis", choices=["x", "y"], default="x")
    teleop.add_argument("--waist-joystick-gain", type=float, default=0.8)
    teleop.add_argument("--waist-joystick-deadband", type=float, default=0.08)
    teleop.add_argument("--elbow-lower-override", type=float, default=-1.30)
    teleop.add_argument("--head-pose-file", default="")
    teleop.add_argument("--no-confirm", action="store_true",
                        help="Skip the 'move' confirmation prompt before publishing")

    image = parser.add_argument_group("PC2 four-camera image service")
    image.add_argument("--image-server-address", default="192.168.123.164")
    image.add_argument("--image-server-port", type=int, default=5555)
    image.add_argument("--image-wait", type=float, default=5.0)
    image.set_defaults(image_show=False)
    image.add_argument("--image-show", dest="image_show", action="store_true")
    image.add_argument("--no-image-show", dest="image_show", action="store_false")
    image.add_argument("--image-preview-scale", type=float, default=0.75)

    record = parser.add_argument_group("MCAP recording")
    record.add_argument("--frequency", type=float, default=30.0)
    record.add_argument("--data-root", default=str(PROJECT_ROOT / "data"))
    record.add_argument("--task-dir-name", default="r1_a7_task")
    record.add_argument("--task-description", default="R1-A7 Dex1 teleoperation")
    record.add_argument("--robot-id", default="r1a7")
    record.add_argument("--auto-start", action="store_true",
                        help="Enter teleoperation and start recording immediately")
    record.add_argument("--duration", type=float, default=0.0,
                        help="Auto-stop each episode after N seconds (0 = manual)")
    return parser


def main() -> None:
    args = build_argparser().parse_args()

    # Pre-flight safety gate, before any DDS channel is opened.
    if args.motion_mode and not args.no_confirm:
        from robot.r1_a7_real import R1A7LowCmdConfig, confirm_real_motion

        confirm_real_motion(
            R1A7LowCmdConfig(interface=args.net, enable_motion=True, no_confirm=False)
        )

    recorder = R1A7McapRecord(args)

    def stop(_signum, _frame) -> None:
        recorder.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    recorder.start()


if __name__ == "__main__":
    main()
