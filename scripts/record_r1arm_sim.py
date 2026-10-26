#!/usr/bin/env python3
"""Record R1Arm MuJoCo teleoperation to MCAP episodes.

The established observation schema is retained, while joint_state/action use
the R1Arm simulation space (8 DoF single-arm, 16 DoF dual-arm).

Teleoperation starts immediately, just like ``teleop_r1arm_sim.py``:
  Y: start/stop and save an episode
  B: pause/resume recording and teleoperation output
  A: soft-reset the arm to home
  grip/trigger: unchanged arm tracking and Dex1 control

For mock/headless collection use ``--auto-start`` and ``--duration``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import queue
import signal
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.r1arm_config import DEFAULT_DUAL_MJCF_PATH, R1ArmConfig
from teleop.mcap_writer import FourCameraMcapRecorder
from teleop.r1arm_dataset import (
    CAMERA_VIEW_CHOICES,
    CAMERA_VIEW_TO_COLOR_KEY,
    R1ArmDatasetAdapter,
    R1ArmObservationRenderer,
)
from teleop.r1arm_mujoco_controller import R1ArmMujocoTeleopController


# Default observation selection. Edit this tuple or override it with
# ``--camera-views``. The number of recorded cameras is simply len(this tuple).
DEFAULT_CAMERA_VIEWS = ("head_left", "wrist_right")


class R1ArmSimMcapRecorder:
    """Control-loop sampler with asynchronous observation rendering/writing."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.camera_views = tuple(args.camera_views)
        if len(set(self.camera_views)) != len(self.camera_views):
            raise ValueError(f"--camera-views contains duplicates: {self.camera_views}")
        self.camera_keys = tuple(CAMERA_VIEW_TO_COLOR_KEY[name] for name in self.camera_views)
        self.running = True
        self.teleop_active = True
        self.is_recording = False
        self.record_paused = False
        self.active_episode_path: str | None = None
        self.record_frame_count = 0
        self.record_start_monotonic = 0.0
        self.record_pause_started_monotonic = 0.0
        self.record_paused_duration = 0.0
        self.next_sample_monotonic = 0.0
        self.image_frame_id = -1
        self.previous_buttons = {name: False for name in ("Y", "B")}
        self._frame_queue: queue.Queue[dict | None] = queue.Queue()
        self._render_ready = threading.Event()
        self._render_error: BaseException | None = None
        self._count_lock = threading.Lock()
        self._backlog_warned = False

        config = R1ArmConfig(
            scale_factor=args.scale_factor,
            dual_arm=args.use_dual_arm,
            control_side=args.side,
            reset_blend_frames=args.reset_blend_frames,
            trajectory_max_speed_rad_s=args.max_joint_speed,
        )
        if args.urdf:
            config.urdf_path = args.urdf
        if args.mjcf:
            config.mjcf_path = args.mjcf
        elif args.use_dual_arm:
            config.mjcf_path = str(DEFAULT_DUAL_MJCF_PATH)
        self.config = config

        self.controller = R1ArmMujocoTeleopController(
            config=config,
            mock_xr=args.mock_xr,
            debug=args.debug,
            scale_factor=args.scale_factor,
            show_viewer=not args.no_viewer,
            sim_substeps=args.sim_substeps,
            ik_dt=args.ik_dt if args.ik_dt > 0.0 else 1.0 / max(args.rate, 1.0),
        )
        self.controller.rate_hz = float(args.rate)
        # Keep the exact R1Arm teleop behavior: A remains the soft-reset key.
        self.controller.enable_a_button_reset = True
        self.adapter = R1ArmDatasetAdapter(
            dual_arm=args.use_dual_arm,
            control_side=args.side,
        )

        task_dir = self._task_dir()
        self.writer = FourCameraMcapRecorder(
            task_dir=str(task_dir),
            task_goal=args.task_description,
            frequency=args.frequency,
            format_name="r1arm_sim_multicamera_mcap_json_v1",
            camera_order=self.camera_keys,
        )
        self._render_thread = threading.Thread(
            target=self._render_writer_loop,
            name="r1arm-sim-mcap-render",
            daemon=True,
        )
        self._render_thread.start()
        if not self._render_ready.wait(timeout=30.0):
            self._frame_queue.put(None)
            self.controller._on_stop()
            raise RuntimeError("Timed out while initializing the MuJoCo observation renderer")
        if self._render_error is not None:
            error = self._render_error
            self._frame_queue.put(None)
            self._render_thread.join(timeout=2.0)
            self.controller._on_stop()
            raise RuntimeError("Failed to initialize the MuJoCo observation renderer") from error
        print(f"[Record] MCAP episodes go to {task_dir}")

    def _render_writer_loop(self) -> None:
        """Own the offscreen GL context and serialize queued snapshots to MCAP."""
        renderer = None
        try:
            import mujoco

            render_data = mujoco.MjData(self.controller.sim.model)
            render_sim = SimpleNamespace(model=self.controller.sim.model, data=render_data)
            renderer = R1ArmObservationRenderer(
                render_sim,
                width=self.args.image_width,
                height=self.args.image_height,
                dual_arm=self.args.use_dual_arm,
                camera_views=self.camera_views,
            )
            # Build shaders before accepting recording work.
            renderer.render()
        except BaseException as exc:
            self._render_error = exc
        finally:
            self._render_ready.set()

        while True:
            task = self._frame_queue.get()
            if task is None:
                try:
                    if renderer is not None:
                        renderer.close()
                finally:
                    self._frame_queue.task_done()
                return
            try:
                if self._render_error is not None or renderer is None:
                    continue

                render_data.qpos[:] = task["qpos"]
                render_data.qvel[:] = task["qvel"]
                if render_data.act.size:
                    render_data.act[:] = task["act"]
                if render_data.ctrl.size:
                    render_data.ctrl[:] = task["ctrl"]
                if render_data.mocap_pos.size:
                    render_data.mocap_pos[:] = task["mocap_pos"]
                    render_data.mocap_quat[:] = task["mocap_quat"]
                if render_data.userdata.size:
                    render_data.userdata[:] = task["userdata"]
                render_data.time = task["sim_time"]
                mujoco.mj_forward(self.controller.sim.model, render_data)

                frames = renderer.render()
                self.writer.add_frame(
                    frames,
                    task["states"],
                    task["actions"],
                    task["image_frame_id"],
                    sample_time_ns=task["sample_time_ns"],
                    control_seq=task["control_seq"],
                )
                with self._count_lock:
                    self.record_frame_count += 1
            except BaseException as exc:
                self._render_error = exc
            finally:
                self._frame_queue.task_done()

    def _close_render_worker(self) -> None:
        self._frame_queue.put(None)
        self._frame_queue.join()
        self._render_thread.join(timeout=5.0)

    def _task_dir(self) -> Path:
        if self.args.output_dir:
            return Path(self.args.output_dir).expanduser().resolve()
        date = dt.datetime.now().strftime("%Y_%m_%d")
        return (
            Path(self.args.data_root).expanduser()
            / self.args.task_dir_name
            / f"{date}_robot_{self.args.robot_id}_dex1"
        )

    def _episode_metadata(self) -> dict:
        return {
            "robot": "R1Arm",
            "eef": "dex1",
            "simulator": "mujoco",
            "motion_mode": False,
            "teleop_stack": "R1ArmMujocoTeleopController",
            "dual_arm": bool(self.args.use_dual_arm),
            "control_side": self.args.side,
            "xr_filter_enabled": False,
            "xr_filter_mode": "none_after_jump_rejection",
            "urdf_path": str(self.config.urdf_path),
            "mjcf_path": str(self.config.mjcf_path),
            "camera_views": list(self.camera_views),
            "camera_count": len(self.camera_views),
            "mcap_color_keys": list(self.camera_keys),
            "joint_names": list(self.adapter.joint_names),
            "state_dim": self.adapter.vector_size,
            "action_dim": self.adapter.vector_size,
            "gripper_dof_per_arm": 1,
            "state_layout": "right_arm(7)+right_gripper(1)+left_arm(7)+left_gripper(1)"
            if self.args.use_dual_arm
            else "arm(7)+gripper(1)",
            "sim_q_layout": "right_arm(7)+right_hand(2)+left_arm(7)+left_hand(2)"
            if self.args.use_dual_arm
            else "arm(7)+hand_fingers(2)",
            "teleop_rate_hz": float(self.args.rate),
            "record_frequency_hz": float(self.args.frequency),
            "trajectory_max_speed_rad_s": float(self.config.trajectory_max_speed_rad_s),
            "ik_dt_s": float(self.args.ik_dt) if self.args.ik_dt > 0 else 1.0 / float(self.args.rate),
            "trajectory_position_gain": float(self.config.trajectory_position_gain),
            "trajectory_velocity_gain": float(self.config.trajectory_velocity_gain),
            "ik_max_acc_per_s2": float(self.config.ik_max_acc_per_s2),
            "ik_max_jerk_per_s3": float(self.config.ik_max_jerk_per_s3),
            "diagnostic_action_fields": [
                "q_ik_raw", "q_purpose", "dq_purpose", "ddq_purpose",
                "ee_intent", "ik_solver", "xr_valid", "xr_raw", "xr_filtered",
                "xr_poses_raw", "xr_poses_filtered", "xr_frame_stamp_ns",
                "xr_poses_jump_rejected", "xr_filter_state", "xr_jump_reject_count",
                "control_seq", "control_time_ns", "grip_active", "teleop_anchor",
            ],
        }

    def _active_duration(self, now: float | None = None) -> float:
        if self.record_start_monotonic <= 0.0:
            return 0.0
        now = time.monotonic() if now is None else now
        paused = self.record_paused_duration
        if self.record_paused and self.record_pause_started_monotonic > 0.0:
            paused += now - self.record_pause_started_monotonic
        return max(0.0, now - self.record_start_monotonic - paused)

    def _start_episode(self) -> None:
        if self.is_recording:
            return
        if self._render_error is not None:
            raise RuntimeError("Observation renderer/writer is unavailable") from self._render_error
        path = self.writer.start_episode(self._episode_metadata())
        self.active_episode_path = path
        self.is_recording = True
        self.record_paused = False
        self.record_frame_count = 0
        self.image_frame_id = -1
        self.record_start_monotonic = time.monotonic()
        self.record_pause_started_monotonic = 0.0
        self.record_paused_duration = 0.0
        self.next_sample_monotonic = self.record_start_monotonic
        self._backlog_warned = False
        print(f"[Record] START episode {self.writer.episode_id}: {path}")

    def _wait_for_observation_flush(self, *, keep_teleop: bool) -> None:
        """Drain queued images without freezing the live R1Arm control loop."""
        control_dt = 1.0 / max(float(self.args.rate), 1.0)
        last_report = time.monotonic()
        while True:
            with self._frame_queue.all_tasks_done:
                unfinished = self._frame_queue.unfinished_tasks
            if unfinished == 0:
                return
            if keep_teleop and self.running:
                tick = time.monotonic()
                # No new recording frame is enqueued because is_recording has
                # already been cleared. Grip/trigger/A retain normal behavior.
                self._advance_simulation(control_dt)
                remaining = control_dt - (time.monotonic() - tick)
                if remaining > 0:
                    time.sleep(remaining)
            else:
                time.sleep(0.02)
            now = time.monotonic()
            if now - last_report >= 5.0:
                print(f"[Record] still flushing {unfinished} observation frame(s)...")
                last_report = now

    def _stop_episode(self, *, keep_teleop: bool = True) -> None:
        if not self.is_recording:
            return
        duration = max(self._active_duration(), 1e-9)
        self.is_recording = False
        self.record_paused = False
        self.record_pause_started_monotonic = 0.0
        queued = self._frame_queue.qsize()
        if queued:
            print(
                f"[Record] flushing {queued} queued observation frame(s); "
                "teleoperation remains active..."
            )
        self._wait_for_observation_flush(keep_teleop=keep_teleop)
        self.writer.stop_episode()
        with self._count_lock:
            frame_count = self.record_frame_count
        fps = frame_count / duration
        print(
            f"[Record] SAVED {self.active_episode_path} | "
            f"frames={frame_count} duration={duration:.2f}s avg_fps={fps:.1f}"
        )
        if self._render_error is not None:
            raise RuntimeError("Observation rendering or MCAP writing failed") from self._render_error
        if fps < self.args.frequency * 0.9:
            print(
                f"[Record] WARNING: effective {fps:.1f} Hz is below 90% of "
                f"requested {self.args.frequency:.1f} Hz"
            )

    def _toggle_pause(self) -> None:
        if not self.is_recording:
            print("[Record] B ignored: no active recording")
            return
        now = time.monotonic()
        self.record_paused = not self.record_paused
        if self.record_paused:
            self.record_pause_started_monotonic = now
            print("[Record] PAUSED (writing and teleoperation output paused)")
        else:
            self.record_paused_duration += now - self.record_pause_started_monotonic
            self.record_pause_started_monotonic = 0.0
            self.next_sample_monotonic = now
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
        if edges["Y"]:
            if self.is_recording:
                self._stop_episode()
            else:
                self._start_episode()
        if edges["B"]:
            self._toggle_pause()

    def _advance_simulation(self, control_dt: float) -> None:
        if (self.teleop_active and not self.record_paused) or self.controller._reset_blending:
            self.controller.step(control_dt)
        else:
            # Keep MuJoCo/viewer alive while holding the last command.
            self.controller._output_command()

    def _record_if_due(self, now: float) -> None:
        if not self.is_recording or self.record_paused or now < self.next_sample_monotonic:
            return
        interval = 1.0 / max(float(self.args.frequency), 1.0)
        while self.next_sample_monotonic <= now:
            self.next_sample_monotonic += interval

        measured = self.controller.sim.get_q()
        commanded = self.controller.sim_q()
        states, actions = self.adapter.frame_data(measured, commanded)
        control_sample = self.controller.control_sample() or {}
        side = self.args.side
        ee_intent = control_sample.get("ee_intent", {})
        actions.update({
            "q_ik_raw": np.asarray(
                control_sample.get("q_ik_raw", self.controller.arm_q(side)), dtype=float
            ).tolist(),
            "q_purpose": np.asarray(
                control_sample.get("q_purpose", self.controller.arm_q(side)), dtype=float
            ).tolist(),
            "dq_purpose": np.asarray(control_sample.get("dq_purpose", np.zeros(7)), dtype=float).tolist(),
            "ddq_purpose": np.asarray(control_sample.get("ddq_purpose", np.zeros(7)), dtype=float).tolist(),
            "ee_intent": {
                "position": np.asarray(ee_intent.get("position", []), dtype=float).tolist(),
                "quaternion_xyzw": np.asarray(ee_intent.get("quaternion_xyzw", []), dtype=float).tolist(),
                "frame": ee_intent.get("frame", "robot_base"),
            },
            "ik_solver": str(control_sample.get("ik_solver", "unknown")),
            "xr_valid": bool(control_sample.get("xr_valid", False)),
            "xr_raw": control_sample.get("xr_raw", {}),
            "xr_filtered": control_sample.get("xr_filtered", {}),
            "xr_poses_raw": control_sample.get("xr_poses_raw", {}),
            "xr_poses_filtered": control_sample.get("xr_poses_filtered", {}),
            "xr_poses_jump_rejected": control_sample.get("xr_poses_jump_rejected", {}),
            "xr_filter_state": control_sample.get("xr_filter_state", {}),
            "xr_jump_reject_count": control_sample.get("xr_jump_reject_count", 0),
            "xr_frame_stamp_ns": control_sample.get("xr_frame_stamp_ns"),
            "control_seq": control_sample.get("control_seq"),
            "control_time_ns": control_sample.get("control_time_ns"),
            "grip_active": bool(control_sample.get("grip_active", False)),
            "teleop_anchor": control_sample.get("teleop_anchor", {}),
        })
        self.image_frame_id += 1
        data = self.controller.sim.data
        self._frame_queue.put(
            {
                "sample_time_ns": time.time_ns(),
                "control_seq": control_sample.get("control_seq"),
                "image_frame_id": self.image_frame_id,
                "states": states,
                "actions": actions,
                "qpos": np.array(data.qpos, copy=True),
                "qvel": np.array(data.qvel, copy=True),
                "act": np.array(data.act, copy=True),
                "ctrl": np.array(data.ctrl, copy=True),
                "mocap_pos": np.array(data.mocap_pos, copy=True),
                "mocap_quat": np.array(data.mocap_quat, copy=True),
                "userdata": np.array(data.userdata, copy=True),
                "sim_time": float(data.time),
            }
        )
        backlog = self._frame_queue.qsize()
        backlog_limit = max(1, int(self.args.frequency * 2.0))
        if backlog > backlog_limit and not self._backlog_warned:
            print(
                f"[Record] WARNING: observation writer backlog={backlog}; "
                "recording will take extra time to flush"
            )
            self._backlog_warned = True
        if self._render_error is not None:
            raise RuntimeError("Observation rendering or MCAP writing failed") from self._render_error

    def run(self) -> None:
        print("\n[R1Arm MuJoCo MCAP Record] ready")
        print("  Teleoperation is active immediately (same as teleop_r1arm_sim.py)")
        print("  Pico Y: start/stop and save an episode")
        print("  Pico B: pause/resume recording and teleoperation")
        print("  Pico A: soft-reset the arm to home")
        print("  Hold grip while moving a controller; trigger controls Dex1")
        print(f"  Recording {len(self.camera_views)} view(s): {list(self.camera_views)}")
        print("  Terminal keyboard x/y are not control inputs")
        if self.args.auto_start:
            self._start_episode()

        control_dt = 1.0 / max(float(self.args.rate), 1.0)
        try:
            while self.running:
                loop_start = time.monotonic()
                self._handle_buttons()
                self._advance_simulation(control_dt)
                now = time.monotonic()
                self._record_if_due(now)
                if self.args.duration > 0 and self.is_recording:
                    if self._active_duration(now) >= self.args.duration:
                        self._stop_episode()
                        if self.args.exit_after_duration:
                            self.running = False
                remaining = control_dt - (time.monotonic() - loop_start)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self.running = False
        stop_error = None
        try:
            self._stop_episode(keep_teleop=False)
        except BaseException as exc:
            stop_error = exc
        finally:
            self._close_render_worker()
            self.controller._on_stop()
            print("[R1Arm MuJoCo MCAP Record] stopped")
        if stop_error is not None:
            raise stop_error


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sim = parser.add_argument_group("R1Arm MuJoCo teleoperation")
    sim.add_argument("--urdf", default="")
    sim.add_argument("--mjcf", default="")
    sim.add_argument("--use-dual-arm", action="store_true")
    sim.add_argument("--side", choices=("left", "right"), default="right")
    sim.add_argument("--scale-factor", type=float, default=1.0)
    sim.add_argument("--mock-xr", action="store_true")
    sim.add_argument("--debug", action="store_true")
    sim.add_argument("--rate", type=float, default=50.0)
    sim.add_argument("--ik-dt", type=float, default=0.0,
                     help="Placo IK timestep seconds; 0 aligns it to 1/--rate")
    sim.add_argument("--max-joint-speed", type=float, default=1.0,
                     help="q_purpose shaper speed limit rad/s")
    sim.add_argument("--no-viewer", action="store_true")
    sim.add_argument("--sim-substeps", type=int, default=10)
    sim.add_argument("--reset-blend-frames", type=int, default=100)

    record = parser.add_argument_group("compact R1Arm MCAP recording")
    record.add_argument("--frequency", type=float, default=30.0)
    record.add_argument(
        "--output-dir",
        default="",
        help="Exact directory in which episode_XXXX.mcap files are written",
    )
    record.add_argument("--data-root", default=str(PROJECT_ROOT / "data"))
    record.add_argument("--task-dir-name", default="r1arm_sim_task")
    record.add_argument("--task-description", default="R1Arm MuJoCo teleoperation")
    record.add_argument("--robot-id", default="r1arm_sim")
    record.add_argument("--image-width", type=int, default=320)
    record.add_argument("--image-height", type=int, default=240)
    record.add_argument(
        "--camera-views",
        nargs="+",
        choices=CAMERA_VIEW_CHOICES,
        default=list(DEFAULT_CAMERA_VIEWS),
        metavar="VIEW",
        help=(
            "Observation views to record; count is inferred from this list. "
            f"Choices: {', '.join(CAMERA_VIEW_CHOICES)}"
        ),
    )
    record.add_argument("--auto-start", action="store_true")
    record.add_argument("--duration", type=float, default=0.0)
    record.add_argument(
        "--exit-after-duration",
        action="store_true",
        help="Exit after --duration saves the episode (useful for mock/headless runs)",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    recorder = R1ArmSimMcapRecorder(args)

    def stop(_signum, _frame) -> None:
        recorder.running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    recorder.run()


if __name__ == "__main__":
    main()
