"""R1-A7 teleoperation runner -- local counterpart of ``teleop_r1a7.py``.

Single entry point for both simulation and the real robot, structured to match
``teleop/xrobotoolkit/unitree_teleop/teleop_r1a7.py`` in the reference
``xr_teleoperate_r1a7`` project so the two stay readable side by side:

- ``motion_mode`` picks the output backend: ``SimArmController`` (no DDS) or
  ``R1A7ArmControllerAdapter`` (``rt/lowcmd`` + ``rt/lowstate``)
- ``eef`` selects the end-effector command shape (dex1 / brainco / inspire)
- ``pc`` / ``pt`` hold the measured (current) and commanded (target) vectors the
  recorder reads
- ``run_loop`` runs teleoperation on a background thread, gated by
  ``can_tele_flag``; ``_update_state_loop`` samples hardware feedback at 100 Hz
- ``set_robot_command`` / ``set_gripper_command`` / ``reset_robot`` are the
  command surface

The tracking + IK core lives in ``teleop.tracking_controller``
(``R1A7TrackingController``), the way the reference delegates to
``PlacoTeleopController``. This module owns everything downstream of the solved
configuration: the waist joystick, the trigger -> gripper mapping, the real hand,
and the arm command itself.

Differences from the reference, all forced by this project's layout:
- IK is ``PlacoIKSolver`` (with a DLS fallback) instead of
  ``xrobotoolkit_teleop.simulation.placo_teleop_controller``; there is no
  dependency on the ``xrobotoolkit_teleop`` package here
- the waist is solved for IK/visualization only and is never published: this
  robot's ``REAL_JOINTS`` has no ``waist_yaw_joint``
- no motion-tracker elbow tasks (the reference's ``use_motion_tracker`` path)
"""

from __future__ import annotations

import threading
import time

import numpy as np

from teleop.tracking_controller import R1A7TrackingController, apply_deadband

from robot.arm_control import create_arm_controller
from robot.config import (
    BRAINCO_CMD_MAX,
    BRAINCO_THUMB_AUX_OPEN,
    DEX1_SIM_CLOSE,
    DEX1_SIM_OPEN,
    DEX1_SIM_SPEED,
    DEX1_TRIGGER_DEADBAND,
    EEF_DOF,
    LEFT_ARM_INIT,
    LEFT_DEX1_JOINTS,
    RIGHT_ARM_INIT,
    RIGHT_DEX1_JOINTS,
    R1A7Config,
    WAIST_LOWER,
    WAIST_UPPER,
    normalize_eef_name,
)


def apply_trigger_deadband(value: float) -> float:
    """Clamp a PICO analog trigger to 0..1 and ignore its resting jitter."""
    value = float(np.clip(value, 0.0, 1.0))
    return 0.0 if value < DEX1_TRIGGER_DEADBAND else value


class TeleopRunner:
    """Drive R1-A7 teleoperation in simulation or on the real robot."""

    def __init__(
        self,
        config: R1A7Config | None = None,
        scale_factor: float = 1.0,
        mode: str = "teleop",
        motion_mode: bool = False,
        net: str = "enp49s0",
        is_vis: bool = True,
        mock_xr: bool = False,
        debug: bool = False,
        eef: str = "dex1",
        enable_dex1: bool = False,
        enable_real_hand: bool = False,
        waist_joystick: str = "right",
        waist_joystick_axis: str = "x",
        waist_joystick_gain: float = 0.8,
        waist_joystick_deadband: float = 0.08,
        rate: float = 50.0,
        teleop_enabled: bool = True,
        start_threads: bool = True,
    ):
        self.mode = mode
        assert self.mode in ["teleop", "eval"], "mode must be 'teleop' or 'eval'"

        config = config if config is not None else R1A7Config()
        config.scale_factor = float(scale_factor)
        self.config = config

        self.motion_mode = bool(motion_mode)
        self.net = net
        self.rate = float(rate)
        self.eef = normalize_eef_name(eef)
        self.eef_dof = EEF_DOF[self.eef]
        self.enable_dex1 = bool(enable_dex1)
        self.enable_real_hand = bool(
            enable_real_hand or (self.eef == "dex1" and enable_dex1)
        )

        self.waist_joystick = waist_joystick
        self.waist_joystick_axis = waist_joystick_axis
        self.waist_joystick_gain = float(waist_joystick_gain)
        self.waist_joystick_deadband = float(waist_joystick_deadband)

        # ---- tracking + IK core (the reference's PlacoTeleopController) ----
        self.controller = R1A7TrackingController(
            config=config,
            visualize=is_vis,
            mock_xr=mock_xr,
            debug=debug,
            scale_factor=scale_factor,
        )
        self.joint_indices = self.controller.joint_indices
        self.head_init = self.controller.head_init
        self.left_dex1_init = self.controller.left_dex1_init
        self.right_dex1_init = self.controller.right_dex1_init

        # ---- command state ----
        self.left_arm_init = LEFT_ARM_INIT.copy()
        self.right_arm_init = RIGHT_ARM_INIT.copy()
        self.waist_init = self.controller.waist_init.copy()
        self.waist_command = float(self.waist_init[0])
        self.last_waist_update = time.monotonic()
        self.gripper_cmd = self._initial_gripper_command()
        self.last_gripper_update = time.monotonic()

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.is_reset_flag = True
        self.can_tele_flag = bool(teleop_enabled)
        self.collect_start = False
        self.thread = None
        self.state_thread = None
        self._stop_event = threading.Event()

        # ``pc`` = measured (plant current), ``pt`` = commanded (plant target).
        # Same names, shapes and semantics as the reference, so the recorder can
        # read them without translation. ``body`` is [waist, vx, vy, vz]; this
        # project never drives a mobile base, so vx/vy/vz are raw joystick
        # values -- archived, never sent.
        self.pc = {
            "arms": np.zeros(14, dtype=float),
            "gripper": self.gripper_cmd.copy(),
            "body": np.zeros(4, dtype=float),
        }
        self.pt = {
            "arms": np.zeros(14, dtype=float),
            "gripper": self.gripper_cmd.copy(),
            "body": np.zeros(4, dtype=float),
        }
        self.state_lock = threading.Lock()

        print(f"R1-A7 URDF: {self.controller.urdf_path}")
        print(
            "R1-A7 teleop options: "
            f"motion_mode={self.motion_mode}, "
            f"eef={self.eef}, "
            f"real_hand={self.enable_real_hand}, "
            f"scale_factor={scale_factor}, "
            f"elbow_lower_override={config.elbow_lower_override}"
        )

        # ---- output backends ----
        self.arm_ctrl = create_arm_controller(
            simulation=not self.motion_mode,
            config=config,
            interface=net,
            enable_motion=True,
            head_target=self.head_init,
        )

        self.hand_ctrl = None
        if self.eef == "dex1" and self.motion_mode and self.enable_real_hand:
            from robot.eef.dex1 import Dex1

            self.hand_ctrl = Dex1()
            print("Dex1 real hand controller enabled")
        elif self.eef in ("brainco", "inspire") and self.enable_real_hand:
            print(
                f"Real-hand bridge for {self.eef!r} is not wired up in this project; "
                "running arm teleop with correctly shaped commands only."
            )

        if start_threads:
            self.start_update_state()
            time.sleep(0.2)
            self.reset_robot()
            if self.mode == "teleop":
                self.start_teleop()

        print(f"R1-A7 {self.eef} teleop runner initialized.")

    # ------------------------------------------------------------------
    # End-effector command shaping
    # ------------------------------------------------------------------

    def _initial_gripper_command(self) -> np.ndarray:
        if self.eef == "dex1":
            return np.hstack([self.left_dex1_init, self.right_dex1_init])
        if self.eef == "brainco":
            left = np.zeros(self.eef_dof, dtype=float)
            right = np.zeros(self.eef_dof, dtype=float)
            if self.eef_dof > 1:
                left[1] = BRAINCO_THUMB_AUX_OPEN
                right[1] = BRAINCO_THUMB_AUX_OPEN
            return np.hstack([left, right])
        if self.eef == "inspire":
            return np.ones(self.eef_dof * 2, dtype=float)
        raise RuntimeError(f"Unhandled eef: {self.eef}")

    def _trigger_to_hand_command(self, left_trigger: float, right_trigger: float, dt: float) -> np.ndarray:
        left_trigger = apply_trigger_deadband(left_trigger)
        right_trigger = apply_trigger_deadband(right_trigger)
        if self.eef == "dex1":
            left = self._trigger_to_sim_gripper(left_trigger, self.gripper_cmd[:2], dt)
            right = self._trigger_to_sim_gripper(right_trigger, self.gripper_cmd[2:], dt)
            return np.hstack([left, right])
        if self.eef == "brainco":
            left = np.full(self.eef_dof, BRAINCO_CMD_MAX * left_trigger, dtype=float)
            right = np.full(self.eef_dof, BRAINCO_CMD_MAX * right_trigger, dtype=float)
            if self.eef_dof > 1:
                left[1] = BRAINCO_THUMB_AUX_OPEN
                right[1] = BRAINCO_THUMB_AUX_OPEN
            return np.hstack([left, right])
        if self.eef == "inspire":
            left = np.full(self.eef_dof, 1.0 - left_trigger, dtype=float)
            right = np.full(self.eef_dof, 1.0 - right_trigger, dtype=float)
            return np.hstack([left, right])
        raise RuntimeError(f"Unhandled eef: {self.eef}")

    def _split_and_clip_hand_command(self, gripper) -> tuple[np.ndarray, np.ndarray]:
        gripper = np.asarray(gripper, dtype=float).reshape(self.eef_dof * 2)
        left = gripper[: self.eef_dof].copy()
        right = gripper[self.eef_dof :].copy()
        if self.eef == "dex1":
            return (
                np.clip(left, DEX1_SIM_CLOSE, DEX1_SIM_OPEN),
                np.clip(right, DEX1_SIM_CLOSE, DEX1_SIM_OPEN),
            )
        if self.eef == "brainco":
            return np.clip(left, 0.0, 0.9), np.clip(right, 0.0, 0.9)
        if self.eef == "inspire":
            return np.clip(left, 0.0, 1.0), np.clip(right, 0.0, 1.0)
        raise RuntimeError(f"Unhandled eef: {self.eef}")

    def _trigger_to_sim_gripper(self, trigger_value: float, current: np.ndarray, dt: float) -> np.ndarray:
        """Map a trigger to a sim-space Dex1 position, rate-limited by dt."""
        trigger_value = float(np.clip(trigger_value, 0.0, 1.0))
        desired = DEX1_SIM_OPEN + (DEX1_SIM_CLOSE - DEX1_SIM_OPEN) * trigger_value
        max_delta = max(0.0, DEX1_SIM_SPEED * max(0.0, dt))
        if max_delta <= 0.0:
            value = desired
        else:
            value = float(current[0] + np.clip(desired - current[0], -max_delta, max_delta))
        value = float(np.clip(value, DEX1_SIM_CLOSE, DEX1_SIM_OPEN))
        return np.array([value, value], dtype=float)

    @staticmethod
    def _sim_to_closed_ratio(sim_value: float) -> float:
        """Map a sim-space Dex1 value to a 0..1 closed ratio (0=open, 1=closed)."""
        denom = DEX1_SIM_OPEN - DEX1_SIM_CLOSE
        ratio = 0.0 if abs(denom) < 1e-9 else (DEX1_SIM_OPEN - sim_value) / denom
        return float(np.clip(ratio, 0.0, 1.0))

    @staticmethod
    def _dex1_ratio_to_sim(open_ratio: float) -> float:
        """Map a Dex1 0..1 open ratio back into sim joint space."""
        open_ratio = float(np.clip(open_ratio, 0.0, 1.0))
        return DEX1_SIM_CLOSE + open_ratio * (DEX1_SIM_OPEN - DEX1_SIM_CLOSE)

    def _sync_visual_hand_joints(self, left_hand: np.ndarray, right_hand: np.ndarray):
        """Write the hand command into ``q`` so IK holds it and meshcat shows it."""
        if self.eef == "dex1":
            left_model = left_hand[:2]
            right_model = right_hand[:2]
        else:
            # Other end effectors are not in this URDF; keep the model at its
            # neutral Dex1 pose rather than writing a mismatched command.
            left_model = self.left_dex1_init
            right_model = self.right_dex1_init
        for i, name in enumerate(LEFT_DEX1_JOINTS):
            if name in self.joint_indices:
                self.controller.q[self.joint_indices[name]] = float(left_model[i])
        for i, name in enumerate(RIGHT_DEX1_JOINTS):
            if name in self.joint_indices:
                self.controller.q[self.joint_indices[name]] = float(right_model[i])

    def _send_real_hand_command(self, left_hand: np.ndarray, right_hand: np.ndarray):
        if self.hand_ctrl is None:
            return
        if self.eef == "dex1":
            self.hand_ctrl.set_gripper_ratios(
                self._sim_to_closed_ratio(float(left_hand[0])),
                self._sim_to_closed_ratio(float(right_hand[0])),
            )
            return
        if self.eef == "brainco":
            self.hand_ctrl.set_gripper_ratios(
                float(np.clip(left_hand[0] / BRAINCO_CMD_MAX, 0.0, 1.0)),
                float(np.clip(right_hand[0] / BRAINCO_CMD_MAX, 0.0, 1.0)),
            )

    def _get_hand_state(self) -> np.ndarray:
        """Measured end-effector state, falling back to the last command."""
        if self.eef == "dex1" and self.hand_ctrl is not None:
            try:
                left_state, right_state = self.hand_ctrl.get_hand_states()
            except Exception:
                return self.gripper_cmd.copy()
            left = self._dex1_ratio_to_sim(float(left_state[0]))
            right = self._dex1_ratio_to_sim(float(right_state[0]))
            return np.array([left, left, right, right], dtype=float)
        return self.gripper_cmd.copy()

    # ------------------------------------------------------------------
    # Waist (IK / visualization only -- never published)
    # ------------------------------------------------------------------

    def _apply_waist_to_q(self, waist_yaw: float):
        idx = self.joint_indices.get("waist_yaw_joint")
        if idx is not None:
            self.controller.q[idx] = float(waist_yaw)

    def _update_waist_from_joystick(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_waist_update)
        self.last_waist_update = now

        joy = self.controller.get_joystick(self.waist_joystick)
        axis_idx = 0 if self.waist_joystick_axis == "x" else 1
        axis_value = apply_deadband(float(joy[axis_idx]), self.waist_joystick_deadband)
        self.waist_command = float(
            np.clip(
                self.waist_command - self.waist_joystick_gain * axis_value * dt,
                WAIST_LOWER,
                WAIST_UPPER,
            )
        )
        self._apply_waist_to_q(self.waist_command)
        with self.state_lock:
            self.pt["body"][0] = self.waist_command

    # ------------------------------------------------------------------
    # Command surface
    # ------------------------------------------------------------------

    def set_robot_command(self, arms, gripper, wrist=0.0, duration=None):
        """Send an arm + end-effector command, optionally ramped over ``duration``."""
        arms = np.asarray(arms, dtype=float).reshape(14)
        left_gripper, right_gripper = self._split_and_clip_hand_command(gripper)
        wrist = float(wrist)

        with self.state_lock:
            self.pt["arms"] = arms.copy()
            self.pt["gripper"] = np.hstack([left_gripper, right_gripper])
            self.pt["body"][0] = wrist

        if duration is not None:
            # Sending a distant target in one frame makes the arms lurch; ramp
            # there at the 100 Hz cadence the DDS bridge expects.
            current = np.asarray(self.arm_ctrl.get_current_dual_arm_q(), dtype=float).reshape(-1)[:14]
            steps = max(1, int(float(duration) * 100))
            for i in range(steps):
                alpha = (i + 1) / steps
                interpolated = current * (1 - alpha) + arms * alpha
                self.arm_ctrl.ctrl_dual_arm(np.hstack([interpolated, wrist]))
                time.sleep(0.01)

        self.arm_ctrl.ctrl_dual_arm(np.hstack([arms, wrist]))
        self._apply_hand_command(left_gripper, right_gripper)

    def set_gripper_command(self, gripper):
        """Drive only the end effector, leaving the arm command untouched."""
        left_gripper, right_gripper = self._split_and_clip_hand_command(gripper)
        with self.state_lock:
            self.pt["gripper"] = np.hstack([left_gripper, right_gripper])
        self._apply_hand_command(left_gripper, right_gripper)

    def _apply_hand_command(self, left_gripper: np.ndarray, right_gripper: np.ndarray):
        self.gripper_cmd = np.hstack([left_gripper, right_gripper])
        self._sync_visual_hand_joints(left_gripper, right_gripper)
        self._send_real_hand_command(left_gripper, right_gripper)

    def reset_robot(self):
        """Return both arms to the initial pose and re-anchor tracking."""
        arms = np.hstack([self.left_arm_init, self.right_arm_init])
        self.waist_command = float(self.waist_init[0])
        self.last_waist_update = time.monotonic()

        # Reset the model first so the interpolated command starts from the same
        # pose the model believes in.
        self.controller.reset_configuration()
        self._apply_waist_to_q(self.waist_command)
        self.set_robot_command(
            arms=arms, gripper=self.gripper_cmd, wrist=self.waist_command, duration=1.0
        )
        self.controller.refresh_visualization()

    def move_to_init(self, duration: float = 1.0):
        """Ramp both arms back to the initial pose without touching the gripper."""
        arms = np.hstack([self.left_arm_init, self.right_arm_init])
        self.set_robot_command(
            arms=arms, gripper=self.gripper_cmd, wrist=self.waist_command, duration=duration
        )

    # ------------------------------------------------------------------
    # Loops
    # ------------------------------------------------------------------

    def _update_state_loop(self):
        """Sample measured hardware feedback into ``pc`` at ~100 Hz."""
        while not self._stop_event.is_set():
            try:
                arm_state = np.asarray(
                    self.arm_ctrl.get_current_dual_arm_q(), dtype=float
                ).reshape(-1)[:14]
            except Exception:
                arm_state = None
            hand_state = self._get_hand_state()
            with self.state_lock:
                if arm_state is not None:
                    self.pc["arms"] = arm_state.copy()
                self.pc["gripper"] = hand_state
                self.pc["body"][0] = self.waist_command
                self.pc["body"][1:4] = np.array([self.vx, self.vy, self.vz], dtype=float)
            time.sleep(0.01)

    def step(self):
        """One teleoperation tick: track, solve, then command."""
        dt = 1.0 / self.rate

        self._update_waist_from_joystick()
        self.controller.run_single_step(dt)
        # run_single_step re-solves the arms only; the waist is ours to hold.
        self._apply_waist_to_q(self.waist_command)

        if self.controller.get_button_state("Y"):
            self.collect_start = True
        if self.controller.get_button_state("X"):
            self.collect_start = False

        # The trigger drives the gripper on its own: computing this outside the
        # arm-activity branch keeps grasping available while the arms are still.
        now = time.monotonic()
        gripper_dt = max(0.0, now - self.last_gripper_update)
        self.last_gripper_update = now
        gripper = self._trigger_to_hand_command(
            self.controller.get_trigger_state("left"),
            self.controller.get_trigger_state("right"),
            gripper_dt,
        )

        if self.controller.reset_flag and not self.is_reset_flag:
            self.reset_robot()
            self.is_reset_flag = True
        elif not self.controller.reset_flag and (
            self.controller.active["left"] or self.controller.active["right"]
        ):
            self.is_reset_flag = False
            arms = np.hstack(
                [
                    self.controller.q[self.controller.left_arm_q_indices],
                    self.controller.q[self.controller.right_arm_q_indices],
                ]
            )
            self.set_robot_command(arms=arms, gripper=gripper, wrist=self.waist_command)
        elif not self.controller.reset_flag:
            self.set_gripper_command(gripper)

        self.vy, self.vx = self.controller.get_joystick("left")[:2]
        self.vz = float(self.controller.get_joystick("right")[0])
        with self.state_lock:
            self.pt["body"][1:4] = np.array([self.vx, self.vy, self.vz], dtype=float)

        # Show the command we actually issued, gripper and waist included.
        self.controller.refresh_visualization()

    def run_loop(self):
        """Background teleoperation loop, gated by ``can_tele_flag``."""
        period = 1.0 / self.rate
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            try:
                if not self.can_tele_flag:
                    time.sleep(0.01)
                    continue
                self.step()
            except KeyboardInterrupt:
                print("\nTeleoperation stopped.")
                break
            except Exception as exc:
                print(f"[Teleop] step failed: {exc}")
                time.sleep(0.05)
                continue
            sleep_time = period - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def start_teleop(self):
        if self.mode == "teleop" and self.thread is None:
            self.thread = threading.Thread(target=self.run_loop, name="r1-a7-teleop", daemon=True)
            self.thread.start()

    def start_update_state(self):
        if self.state_thread is None:
            self.state_thread = threading.Thread(
                target=self._update_state_loop, name="r1-a7-state", daemon=True
            )
            self.state_thread.start()

    def stop(self):
        self._stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            self.thread = None
        if self.state_thread is not None:
            self.state_thread.join(timeout=3.0)
            self.state_thread = None
        if self.hand_ctrl is not None and hasattr(self.hand_ctrl, "stop"):
            self.hand_ctrl.stop()
        if self.arm_ctrl is not None:
            self.arm_ctrl.stop()