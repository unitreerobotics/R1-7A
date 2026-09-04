"""Shared R1-A7 teleoperation core (tracking + IK), output-agnostic.

Holds the controller-pose tracking and IK logic that is identical between
simulation and the real robot, mirroring teleop_r1a7.py:
- Hold left/right grip to activate the corresponding arm
- Controller pose (position + orientation) maps to end-effector target
- IK (Placo QP, or iterative DLS fallback) solves for joint angles
- Trigger drives the Dex1 gripper (release=open, hold=close)

Subclasses (``SimTeleopController``, ``RealTeleopController``) implement the
output side by overriding ``_output_command`` (and optionally ``_on_start`` /
``_on_stop`` for lifecycle hooks). The solved configuration lives in
``self.q`` (URDF joint order, 21 DoF: waist, head, left arm, left dex1,
right arm, right dex1).
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from teleop.ik_solver import solve_upper_arm_ik
from teleop.placo_solver import PlacoIKSolver
from teleop.utils.mock import make_mock_controller_poses, make_mock_grip_values, xr_pose_is_valid
from teleop.utils.geometry import R_HEADSET_TO_WORLD, quaternion_to_euler, quaternion_to_world

from robot.arm_model import clamp_configuration, joint_indices_by_name, link_pose, load_robot_model
from robot.config import (
    DEX1_SIM_CLOSE,
    DEX1_SIM_OPEN,
    DEX1_SIM_SPEED,
    DEX1_TRIGGER_DEADBAND,
    LEFT_ARM_INIT,
    LEFT_ARM_JOINTS,
    LEFT_DEX1_JOINTS,
    RIGHT_ARM_INIT,
    RIGHT_ARM_JOINTS,
    RIGHT_DEX1_JOINTS,
    R1A7Config,
    WAIST_INIT,
    WAIST_LOWER,
    WAIST_UPPER,
    load_head_pose,
)
from robot.gripper import GripperController
from robot.urdf_utils import patch_urdf_elbow_lower
from robot.visualization import apply_joint_configuration, create_robot_visualization
from robot.xr_interface import get_joystick_state, get_key_value, make_xr_client


class R1A7TeleopControllerBase:
    """Controller-pose tracking + IK core shared by sim and real controllers.

    This is the single base class for the R1-A7 teleop controllers; the output
    backend is supplied by overriding ``_output_command`` (see the concrete
    ``SimTeleopController`` / ``RealTeleopController`` subclasses).
    """

    def __init__(
        self,
        config: R1A7Config,
        visualize: bool = True,
        mock_xr: bool = False,
        debug: bool = False,
        scale_factor: float = 1.0,
    ):
        self.config = config
        self.visualize = visualize
        self.mock_xr = mock_xr
        self.debug = debug
        self.scale_factor = scale_factor

        # URDF and model. Patch the elbow limit first so the IK solver and the
        # post-solve clamp both use the relaxed limit.
        self.urdf_path = Path(config.urdf_path)
        if config.elbow_lower_override is not None:
            self.urdf_path = Path(
                patch_urdf_elbow_lower(self.urdf_path, config.elbow_lower_override)
            )
            print(f"R1-A7 elbow lower limit relaxed to {config.elbow_lower_override:+.3f} rad")
        self.model, self.data = load_robot_model(str(self.urdf_path))
        self.joint_indices = joint_indices_by_name(self.model)

        # Arm joint indices
        self.left_arm_q_indices = [self.joint_indices[name] for name in LEFT_ARM_JOINTS]
        self.right_arm_q_indices = [self.joint_indices[name] for name in RIGHT_ARM_JOINTS]

        # Initial configuration
        self.head_init = load_head_pose(config.head_pose_file) if config.head_pose_file else np.zeros(2)
        self.waist_init = WAIST_INIT.copy()
        self.left_dex1_init = np.array([DEX1_SIM_OPEN, DEX1_SIM_OPEN], dtype=float)
        self.right_dex1_init = np.array([DEX1_SIM_OPEN, DEX1_SIM_OPEN], dtype=float)

        self.q = self._create_initial_q()

        # End-effector frame names
        self.ee_frames = {
            "left": "left_dex1_base_link",
            "right": "right_dex1_base_link",
        }

        # IK solver: prefer Placo (same QP solver as teleop_r1a7.py), fall back
        # to the iterative DLS solver when Placo is not importable in this env.
        self.use_placo = False
        self.placo_solver = None
        try:
            self.placo_solver = PlacoIKSolver(self.urdf_path, self.q, self.ee_frames, self.joint_indices)
            self.use_placo = True
            print("Using Placo QP IK solver (same as teleop_r1a7.py)")
        except Exception as exc:
            if self.debug:
                import traceback
                traceback.print_exc()
            print(f"Warning: Placo setup failed ({exc}); using DLS IK fallback.")
            self.use_placo = False

        # Visualization
        self.viz = None
        if visualize:
            try:
                self.viz = create_robot_visualization(self.model, self.urdf_path)
                apply_joint_configuration(self.viz, self.q)
                print("Meshcat visualization enabled")
            except Exception as e:
                import traceback
                traceback.print_exc()
                print(f"Warning: Could not setup meshcat: {e}")
                self.viz = None

        # XR client
        if mock_xr:
            self.xr_client = None
            print("Using mock XR input")
        else:
            try:
                self.xr_client = make_xr_client()
                print("Connected to XRoboToolkit")
            except Exception as e:
                print(f"Warning: Could not connect to XRoboToolkit: {e}")
                print("Falling back to mock XR input")
                self.xr_client = None
                self.mock_xr = True

        # Tracking state (same as teleop_r1a7.py)
        self.ref_ee_pos = {"left": None, "right": None}
        self.ref_ee_quat = {"left": None, "right": None}
        self.ref_controller_pos = {"left": None, "right": None}
        self.ref_controller_rot = {"left": None, "right": None}
        self.ref_heading_yaw = {"left": None, "right": None}
        self.active = {"left": False, "right": False}

        # Gripper (trigger-driven; independent of arm activation)
        self.gripper = GripperController(
            self.model,
            self.joint_indices,
            open_value=DEX1_SIM_OPEN,
            close_value=DEX1_SIM_CLOSE,
            speed=DEX1_SIM_SPEED,
        )
        self.last_gripper_update = time.monotonic()

        # Waist control
        self.waist_command = float(self.waist_init[0])
        self.last_waist_update = time.monotonic()
        self.waist_joystick_gain = 0.8
        self.waist_joystick_deadband = 0.08

        # Reset state
        self.reset_flag = True
        self.is_reset_flag = True
        # Recording binds A to "delete episode"; teleop must not also reset on A.
        self.enable_a_button_reset = True

        # State
        self.running = True
        self.t = 0.0

    def _create_initial_q(self) -> np.ndarray:
        """Create initial configuration vector."""
        q = np.zeros(self.model.nq, dtype=float)
        # Set waist
        if "waist_yaw_joint" in self.joint_indices:
            q[self.joint_indices["waist_yaw_joint"]] = self.waist_init[0]
        # Set head
        if "head_pitch_joint" in self.joint_indices:
            q[self.joint_indices["head_pitch_joint"]] = self.head_init[0]
        if "head_yaw_joint" in self.joint_indices:
            q[self.joint_indices["head_yaw_joint"]] = self.head_init[1]
        # Set left arm
        for i, name in enumerate(LEFT_ARM_JOINTS):
            if name in self.joint_indices:
                q[self.joint_indices[name]] = LEFT_ARM_INIT[i]
        # Set right arm
        for i, name in enumerate(RIGHT_ARM_JOINTS):
            if name in self.joint_indices:
                q[self.joint_indices[name]] = RIGHT_ARM_INIT[i]
        # Set gripper
        for i, name in enumerate(LEFT_DEX1_JOINTS):
            if name in self.joint_indices:
                q[self.joint_indices[name]] = self.left_dex1_init[i]
        for i, name in enumerate(RIGHT_DEX1_JOINTS):
            if name in self.joint_indices:
                q[self.joint_indices[name]] = self.right_dex1_init[i]
        return q

    def _apply_deadband(self, value: float, deadband: float) -> float:
        """Apply deadband to a value."""
        value = float(value)
        deadband = abs(float(deadband))
        if abs(value) <= deadband:
            return 0.0
        return value

    def _link_pose(self, frame_name: str) -> tuple:
        """Get a frame's (pos, xyzw quat) at the current q; (None, None) on error."""
        try:
            return link_pose(self.model, self.data, self.q, frame_name)
        except Exception:
            return None, None

    def _solve_arm_ik(self, side: str, target_pos: np.ndarray, target_quat: np.ndarray | None = None):
        """Solve IK for one arm with iterative damped least squares (DLS)."""
        q_indices = self.left_arm_q_indices if side == "left" else self.right_arm_q_indices
        target_rot = None
        if target_quat is not None:
            target_rot = R.from_quat(target_quat).as_matrix()
        q_next, _ = solve_upper_arm_ik(
            self.model,
            self.data,
            self.q,
            ee_frame_name=self.ee_frames[side],
            ee_target_pos=target_pos,
            ee_target_rot=target_rot,
            elbow_frame_name=None,
            elbow_target_pos=None,
            q_indices=q_indices,
            q_reference=self.q.copy(),
            control_mode="pose" if target_rot is not None else "position",
            debug=self.debug,
        )
        self.q = q_next

    def _solve(self, targets: dict):
        """Dispatch to the Placo QP solver, or iterative DLS as a fallback."""
        if self.use_placo:
            sol = self.placo_solver.solve(self.q, targets)
            if sol is not None:
                for idx in self.left_arm_q_indices + self.right_arm_q_indices:
                    self.q[idx] = sol[idx]
                return
            # Placo QP failed — fall back to warm-start DLS using current q as the
            # regularization reference so the solver continues from where Placo left off.
        for side, (pos, quat) in targets.items():
            self._solve_arm_ik(side, pos, quat)

    def reset(self):
        """Reset robot to initial configuration."""
        self.q = self._create_initial_q()
        self.waist_command = float(self.waist_init[0])
        self.gripper.reset()

        # Reset tracking references
        for side in ["left", "right"]:
            self.ref_ee_pos[side] = None
            self.ref_ee_quat[side] = None
            self.ref_controller_pos[side] = None
            self.ref_controller_rot[side] = None
            self.ref_heading_yaw[side] = None
            self.active[side] = False

        self.reset_flag = True
        self.is_reset_flag = True

    def read_controller_poses(self) -> tuple:
        """Read controller poses from XR or mock.

        Returns:
            Tuple of (controller_poses dict, is_valid bool)
        """
        if self.mock_xr or self.xr_client is None:
            return make_mock_controller_poses(self.t), True

        try:
            controller_poses = {}
            for name in ["headset", "left_controller", "right_controller"]:
                pose = self.xr_client.get_pose_by_name(name)
                if xr_pose_is_valid(pose):
                    controller_poses[name] = np.asarray(pose, dtype=float)[:7]
                else:
                    return make_mock_controller_poses(self.t), True
            return controller_poses, True
        except Exception as e:
            if self.debug:
                print(f"XR read error: {e}")
            return make_mock_controller_poses(self.t), True

    def get_grip_state(self, side: str) -> float:
        """Get the side grip value, which only toggles arm activation."""
        if self.mock_xr or self.xr_client is None:
            left_grip, right_grip, _, _ = make_mock_grip_values(self.t)
            return left_grip if side == "left" else right_grip
        return get_key_value(self.xr_client, f"{side}_grip")

    def get_trigger_state(self, side: str) -> float:
        """Get the analog trigger value that drives the Dex1 gripper.

        Kept separate from the grip: the grip only toggles arm activation, while
        the trigger alone controls open/close (mirrors teleop_r1a7.py, which
        reads "left_trigger"/"right_trigger" for the gripper and
        "left_grip"/"right_grip" for arm activation).
        """
        if self.mock_xr or self.xr_client is None:
            _, _, left_trigger, right_trigger = make_mock_grip_values(self.t)
            return left_trigger if side == "left" else right_trigger
        return get_key_value(self.xr_client, f"{side}_trigger")

    def get_waist_joystick(self) -> np.ndarray:
        """Get right-joystick (x, y) values for waist control."""
        if self.mock_xr or self.xr_client is None:
            return np.zeros(2)
        return get_joystick_state(self.xr_client, "right")

    def get_button_state(self, name: str) -> bool:
        """Read a Pico digital button (``A``/``B``/``X``/``Y``)."""
        if self.mock_xr or self.xr_client is None:
            return False
        try:
            return bool(self.xr_client.get_button_state_by_name(name))
        except Exception:
            return False

    def get_reset_button(self) -> bool:
        """Check if reset button (A) is pressed."""
        if not self.enable_a_button_reset:
            return False
        return self.get_button_state("A")

    def update_waist(self, dt: float):
        """Update waist from joystick."""
        now = time.monotonic()
        dt_actual = max(0.0, now - self.last_waist_update)
        self.last_waist_update = now

        joy = self.get_waist_joystick()
        # Right joystick X: positive=right, negative=left
        # Positive = turn waist right, negative = turn waist left
        axis_value = self._apply_deadband(float(joy[0]), self.waist_joystick_deadband)
        self.waist_command = float(
            np.clip(
                self.waist_command - self.waist_joystick_gain * axis_value * dt_actual,
                WAIST_LOWER,
                WAIST_UPPER,
            )
        )

        if "waist_yaw_joint" in self.joint_indices:
            self.q[self.joint_indices["waist_yaw_joint"]] = self.waist_command

    def step(self, dt: float) -> np.ndarray:
        """Run one control loop step.

        Same logic as teleop_r1a7.py:
        1. Read controller poses
        2. Check gripper activation
        3. If activated, compute IK target from controller delta
        4. Solve IK for arms
        5. Update gripper from triggers
        """
        self.t += dt

        # Update waist from joystick
        self.update_waist(dt)

        # Read controller poses
        controller_poses, valid = self.read_controller_poses()
        if not valid:
            return self.q

        # Check gripper activation
        left_grip = self.get_grip_state("left")
        right_grip = self.get_grip_state("right")
        self.active["left"] = left_grip > 0.9
        self.active["right"] = right_grip > 0.9

        # Check reset button
        if self.get_reset_button() and not self.is_reset_flag:
            self.reset()
            return self.q
        elif not self.get_reset_button():
            self.is_reset_flag = False

        # Update head reference frame from headset
        headset_pose = controller_poses.get("headset")
        if headset_pose is not None:
            # Headset forward direction in world frame
            self.head_forward = np.array([0, 1, 0])
        else:
            self.head_forward = np.array([0, 1, 0])

        # Collect IK targets for active arms; solved once after gripper/head update.
        targets = {}

        # Process each arm
        for side in ["left", "right"]:
            controller_name = f"{side}_controller"
            controller_pose = controller_poses.get(controller_name)

            if controller_pose is None:
                continue

            if not self.active[side]:
                # Deactivated - reset reference
                self.ref_ee_pos[side] = None
                self.ref_ee_quat[side] = None
                self.ref_controller_pos[side] = None
                self.ref_controller_rot[side] = None
                continue

            # Get current EE pose
            ee_pos, ee_quat = self._link_pose(self.ee_frames[side])
            if ee_pos is None:
                continue

            # Controller pose (position + quaternion)
            # Convert controller coords to world coords, matching
            # BaseTeleopController._process_xr_pose in teleop_r1a7.py:
            # PICO frame x=left/right, y=forward/back, z=up/down
            # -> world_x = -h_z, world_y = -h_x, world_z = h_y.
            # The quaternion must be rotated into the SAME world frame, otherwise
            # roll/pitch/yaw are scrambled relative to the (correct) position.
            ctrl_pos_raw = controller_pose[:3]
            ctrl_pos = R_HEADSET_TO_WORLD @ ctrl_pos_raw
            ctrl_quat_raw = controller_pose[3:7]  # [x, y, z, w]
            ctrl_rot_world = quaternion_to_world(ctrl_quat_raw)

            # First activation - initialize references
            if self.ref_ee_pos[side] is None:
                self.ref_ee_pos[side] = ee_pos.copy()
                self.ref_ee_quat[side] = ee_quat.copy() if ee_quat is not None else None
                self.ref_controller_pos[side] = ctrl_pos.copy()  # Already converted to world frame
                self.ref_controller_rot[side] = ctrl_rot_world

                # Compute yaw reference for heading compensation (world frame)
                euler = quaternion_to_euler(*ctrl_rot_world.as_quat())
                self.ref_heading_yaw[side] = float(euler[2])
                continue

            # Compute delta in world frame
            delta_pos = ctrl_pos - self.ref_controller_pos[side]

            # Apply heading compensation (rotate delta to align with reference yaw)
            yaw_ref = self.ref_heading_yaw.get(side, 0.0)
            c = math.cos(-yaw_ref)
            s = math.sin(-yaw_ref)
            delta_pos_corrected = np.array([
                c * delta_pos[0] - s * delta_pos[1],
                s * delta_pos[0] + c * delta_pos[1],
                delta_pos[2]
            ])

            # Apply scale factor
            delta_pos_scaled = delta_pos_corrected * self.scale_factor

            # Compute target position
            target_pos = self.ref_ee_pos[side] + delta_pos_scaled

            # Compute target orientation (mirror BaseTeleopController._process_xr_pose)
            target_quat = None
            if self.ref_controller_rot[side] is not None and self.ref_ee_quat[side] is not None:
                # Orientation delta in world frame, as angle-axis
                rot_delta = ctrl_rot_world * self.ref_controller_rot[side].inv()
                delta_rot = rot_delta.as_rotvec()

                # Apply the same heading compensation used for position
                delta_rot = np.array([
                    c * delta_rot[0] - s * delta_rot[1],
                    s * delta_rot[0] + c * delta_rot[1],
                    delta_rot[2],
                ])

                # Apply the (heading-aligned) orientation delta onto the EE reference
                rot_target = R.from_rotvec(delta_rot) * R.from_quat(self.ref_ee_quat[side])
                target_quat = rot_target.as_quat()  # [x, y, z, w]

            # Collect target; solved once after gripper/head update below.
            targets[side] = (target_pos, target_quat)

        # Update gripper from triggers (independent of arm activation; trigger
        # alone controls open/close, matching teleop_r1a7.py).
        now = time.monotonic()
        gripper_dt = max(0.0, now - self.last_gripper_update)
        self.last_gripper_update = now

        left_trigger = self.get_trigger_state("left")
        right_trigger = self.get_trigger_state("right")

        # Apply deadband
        left_trigger = 0.0 if left_trigger < DEX1_TRIGGER_DEADBAND else left_trigger
        right_trigger = 0.0 if right_trigger < DEX1_TRIGGER_DEADBAND else right_trigger

        gripper_cmd = self.gripper.update(left_trigger, right_trigger, gripper_dt)

        # Apply gripper to configuration
        for name in LEFT_DEX1_JOINTS:
            if name in self.joint_indices:
                self.q[self.joint_indices[name]] = gripper_cmd.left
        for name in RIGHT_DEX1_JOINTS:
            if name in self.joint_indices:
                self.q[self.joint_indices[name]] = gripper_cmd.right

        # Lock head to fixed pose
        if "head_pitch_joint" in self.joint_indices:
            self.q[self.joint_indices["head_pitch_joint"]] = self.head_init[0]
        if "head_yaw_joint" in self.joint_indices:
            self.q[self.joint_indices["head_yaw_joint"]] = self.head_init[1]

        # Solve IK for active arms.  Skip entirely when nothing is active so the
        # arm joints stay at their last solved position (no spring-back to init).
        if targets:
            self._solve(targets)

        # Apply limits
        self.q = clamp_configuration(self.model, self.q)

        # Update visualization
        if self.viz:
            apply_joint_configuration(self.viz, self.q)

        # Output hook: send the solved q to the sim viz / real DDS backend.
        self._output_command()

        # Debug output
        if self.debug and int(self.t * 5) % 5 == 0:
            left_arm_q = [self.q[idx] for idx in self.left_arm_q_indices]
            print(f"t={self.t:.2f}s | left: {np.round(left_arm_q, 3)} | active: {self.active}")

        return self.q

    # ---------------------------------------------------------
    # --- Overridable hooks (output backend + lifecycle) ---
    # ---------------------------------------------------------

    def _output_command(self):
        """Send the solved configuration to the output backend (no-op default)."""

    def _on_start(self):
        """Hook called once before the control loop starts."""

    def _on_stop(self):
        """Hook called once when the control loop exits."""

    def set_target_poses(self, poses: dict) -> None:
        """Store externally supplied target poses (injection hook for tests).

        This controller reads controller poses from the XR client (or mock)
        inside ``step``, so this hook only stores the value for callers that
        drive the controller externally (a future non-XR driver).
        """
        self._external_target_poses = poses

    def get_joint_states(self) -> dict:
        """Return current joint states as a name -> value dict."""
        return {name: float(self.q[idx]) for name, idx in self.joint_indices.items()}

    def run(self, rate: float = 50.0):
        """Run the teleoperation loop."""
        print(f"Starting teleoperation (rate={rate} Hz)")
        print("Press Ctrl+C to stop")
        print("Hold controller grip to activate arm tracking")
        print("Press A button to reset to initial pose")

        self._on_start()

        dt = 1.0 / rate
        try:
            while self.running:
                loop_start = time.time()
                self.step(dt)

                elapsed = time.time() - loop_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nStopping teleoperation...")
        finally:
            self.running = False
            self._on_stop()
