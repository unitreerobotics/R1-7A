"""R1 单臂 PICO 遥操核心（相对位姿跟踪 + IK），输出无关。

复用 A7 同款手柄相对跟踪语义：
- 按住 grip 激活跟踪
- 手柄 Δpos/Δrot → EE 目标
- Placo QP（失败则 DLS）求 7 关节角
- A 键复位

子类通过 ``_output_command`` 接到 MuJoCo / 真机。
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.arm_model import clamp_configuration, joint_indices_by_name, link_pose, load_robot_model
from robot.r1arm_config import ARM_INIT, ARM_JOINTS, R_TELEOP_TO_BASE, R1ArmConfig
from robot.xr_interface import get_key_value, make_xr_client
from teleop.ik_solver import solve_upper_arm_ik
from teleop.r1arm_placo_solver import PlacoArmIKSolver
from teleop.utils.geometry import R_HEADSET_TO_WORLD, quaternion_to_euler, quaternion_to_world
from teleop.utils.mock import make_mock_controller_poses, make_mock_grip_values, xr_pose_is_valid


class R1ArmTeleopControllerBase:
    """单臂 tracking + IK；默认跟右侧手柄。"""

    def __init__(
        self,
        config: R1ArmConfig,
        mock_xr: bool = False,
        debug: bool = False,
        scale_factor: float | None = None,
    ):
        self.config = config
        self.mock_xr = mock_xr
        self.debug = debug
        self.scale_factor = float(config.scale_factor if scale_factor is None else scale_factor)
        self.control_side = str(config.control_side).lower()
        if self.control_side not in {"left", "right"}:
            raise ValueError(f"control_side must be left|right, got {config.control_side!r}")
        self.R_teleop_to_base = (
            R_TELEOP_TO_BASE.copy() if config.align_teleop_frame else np.eye(3, dtype=float)
        )

        self.urdf_path = Path(config.urdf_path)
        self.model, self.data = load_robot_model(str(self.urdf_path))
        self.joint_indices = joint_indices_by_name(self.model)
        self.arm_q_indices = [self.joint_indices[name] for name in ARM_JOINTS]
        self.ee_frame = config.ee_frame

        self.q = self._create_initial_q()

        self.use_placo = False
        self.placo_solver = None
        try:
            self.placo_solver = PlacoArmIKSolver(
                self.urdf_path,
                self.q,
                self.ee_frame,
                self.joint_indices,
                arm_joints=ARM_JOINTS,
                arm_task_weight=config.arm_task_weight,
            )
            self.use_placo = True
            print("Using Placo QP IK solver (single-arm)")
        except Exception as exc:
            if self.debug:
                import traceback

                traceback.print_exc()
            print(f"Warning: Placo setup failed ({exc}); using DLS IK fallback.")

        if config.align_teleop_frame:
            print("Teleop frame aligned to r1arm base (A7 +X前/+Z上 → URDF -Y前/+Z上)")
        else:
            print("Teleop frame alignment OFF (raw A7 world axes)")
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

        self.ref_ee_pos = None
        self.ref_ee_quat = None
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_heading_yaw = None
        self.active = False

        self.reset_flag = True
        self.is_reset_flag = True
        self.enable_a_button_reset = True
        self.running = True
        self.t = 0.0

    def _create_initial_q(self) -> np.ndarray:
        q = np.zeros(self.model.nq, dtype=float)
        for i, name in enumerate(ARM_JOINTS):
            if name in self.joint_indices:
                q[self.joint_indices[name]] = ARM_INIT[i]
        return q

    def _link_pose(self, frame_name: str) -> tuple:
        try:
            return link_pose(self.model, self.data, self.q, frame_name)
        except Exception:
            return None, None

    def _solve_dls(self, target_pos: np.ndarray, target_quat: np.ndarray | None):
        target_rot = None
        if target_quat is not None:
            target_rot = R.from_quat(target_quat).as_matrix()
        q_next, _ = solve_upper_arm_ik(
            self.model,
            self.data,
            self.q,
            ee_frame_name=self.ee_frame,
            ee_target_pos=target_pos,
            ee_target_rot=target_rot,
            elbow_frame_name=None,
            elbow_target_pos=None,
            q_indices=self.arm_q_indices,
            q_reference=self.q.copy(),
            control_mode="pose" if target_rot is not None else "position",
            debug=self.debug,
        )
        self.q = q_next

    def _solve(self, target_pos: np.ndarray, target_quat: np.ndarray | None):
        if self.use_placo and self.placo_solver is not None:
            sol = self.placo_solver.solve(self.q, target_pos, target_quat)
            if sol is not None:
                for idx in self.arm_q_indices:
                    self.q[idx] = sol[idx]
                return
        self._solve_dls(target_pos, target_quat)

    def reset(self):
        self.q = self._create_initial_q()
        self.ref_ee_pos = None
        self.ref_ee_quat = None
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_heading_yaw = None
        self.active = False
        self.reset_flag = True
        self.is_reset_flag = True

    def read_controller_poses(self) -> tuple:
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

    def get_grip_state(self) -> float:
        side = self.control_side
        if self.mock_xr or self.xr_client is None:
            left_grip, right_grip, _, _ = make_mock_grip_values(self.t)
            return left_grip if side == "left" else right_grip
        return get_key_value(self.xr_client, f"{side}_grip")

    def get_button_state(self, name: str) -> bool:
        if self.mock_xr or self.xr_client is None:
            return False
        try:
            return bool(self.xr_client.get_button_state_by_name(name))
        except Exception:
            return False

    def get_reset_button(self) -> bool:
        if not self.enable_a_button_reset:
            return False
        return self.get_button_state("A")

    def step(self, dt: float) -> np.ndarray:
        self.t += dt

        controller_poses, valid = self.read_controller_poses()
        if not valid:
            self._output_command()
            return self.q

        self.active = self.get_grip_state() > 0.9

        if self.get_reset_button() and not self.is_reset_flag:
            self.reset()
            self._output_command()
            return self.q
        if not self.get_reset_button():
            self.is_reset_flag = False

        side = self.control_side
        controller_pose = controller_poses.get(f"{side}_controller")

        if controller_pose is not None and self.active:
            ee_pos, ee_quat = self._link_pose(self.ee_frame)
            if ee_pos is not None:
                ctrl_pos = R_HEADSET_TO_WORLD @ controller_pose[:3]
                ctrl_rot_world = quaternion_to_world(controller_pose[3:7])

                if self.ref_ee_pos is None:
                    self.ref_ee_pos = ee_pos.copy()
                    self.ref_ee_quat = ee_quat.copy() if ee_quat is not None else None
                    self.ref_controller_pos = ctrl_pos.copy()
                    self.ref_controller_rot = ctrl_rot_world
                    euler = quaternion_to_euler(*ctrl_rot_world.as_quat())
                    self.ref_heading_yaw = float(euler[2])
                else:
                    delta_pos = ctrl_pos - self.ref_controller_pos
                    yaw_ref = self.ref_heading_yaw or 0.0
                    c = math.cos(-yaw_ref)
                    s = math.sin(-yaw_ref)
                    delta_pos_corrected = np.array(
                        [
                            c * delta_pos[0] - s * delta_pos[1],
                            s * delta_pos[0] + c * delta_pos[1],
                            delta_pos[2],
                        ]
                    )
                    # A7 遥操世界 → r1arm URDF 基座（前/左/上 对齐到臂工作空间）
                    delta_pos_base = self.R_teleop_to_base @ delta_pos_corrected
                    target_pos = self.ref_ee_pos + delta_pos_base * self.scale_factor

                    target_quat = None
                    if self.ref_controller_rot is not None and self.ref_ee_quat is not None:
                        rot_delta = ctrl_rot_world * self.ref_controller_rot.inv()
                        delta_rot = rot_delta.as_rotvec()
                        delta_rot = np.array(
                            [
                                c * delta_rot[0] - s * delta_rot[1],
                                s * delta_rot[0] + c * delta_rot[1],
                                delta_rot[2],
                            ]
                        )
                        delta_rot = self.R_teleop_to_base @ delta_rot
                        rot_target = R.from_rotvec(delta_rot) * R.from_quat(self.ref_ee_quat)
                        target_quat = rot_target.as_quat()

                    self._solve(target_pos, target_quat)
        else:
            self.ref_ee_pos = None
            self.ref_ee_quat = None
            self.ref_controller_pos = None
            self.ref_controller_rot = None
            self.ref_heading_yaw = None

        self.q = clamp_configuration(self.model, self.q)
        self._output_command()

        if self.debug and int(self.t * 5) % 5 == 0:
            arm_q = [self.q[idx] for idx in self.arm_q_indices]
            print(f"t={self.t:.2f}s | q={np.round(arm_q, 3)} | active={self.active} side={side}")

        return self.q

    def _output_command(self):
        """子类实现：把 self.q 下发到仿真 / 真机。"""

    def _on_start(self):
        pass

    def _on_stop(self):
        pass

    def get_joint_states(self) -> dict:
        return {name: float(self.q[idx]) for name, idx in self.joint_indices.items()}

    def arm_q(self) -> np.ndarray:
        return np.array([self.q[idx] for idx in self.arm_q_indices], dtype=float)

    def run(self, rate: float = 50.0):
        print(f"Starting R1Arm teleoperation (rate={rate} Hz, side={self.control_side})")
        print("Press Ctrl+C to stop")
        print(f"Hold {self.control_side} grip to activate arm tracking")
        print("Press A button to reset to initial pose")

        self._on_start()
        dt = 1.0 / rate
        try:
            while self.running:
                loop_start = time.time()
                self.step(dt)
                elapsed = time.time() - loop_start
                sleep_t = dt - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.running = False
            self._on_stop()
