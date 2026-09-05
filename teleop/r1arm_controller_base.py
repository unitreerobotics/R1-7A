"""R1 单臂 / 双臂 PICO 遥操核心（相对位姿跟踪 + IK + Dex1），输出无关。

复用 A7 同款手柄相对跟踪语义：
- 按住 grip 激活跟踪
- 手柄 Δpos/Δrot → EE 目标（dex1_base_link）
- Placo QP（失败则 DLS）求 7 关节角
- trigger 驱动 Dex1（松开=开，按下=关），与 A7 GripperController 一致
- A 键复位

单臂：``control_side`` 指定左/右手柄。
双臂（``dual_arm=True``）：左右手柄分别控左右臂，忽略 ``control_side``。

子类通过 ``_output_command`` 接到 MuJoCo / 真机；真机可覆写 ``_send_dex1``。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.arm_model import clamp_configuration, joint_indices_by_name, link_pose, load_robot_model
from robot.gripper import Dex1SimGripper
from robot.r1arm_config import (
    ARM_INIT,
    ARM_JOINTS,
    DEX1_JOINTS,
    R_TELEOP_TO_BASE,
    R1ArmConfig,
    sim_to_closed_ratio,
)
from robot.xr_interface import get_key_value, make_xr_client
from teleop.ik_solver import solve_upper_arm_ik
from teleop.r1arm_placo_solver import PlacoArmIKSolver
from teleop.utils.geometry import R_HEADSET_TO_WORLD, quaternion_to_euler, quaternion_to_world
from teleop.utils.mock import make_mock_controller_poses, make_mock_grip_values, xr_pose_is_valid


@dataclass
class _SideTrackState:
    """单侧手柄相对跟踪状态（左右各一份）。"""

    ref_ee_pos: np.ndarray | None = None
    ref_ee_quat: np.ndarray | None = None
    ref_controller_pos: np.ndarray | None = None
    ref_controller_rot: object | None = None
    ref_heading_yaw: float | None = None
    active: bool = False

    def clear(self) -> None:
        self.ref_ee_pos = None
        self.ref_ee_quat = None
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_heading_yaw = None
        self.active = False


class R1ArmTeleopControllerBase:
    """单臂或双臂 tracking + IK + Dex1。"""

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
        self.dual_arm = bool(config.dual_arm)
        self.control_side = str(config.control_side).lower()
        if self.dual_arm:
            self.control_sides = ("left", "right")
        else:
            if self.control_side not in {"left", "right"}:
                raise ValueError(f"control_side must be left|right, got {config.control_side!r}")
            self.control_sides = (self.control_side,)
        self.R_teleop_to_base = R_TELEOP_TO_BASE.copy()

        self.urdf_path = Path(config.urdf_path)
        self.model, self.data = load_robot_model(str(self.urdf_path))
        self.joint_indices = joint_indices_by_name(self.model)
        self.arm_q_indices = [self.joint_indices[name] for name in ARM_JOINTS]
        self.dex1_q_indices = [self.joint_indices[name] for name in DEX1_JOINTS if name in self.joint_indices]
        if len(self.dex1_q_indices) != len(DEX1_JOINTS):
            missing = [n for n in DEX1_JOINTS if n not in self.joint_indices]
            raise RuntimeError(f"URDF missing Dex1 joints: {missing}")
        self.ee_frame = config.ee_frame

        # 右臂（及单臂）用 self.q；双臂时左臂另有一份同构 URDF 状态
        self.q = self._create_initial_q()
        self.q_left = self._create_initial_q() if self.dual_arm else None

        self.use_placo = False
        self.placo_solver = None
        self.placo_solver_left = None
        try:
            self.placo_solver = PlacoArmIKSolver(
                self.urdf_path,
                self.q,
                self.ee_frame,
                self.joint_indices,
                arm_joints=ARM_JOINTS,
                hold_joints=DEX1_JOINTS,
                arm_task_weight=config.arm_task_weight,
            )
            if self.dual_arm:
                self.placo_solver_left = PlacoArmIKSolver(
                    self.urdf_path,
                    self.q_left,
                    self.ee_frame,
                    self.joint_indices,
                    arm_joints=ARM_JOINTS,
                    hold_joints=DEX1_JOINTS,
                    arm_task_weight=config.arm_task_weight,
                )
            self.use_placo = True
            mode = "dual-arm" if self.dual_arm else "single-arm"
            print(f"Using Placo QP IK solver ({mode})")
        except Exception as exc:
            if self.debug:
                import traceback

                traceback.print_exc()
            print(f"Warning: Placo setup failed ({exc}); using DLS IK fallback.")

        print("Teleop frame aligned to r1arm base (A7 +X前/+Z上 → URDF -Y前/+Z上)")

        self.hand_ctrl = None
        self.last_gripper_update = time.monotonic()
        self.grippers = {
            side: Dex1SimGripper(
                open_value=config.dex1_open,
                close_value=config.dex1_close,
                speed=config.dex1_speed,
            )
            for side in self.control_sides
        }
        # 兼容旧字段：单臂时指向唯一夹爪
        self.gripper = self.grippers[self.control_sides[-1]]
        print(f"Dex1 sim gripper enabled (EE={self.ee_frame}, sides={list(self.control_sides)})")

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

        self.track: dict[str, _SideTrackState] = {side: _SideTrackState() for side in self.control_sides}
        # 兼容单臂调试字段
        self.active = False

        self.reset_flag = True
        self.is_reset_flag = True
        self.enable_a_button_reset = True
        self.reset_blend_frames = max(1, int(config.reset_blend_frames))
        self._reset_blending = False
        self._reset_blend_frame = 0
        self._q_blend_start: np.ndarray | None = None
        self._q_blend_goal: np.ndarray | None = None
        self._q_left_blend_start: np.ndarray | None = None
        self._q_left_blend_goal: np.ndarray | None = None
        self.running = True
        self.t = 0.0

    def _create_initial_q(self) -> np.ndarray:
        q = np.zeros(self.model.nq, dtype=float)
        for i, name in enumerate(ARM_JOINTS):
            if name in self.joint_indices:
                q[self.joint_indices[name]] = ARM_INIT[i]
        open_q = float(self.config.dex1_open)
        for name in DEX1_JOINTS:
            q[self.joint_indices[name]] = open_q
        return q

    def _q_for_side(self, side: str) -> np.ndarray:
        if side == "left" and self.dual_arm:
            return self.q_left
        return self.q

    def _set_q_for_side(self, side: str, q: np.ndarray) -> None:
        if side == "left" and self.dual_arm:
            self.q_left = q
        else:
            self.q = q

    def _placo_for_side(self, side: str):
        if side == "left" and self.dual_arm:
            return self.placo_solver_left
        return self.placo_solver

    def _link_pose(self, frame_name: str, q: np.ndarray | None = None) -> tuple:
        try:
            return link_pose(self.model, self.data, self.q if q is None else q, frame_name)
        except Exception:
            return None, None

    def _solve_dls(self, q: np.ndarray, target_pos: np.ndarray, target_quat: np.ndarray | None) -> np.ndarray:
        target_rot = None
        if target_quat is not None:
            target_rot = R.from_quat(target_quat).as_matrix()
        q_next, _ = solve_upper_arm_ik(
            self.model,
            self.data,
            q,
            ee_frame_name=self.ee_frame,
            ee_target_pos=target_pos,
            ee_target_rot=target_rot,
            elbow_frame_name=None,
            elbow_target_pos=None,
            q_indices=self.arm_q_indices,
            q_reference=q.copy(),
            control_mode="pose" if target_rot is not None else "position",
            debug=self.debug,
        )
        return q_next

    def _solve(self, side: str, target_pos: np.ndarray, target_quat: np.ndarray | None) -> None:
        q = self._q_for_side(side)
        solver = self._placo_for_side(side)
        if self.use_placo and solver is not None:
            sol = solver.solve(q, target_pos, target_quat)
            if sol is not None:
                q = q.copy()
                for idx in self.arm_q_indices:
                    q[idx] = sol[idx]
                self._set_q_for_side(side, q)
                return
        self._set_q_for_side(side, self._solve_dls(q, target_pos, target_quat))

    def reset(self):
        """Start a soft home recovery (linear joint cross-fade), not an instant snap."""
        self._q_blend_start = self.q.copy()
        self._q_blend_goal = self._create_initial_q()
        if self.dual_arm:
            self._q_left_blend_start = self.q_left.copy()
            self._q_left_blend_goal = self._create_initial_q()
        self._reset_blend_frame = 0
        self._reset_blending = True

        for st in self.track.values():
            st.clear()
        self.active = False
        self.reset_flag = True
        self.is_reset_flag = True
        if self.debug:
            print(
                f"[reset] soft blend to home over {self.reset_blend_frames} frames "
                f"(~{self.reset_blend_frames * 0.02:.2f}s @50Hz)"
            )

    def _step_reset_blend(self) -> bool:
        if not self._reset_blending or self._q_blend_start is None or self._q_blend_goal is None:
            return False

        self._reset_blend_frame += 1
        w_new = float(np.clip(self._reset_blend_frame / float(self.reset_blend_frames), 0.0, 1.0))
        w_old = 1.0 - w_new
        self.q = clamp_configuration(self.model, w_old * self._q_blend_start + w_new * self._q_blend_goal)
        if self.dual_arm and self._q_left_blend_start is not None and self._q_left_blend_goal is not None:
            self.q_left = clamp_configuration(
                self.model, w_old * self._q_left_blend_start + w_new * self._q_left_blend_goal
            )

        if self._reset_blend_frame >= self.reset_blend_frames:
            self.q = clamp_configuration(self.model, self._q_blend_goal.copy())
            if self.dual_arm and self._q_left_blend_goal is not None:
                self.q_left = clamp_configuration(self.model, self._q_left_blend_goal.copy())
            for side, gripper in self.grippers.items():
                gripper.reset()
                q = self._q_for_side(side)
                for idx in self.dex1_q_indices:
                    q[idx] = gripper.current
                self._set_q_for_side(side, q)
            self._send_dex1_all()
            self._reset_blending = False
            self._q_blend_start = None
            self._q_blend_goal = None
            self._q_left_blend_start = None
            self._q_left_blend_goal = None
            self._reset_blend_frame = 0
            if self.debug:
                print("[reset] soft blend done")
            return False
        return True

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

    def get_grip_state(self, side: str | None = None) -> float:
        side = side or self.control_side
        if self.mock_xr or self.xr_client is None:
            left_grip, right_grip, _, _ = make_mock_grip_values(self.t)
            return left_grip if side == "left" else right_grip
        return get_key_value(self.xr_client, f"{side}_grip")

    def get_trigger_state(self, side: str | None = None) -> float:
        side = side or self.control_side
        if self.mock_xr or self.xr_client is None:
            _, _, left_trigger, right_trigger = make_mock_grip_values(self.t)
            return left_trigger if side == "left" else right_trigger
        return get_key_value(self.xr_client, f"{side}_trigger")

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

    def _update_dex1_side(self, side: str, dt: float) -> None:
        trigger = self.get_trigger_state(side)
        if trigger < self.config.trigger_deadband:
            trigger = 0.0
        value = self.grippers[side].update(trigger, dt)
        q = self._q_for_side(side).copy()
        for idx in self.dex1_q_indices:
            q[idx] = value
        self._set_q_for_side(side, q)

    def _send_dex1_all(self) -> None:
        if self.hand_ctrl is None:
            return
        if self.dual_arm:
            left_v = self.grippers["left"].current
            right_v = self.grippers["right"].current
            left_r = sim_to_closed_ratio(left_v, self.config.dex1_open, self.config.dex1_close)
            right_r = sim_to_closed_ratio(right_v, self.config.dex1_open, self.config.dex1_close)
            self.hand_ctrl.set_gripper_ratios(left_r, right_r)
            return
        side = self.control_side
        ratio = sim_to_closed_ratio(
            self.grippers[side].current, self.config.dex1_open, self.config.dex1_close
        )
        if side == "left":
            self.hand_ctrl.set_gripper_ratios(ratio, 0.0)
        else:
            self.hand_ctrl.set_gripper_ratios(0.0, ratio)

    def _track_side(self, side: str, controller_poses: dict) -> None:
        st = self.track[side]
        st.active = self.get_grip_state(side) > 0.9
        controller_pose = controller_poses.get(f"{side}_controller")
        q = self._q_for_side(side)

        if controller_pose is not None and st.active:
            ee_pos, ee_quat = self._link_pose(self.ee_frame, q)
            if ee_pos is not None:
                ctrl_pos = R_HEADSET_TO_WORLD @ controller_pose[:3]
                ctrl_rot_world = quaternion_to_world(controller_pose[3:7])

                if st.ref_ee_pos is None:
                    st.ref_ee_pos = ee_pos.copy()
                    st.ref_ee_quat = ee_quat.copy() if ee_quat is not None else None
                    st.ref_controller_pos = ctrl_pos.copy()
                    st.ref_controller_rot = ctrl_rot_world
                    euler = quaternion_to_euler(*ctrl_rot_world.as_quat())
                    st.ref_heading_yaw = float(euler[2])
                else:
                    delta_pos = ctrl_pos - st.ref_controller_pos
                    yaw_ref = st.ref_heading_yaw or 0.0
                    c = math.cos(-yaw_ref)
                    s = math.sin(-yaw_ref)
                    delta_pos_corrected = np.array(
                        [
                            c * delta_pos[0] - s * delta_pos[1],
                            s * delta_pos[0] + c * delta_pos[1],
                            delta_pos[2],
                        ]
                    )
                    delta_pos_base = self.R_teleop_to_base @ delta_pos_corrected
                    target_pos = st.ref_ee_pos + delta_pos_base * self.scale_factor

                    target_quat = None
                    if st.ref_controller_rot is not None and st.ref_ee_quat is not None:
                        rot_delta = ctrl_rot_world * st.ref_controller_rot.inv()
                        delta_rot = rot_delta.as_rotvec()
                        delta_rot = np.array(
                            [
                                c * delta_rot[0] - s * delta_rot[1],
                                s * delta_rot[0] + c * delta_rot[1],
                                delta_rot[2],
                            ]
                        )
                        delta_rot = self.R_teleop_to_base @ delta_rot
                        rot_target = R.from_rotvec(delta_rot) * R.from_quat(st.ref_ee_quat)
                        target_quat = rot_target.as_quat()

                    self._solve(side, target_pos, target_quat)
        else:
            st.clear()

    def step(self, dt: float) -> np.ndarray:
        self.t += dt

        if self._reset_blending:
            self._step_reset_blend()
            self._output_command()
            return self.q

        controller_poses, valid = self.read_controller_poses()
        if not valid:
            self._output_command()
            return self.q

        if self.get_reset_button() and not self.is_reset_flag:
            self.reset()
            self._step_reset_blend()
            self._output_command()
            return self.q
        if not self.get_reset_button():
            self.is_reset_flag = False

        for side in self.control_sides:
            self._track_side(side, controller_poses)
        self.active = any(self.track[s].active for s in self.control_sides)

        now = time.monotonic()
        gripper_dt = max(0.0, now - self.last_gripper_update)
        self.last_gripper_update = now
        for side in self.control_sides:
            self._update_dex1_side(side, gripper_dt)
        self._send_dex1_all()

        self.q = clamp_configuration(self.model, self.q)
        if self.dual_arm:
            self.q_left = clamp_configuration(self.model, self.q_left)

        self._output_command()

        if self.debug and int(self.t * 5) % 5 == 0:
            parts = []
            for side in self.control_sides:
                q = self._q_for_side(side)
                arm_q = [q[idx] for idx in self.arm_q_indices]
                parts.append(
                    f"{side}={np.round(arm_q, 3)} act={self.track[side].active}"
                )
            print(f"t={self.t:.2f}s | " + " | ".join(parts))

        return self.q

    def _output_command(self):
        """子类实现：把 self.q（及 dual 的 q_left）下发到仿真 / 真机。"""

    def _on_start(self):
        pass

    def _on_stop(self):
        if self.hand_ctrl is not None:
            self.hand_ctrl.stop()

    def get_joint_states(self) -> dict:
        return {name: float(self.q[idx]) for name, idx in self.joint_indices.items()}

    def arm_q(self, side: str = "right") -> np.ndarray:
        q = self._q_for_side(side)
        return np.array([q[idx] for idx in self.arm_q_indices], dtype=float)

    def dex1_q(self, side: str = "right") -> np.ndarray:
        q = self._q_for_side(side)
        return np.array([q[idx] for idx in self.dex1_q_indices], dtype=float)

    def sim_q(self) -> np.ndarray:
        """MuJoCo 控制向量。

        单臂：右臂 7+2。
        双臂：右臂 7+2 再左臂 7+2（与 scene_dual_arm actuator 序一致）。
        """
        right = np.hstack([self.arm_q("right"), self.dex1_q("right")])
        if not self.dual_arm:
            return right
        left = np.hstack([self.arm_q("left"), self.dex1_q("left")])
        return np.hstack([right, left])

    def run(self, rate: float = 50.0):
        if self.dual_arm:
            print(f"Starting R1Arm dual-arm teleoperation (rate={rate} Hz)")
            print("Hold left/right grip to activate corresponding arm")
            print("Hold left/right trigger to close corresponding Dex1")
        else:
            print(f"Starting R1Arm teleoperation (rate={rate} Hz, side={self.control_side})")
            print(f"Hold {self.control_side} grip to activate arm tracking")
            print(f"Hold {self.control_side} trigger to close Dex1 gripper")
        print(
            f"Press A button to soft-reset to home "
            f"({self.reset_blend_frames} frames cross-fade)"
        )
        print("Press Ctrl+C to stop")

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
