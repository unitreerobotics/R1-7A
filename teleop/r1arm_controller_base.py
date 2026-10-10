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
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

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
        allow_xr_fallback: bool = True,
    ):
        self.config = config
        self.mock_xr = mock_xr
        self.debug = debug
        self.allow_xr_fallback = bool(allow_xr_fallback)
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
        self._solver_seeds = {"right": self.q.copy()}
        if self.dual_arm:
            self._solver_seeds["left"] = self.q_left.copy()

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
                joint_velocity_limits=config.ik_joint_velocity_limits,
                default_velocity_limit=(
                    config.ik_qp_velocity_limit
                    if config.ik_qp_velocity_limit is not None else 0.0
                ),
                default_acceleration_limit=config.ik_qp_acceleration_limit,
                default_jerk_limit=config.ik_qp_jerk_limit,
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
                    joint_velocity_limits=config.ik_joint_velocity_limits,
                    default_velocity_limit=(
                        config.ik_qp_velocity_limit
                        if config.ik_qp_velocity_limit is not None else 0.0
                    ),
                    default_acceleration_limit=config.ik_qp_acceleration_limit,
                    default_jerk_limit=config.ik_qp_jerk_limit,
                )
            self.use_placo = True
            mode = "dual-arm" if self.dual_arm else "single-arm"
            qp_vlim = config.ik_qp_velocity_limit
            vlim_note = f", QP v≤{qp_vlim:g} rad/s" if qp_vlim else ""
            print(f"Using Placo QP IK solver ({mode}{vlim_note})")
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
                if not self.allow_xr_fallback:
                    raise RuntimeError(
                        "Could not connect to XRoboToolkit and XR fallback is disabled"
                    ) from e
                print(f"Warning: Could not connect to XRoboToolkit: {e}")
                print("Falling back to mock XR input")
                self.xr_client = None
                self.mock_xr = True

        self.track: dict[str, _SideTrackState] = {side: _SideTrackState() for side in self.control_sides}
        # One authoritative joint trajectory state per arm side. Positions are
        # stored separately from self.q so IK seeds and command trajectories
        # cannot silently overwrite one another.
        self._trajectory_q_cmd: dict[str, np.ndarray] = {}
        self._trajectory_v_cmd: dict[str, np.ndarray] = {}
        self._trajectory_a_cmd: dict[str, np.ndarray] = {}
        self.velocity_only_shaper = False
        self._trigger_samples: dict[str, tuple[float, bool, float]] = {
            side: (0.0, False, 0.0) for side in self.control_sides
        }
        self._control_seq = 0
        self._control_sample: dict | None = None
        self._control_sample_lock = threading.Lock()
        self._capture_control_samples = False
        self._captured_control_samples: deque[dict] = deque()
        self._last_ee_intent: dict[str, dict] = {}
        self._last_ik_raw: dict[str, np.ndarray] = {}
        self._last_ik_solver: dict[str, str] = {}
        # XR 帧时钟感知缓存（P2）：服务器帧时间戳 + 上一次的有效位姿快照
        self._xr_last_stamp: int | None = None
        self._xr_pose_cache: dict | None = None
        self._xr_pose_raw: dict | None = None
        self._xr_filter_state: dict[str, dict] = {}
        self._xr_filter_last_stamp_ns: int | None = None
        self._xr_filter_dt: float = 1.0 / 30.0
        # 兼容单臂调试字段
        self.active = False

        self.reset_flag = True
        self.is_reset_flag = True
        self.enable_a_button_reset = True
        self.reset_blend_frames = max(1, int(config.reset_blend_frames))
        self.rate_hz = 50.0
        self._reset_blending = False
        self._reset_blend_deadline = 0.0
        self._reset_settle_count = 0
        # Kept as reset lifecycle markers; the actual path is the shared
        # trajectory generator, not a separate cross-fade interpolation.
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
            **({
                "max_step": self._dls_max_step,
                "iterations": 1,
            } if hasattr(self, "_dls_max_step") else {}),
        )
        return q_next

    def _ensure_trajectory_state(self, side: str, q: np.ndarray) -> None:
        if side in self._trajectory_q_cmd:
            return
        self._trajectory_q_cmd[side] = np.asarray(
            [q[idx] for idx in self.arm_q_indices], dtype=float
        )
        self._trajectory_v_cmd[side] = np.zeros(len(self.arm_q_indices))
        self._trajectory_a_cmd[side] = np.zeros(len(self.arm_q_indices))

    def _reset_trajectory_state(self, side: str, q: np.ndarray) -> None:
        self._trajectory_q_cmd[side] = np.asarray(
            [q[idx] for idx in self.arm_q_indices], dtype=float
        )
        self._trajectory_v_cmd[side] = np.zeros(len(self.arm_q_indices))
        self._trajectory_a_cmd[side] = np.zeros(len(self.arm_q_indices))

    def _trajectory_step(
        self, side: str, q_reference: np.ndarray, q_desired: np.ndarray, dt: float | None = None
    ) -> np.ndarray:
        """Advance the sole q_cmd/v_cmd/a_cmd trajectory generator by one cycle."""
        rate_hz = getattr(self, "rate_hz", 50.0) or 50.0
        dt = min(max(float(dt if dt is not None else getattr(self, "_step_dt", 1.0 / rate_hz)), 1e-4), 0.1)
        self._ensure_trajectory_state(side, q_reference)
        q_cmd = self._trajectory_q_cmd[side].copy()
        v_cmd = self._trajectory_v_cmd[side].copy()
        a_cmd = self._trajectory_a_cmd[side].copy()
        v_limit_cfg = self.config.trajectory_max_speed_rad_s
        v_max = (float(v_limit_cfg)
                 if v_limit_cfg is not None and v_limit_cfg > 0 else float("inf"))
        a_max = max(float(self.config.ik_max_acc_per_s2), 1e-3)
        jerk_max = max(float(self.config.ik_max_jerk_per_s3), 1e-3)
        q_next = q_reference.copy()
        for k, idx in enumerate(self.arm_q_indices):
            error = float(q_desired[idx] - q_cmd[k])
            if abs(error) <= 1e-8 and abs(v_cmd[k]) <= 1e-4:
                q_cmd[k] = float(q_desired[idx])
                v_cmd[k] = 0.0
                a_cmd[k] = 0.0
                q_next[idx] = q_cmd[k]
                continue

            # Critically damped position/velocity feedback converges at a fixed
            # home target; distance-only braking can form a discrete limit cycle.
            a_desired = float(np.clip(
                self.config.trajectory_position_gain * error
                - self.config.trajectory_velocity_gain * v_cmd[k],
                -a_max, a_max,
            ))
            # Acceleration/velocity are hard bounds; jerk is a smoothness
            # bound. At a speed boundary those intervals can be disjoint
            # (e.g. still accelerating toward +v_max while jerk forbids
            # braking quickly enough). In that case relax jerk for this
            # sample instead of passing an invalid [lower, upper] to clip,
            # which would otherwise silently exceed the speed limit.
            a_lower = max(-a_max, (-v_max - v_cmd[k]) / dt)
            a_upper = min(a_max, (v_max - v_cmd[k]) / dt)
            jerk_lower = max(-a_max, a_cmd[k] - jerk_max * dt)
            jerk_upper = min(a_max, a_cmd[k] + jerk_max * dt)
            feasible_lower = max(a_lower, jerk_lower)
            feasible_upper = min(a_upper, jerk_upper)
            if feasible_lower <= feasible_upper:
                a_cmd[k] = float(np.clip(a_desired, feasible_lower, feasible_upper))
            else:
                a_cmd[k] = float(np.clip(a_desired, a_lower, a_upper))
            v_new = v_cmd[k] + a_cmd[k] * dt
            q_new = q_cmd[k] + 0.5 * (v_cmd[k] + v_new) * dt
            q_cmd[k] = q_new
            v_cmd[k] = v_new
            q_next[idx] = q_new

        q_next = clamp_configuration(self.model, q_next)
        for k, idx in enumerate(self.arm_q_indices):
            if abs(q_next[idx] - q_cmd[k]) > 1e-8:
                # Joint-limit projection is a safety event; synchronize the
                # trajectory state at the feasible boundary.
                q_cmd[k] = float(q_next[idx])
                v_cmd[k] = 0.0
                a_cmd[k] = 0.0
        self._trajectory_q_cmd[side] = q_cmd
        self._trajectory_v_cmd[side] = v_cmd
        self._trajectory_a_cmd[side] = a_cmd
        return q_next

    def _velocity_only_step(
        self, side: str, q_reference: np.ndarray, q_desired: np.ndarray
    ) -> np.ndarray:
        """Track the IK target with only a per-cycle joint-speed bound."""
        rate_hz = getattr(self, "rate_hz", 50.0) or 50.0
        dt = min(max(float(getattr(self, "_step_dt", 1.0 / rate_hz)), 1e-4), 0.1)
        self._ensure_trajectory_state(side, q_reference)
        q_cmd = self._trajectory_q_cmd[side].copy()
        v_prev = self._trajectory_v_cmd[side].copy()
        v_limit = self.config.trajectory_max_speed_rad_s
        v_max = float(v_limit) if v_limit is not None and v_limit > 0 else float("inf")
        q_next = q_reference.copy()
        for k, idx in enumerate(self.arm_q_indices):
            q_target = float(q_desired[idx])
            q_cmd[k] += float(np.clip(q_target - q_cmd[k], -v_max * dt, v_max * dt))
            q_next[idx] = q_cmd[k]
        q_next = clamp_configuration(self.model, q_next)
        v_cmd = np.asarray([q_next[idx] for idx in self.arm_q_indices], dtype=float)
        v_cmd = (v_cmd - self._trajectory_q_cmd[side]) / dt
        a_cmd = (v_cmd - v_prev) / dt
        self._trajectory_q_cmd[side] = np.asarray(
            [q_next[idx] for idx in self.arm_q_indices], dtype=float
        )
        self._trajectory_v_cmd[side] = v_cmd
        self._trajectory_a_cmd[side] = a_cmd
        return q_next

    def _trajectory_state(self, side: str, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        self._ensure_trajectory_state(side, q)
        return (
            self._trajectory_q_cmd[side].copy(),
            self._trajectory_v_cmd[side].copy(),
            self._trajectory_a_cmd[side].copy(),
        )

    def _action_trajectory_state(
        self, side: str, q: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self._trajectory_state(side, q)

    def publish_control_sample(self, sample: dict) -> None:
        """Atomically publish a complete, single-control-cycle snapshot."""
        with self._control_sample_lock:
            snapshot = {
                key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in sample.items()
            }
            self._control_sample = snapshot
            if self._capture_control_samples:
                self._captured_control_samples.append(snapshot)

    def start_control_sample_capture(self) -> None:
        with self._control_sample_lock:
            self._captured_control_samples.clear()
            self._capture_control_samples = True

    def drain_control_sample_capture(self, *, stop: bool = False) -> tuple[list[dict], dict | None]:
        """Atomically drain captured cycles and return the matching latest snapshot."""
        with self._control_sample_lock:
            if stop:
                self._capture_control_samples = False
            samples = list(self._captured_control_samples)
            self._captured_control_samples.clear()
            latest = self._control_sample
            if latest is not None:
                latest = {
                    key: value.copy() if isinstance(value, np.ndarray) else value
                    for key, value in latest.items()
                }
            return samples, latest

    def control_sample(self) -> dict | None:
        with self._control_sample_lock:
            if self._control_sample is None:
                return None
            return {
                key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in self._control_sample.items()
            }

    def _solve(self, side: str, target_pos: np.ndarray, target_quat: np.ndarray | None) -> None:
        q = self._q_for_side(side)
        solver_seed = self._solver_seeds.get(side, q)
        solver = self._placo_for_side(side)
        if self.use_placo and solver is not None:
            sol = solver.solve(solver_seed, target_pos, target_quat)
            if sol is not None:
                q_next = q.copy()
                for idx in self.arm_q_indices:
                    q_next[idx] = sol[idx]
                self._last_ik_raw[side] = np.array([q_next[i] for i in self.arm_q_indices])
                self._last_ik_solver[side] = "placo"
                q_shaped = (self._velocity_only_step(side, q, q_next)
                            if self.velocity_only_shaper else self._trajectory_step(side, q, q_next))
                self._set_q_for_side(side, q_shaped)
                self._solver_seeds[side] = q_shaped.copy()
                return
        # Placo 失败（不可达/奇异/异常）→ DLS 回退。DLS 在远离目标时可能
        # 返回一个大幅解——q 不动的回退语义应保持"上一个可行目标"：
        # DLS falls through the same q/v/a trajectory generator, preserving
        # velocity and acceleration across solver fallback.
        q_raw = self._solve_dls(solver_seed, target_pos, target_quat)
        self._last_ik_raw[side] = np.array([q_raw[i] for i in self.arm_q_indices])
        self._last_ik_solver[side] = "dls"
        q_shaped = (self._velocity_only_step(side, q, q_raw)
                    if self.velocity_only_shaper else self._trajectory_step(side, q, q_raw))
        self._set_q_for_side(side, q_shaped)
        self._solver_seeds[side] = q_shaped.copy()

    def reset(self):
        """Send the home target through the shared velocity/acceleration state."""
        self._q_blend_start = self.q.copy()
        self._q_blend_goal = self._create_initial_q()
        if self.dual_arm:
            self._q_left_blend_start = self.q_left.copy()
            self._q_left_blend_goal = self._create_initial_q()
        # Deadline follows configured command rate; actual movement remains
        # governed by the shared q/v/a trajectory state below.
        rate_hz = getattr(self, "rate_hz", 50.0) or 50.0
        self._reset_blend_deadline = time.monotonic() + self.reset_blend_frames / rate_hz
        self._reset_settle_count = 0
        self._reset_last_diag_time = 0.0
        self._reset_blending = True

        for st in self.track.values():
            st.clear()
        for side in self.control_sides:
            self._ensure_trajectory_state(side, self._q_for_side(side))
        self.active = False
        self.reset_flag = True
        self.is_reset_flag = True
        if self.debug:
            print(
                f"[reset] trajectory to home over {self.reset_blend_frames / rate_hz:.2f}s "
                f"(wall-clock; was frame-count at 50Hz)"
            )

    def _step_reset_blend(self) -> bool:
        if not self._reset_blending or self._q_blend_start is None or self._q_blend_goal is None:
            return False
        self.q = self._trajectory_step("right", self.q, self._q_blend_goal)
        self._solver_seeds["right"] = self.q.copy()
        self._last_ik_raw["right"] = np.array(
            [self._q_blend_goal[i] for i in self.arm_q_indices]
        )
        self._last_ik_solver["right"] = "reset_blend"
        if self.dual_arm and self._q_left_blend_start is not None and self._q_left_blend_goal is not None:
            self.q_left = self._trajectory_step("left", self.q_left, self._q_left_blend_goal)
            self._solver_seeds["left"] = self.q_left.copy()

        at_goal = True
        stopped = True
        feedback_settled = True
        diagnostics = []
        for side, goal in (("right", self._q_blend_goal), ("left", self._q_left_blend_goal)):
            if side == "left" and not self.dual_arm:
                continue
            q_cmd, v_cmd, a_cmd = self._trajectory_state(side, self._q_for_side(side))
            q_cmd_error = float(np.max(np.abs(q_cmd - goal[self.arm_q_indices])))
            v_cmd_max = float(np.max(np.abs(v_cmd)))
            a_cmd_max = float(np.max(np.abs(a_cmd)))
            side_at_goal = q_cmd_error < 1e-3
            side_stopped = v_cmd_max < 0.02 and a_cmd_max < 0.1
            at_goal &= side_at_goal
            stopped &= side_stopped
            measured = self._reset_feedback_state(side)
            if measured is not None:
                q_measured, dq_measured = measured
                q_measured_error = float(np.max(np.abs(q_measured - goal[self.arm_q_indices])))
                dq_measured_max = float(np.max(np.abs(dq_measured)))
                side_feedback_settled = q_measured_error < 0.05 and dq_measured_max < 0.05
                feedback_settled &= side_feedback_settled
            else:
                q_measured_error = dq_measured_max = None
                side_feedback_settled = True
            diagnostics.append(
                f"{side}: cmd_err={q_cmd_error:.5f}rad v={v_cmd_max:.4f} a={a_cmd_max:.4f} "
                f"cmd_ok={side_at_goal and side_stopped} "
                f"meas_err={'n/a' if q_measured_error is None else f'{q_measured_error:.5f}rad'} "
                f"meas_dq={'n/a' if dq_measured_max is None else f'{dq_measured_max:.4f}'} "
                f"meas_ok={side_feedback_settled}"
            )
        if at_goal and stopped and feedback_settled:
            self._reset_settle_count += 1
        else:
            self._reset_settle_count = 0
        settle_cycles = max(1, int(math.ceil(0.1 * (getattr(self, "rate_hz", 50.0) or 50.0))))
        now = time.monotonic()
        if now - self._reset_last_diag_time >= 1.0:
            self._reset_last_diag_time = now
            print(
                "[reset] settling: " + "; ".join(diagnostics)
                + f"; settle={self._reset_settle_count}/{settle_cycles}"
                + f" deadline={'passed' if now >= self._reset_blend_deadline else 'pending'}"
            )
        if (
            now >= self._reset_blend_deadline
            and at_goal and stopped and feedback_settled
            and self._reset_settle_count >= settle_cycles
        ):
            self._solver_seeds["right"] = self.q.copy()
            if self.dual_arm and self._q_left_blend_goal is not None:
                self._solver_seeds["left"] = self.q_left.copy()
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
            print("[reset] home reached and measured arm settled; grip tracking enabled")
            return False
        return True

    def _reset_feedback_state(self, side: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Optional measured ``(q, dq)`` in model arm-joint order for reset settling."""
        return None

    def _publish_ik_diagnostics(self, controller_poses: dict, xr_valid: bool) -> None:
        # Real controllers publish the authoritative sample after feedback and
        # command output. Publishing this interim IK-only snapshot races the
        # recorder and creates apparent multi-second sample gaps.
        if getattr(self, "motion_mode", False):
            return
        self._control_seq += 1
        control_time_ns = time.monotonic_ns()
        xr_frame_stamp_ns = self._xr_last_stamp or self._xr_filter_last_stamp_ns
        raw_poses = {
            name: np.asarray(pose, dtype=float).tolist()
            for name, pose in (self._xr_pose_raw or {}).items()
        }
        filtered_poses = {
            name: np.asarray(pose, dtype=float).tolist()
            for name, pose in controller_poses.items()
        }
        filter_state = {
            name: {
                "pose": state["pose"].tolist(),
                "derivative": state["derivative"].tolist(),
                "dt": float(state["dt"]),
                "stamp_ns": int(state["stamp_ns"]),
            }
            for name, state in self._xr_filter_state.items()
        }
        jump_rejected_poses = {
            name: np.asarray(pose, dtype=float).tolist()
            for name, pose in getattr(self, "_xr_prev_poses", {}).items()
        }
        for side in self.control_sides:
            q = self._q_for_side(side)
            q_cmd, v_cmd, a_cmd = self._trajectory_state(side, q)
            xr_name = f"{side}_controller"
            raw_pose = (self._xr_pose_raw or {}).get(xr_name)
            filtered_pose = controller_poses.get(xr_name)
            track = self.track[side]
            anchor = {
                "ee_position": None if track.ref_ee_pos is None else track.ref_ee_pos.tolist(),
                "ee_quaternion_xyzw": None if track.ref_ee_quat is None else track.ref_ee_quat.tolist(),
                "controller_position": None if track.ref_controller_pos is None else track.ref_controller_pos.tolist(),
                "controller_rotation_xyzw": (
                    None if track.ref_controller_rot is None else track.ref_controller_rot.as_quat().tolist()
                ),
                "heading_yaw": track.ref_heading_yaw,
            }
            self.publish_control_sample({
                "side": side,
                "control_seq": self._control_seq,
                "control_time_ns": control_time_ns,
                "xr_frame_stamp_ns": xr_frame_stamp_ns,
                "xr_valid": bool(xr_valid),
                "grip_active": bool(track.active),
                "teleop_anchor": anchor,
                "q_ik_raw": self._last_ik_raw.get(side, self.arm_q(side)),
                "q_purpose": q_cmd,
                "dq_purpose": v_cmd,
                "ddq_purpose": a_cmd,
                "ee_intent": self._last_ee_intent.get(side, {}),
                "ik_solver": self._last_ik_solver.get(side, "unknown"),
                "xr_raw": np.asarray(raw_pose, dtype=float).tolist() if raw_pose is not None else [],
                "xr_filtered": np.asarray(filtered_pose, dtype=float).tolist() if filtered_pose is not None else [],
                "xr_poses_raw": raw_poses,
                "xr_poses_filtered": filtered_poses,
                "xr_poses_jump_rejected": jump_rejected_poses,
                "xr_filter_state": filter_state,
                "xr_jump_reject_count": int(getattr(self, "_xr_jump_rejects", 0)),
            })

    def read_controller_poses(self) -> tuple:
        if self.mock_xr or self.xr_client is None:
            raw = make_mock_controller_poses(self.t)
            self._xr_pose_raw = {k: v.copy() for k, v in raw.items()}
            return self._filter_xr_poses(raw, time.monotonic_ns()), True
        # P2 修复：帧时钟感知。XR 服务端按自己的帧率（~30Hz）推送位姿，
        # 而控制环 50Hz 每拍都读——同一个位姿被重复消费 1~2 次且交替不
        # 均匀（33ms 与 20ms 不成整数比），IK 目标流成为"30Hz 阶梯 ×
        # 50Hz 采样"的拍频调制信号。只在服务器帧时间戳更新时读新位姿，
        # 其余拍沿用上一次的快照——目标更新节拍与 XR 服务端一致，拍频
        # 调制从源头消失（读取代价：一次 int 调用 vs 三次 pose 拷贝）。
        stamp = self.xr_client.get_frame_stamp_ns()
        if stamp is not None and stamp == self._xr_last_stamp:
            if self._xr_pose_cache is not None:
                return self._xr_pose_cache, True
        try:
            controller_poses = {}
            for name in ["headset", "left_controller", "right_controller"]:
                pose = self.xr_client.get_pose_by_name(name)
                if xr_pose_is_valid(pose):
                    controller_poses[name] = np.asarray(pose, dtype=float)[:7]
                else:
                    if not self.allow_xr_fallback:
                        if self.debug:
                            print(f"XR pose invalid: {name}; holding current command")
                        return {}, False
                    return make_mock_controller_poses(self.t), True
            self._xr_pose_raw = {k: v.copy() for k, v in controller_poses.items()}
            out = self._reject_xr_jumps(controller_poses)
            stamp_ns = int(stamp) if stamp is not None else time.monotonic_ns()
            out = self._filter_xr_poses(out, stamp_ns)
            self._xr_pose_cache = out
            self._xr_last_stamp = stamp
            return out, True
        except Exception as e:
            if self.debug:
                print(f"XR read error: {e}")
            if not self.allow_xr_fallback:
                return {}, False
            return make_mock_controller_poses(self.t), True

    def _filter_xr_poses(self, poses: dict, stamp_ns: int) -> dict:
        """One-Euro filter valid XR poses once per new XR frame."""
        if stamp_ns == self._xr_filter_last_stamp_ns:
            return {k: v.copy() for k, v in self._xr_pose_cache.items()} if self._xr_pose_cache else poses
        cutoff = max(float(getattr(self.config, "xr_filter_min_cutoff", 8.0)), 1e-3)
        beta = max(float(getattr(self.config, "xr_filter_beta", 1.0)), 0.0)
        if self._xr_filter_last_stamp_ns is None:
            dt = 1.0 / 30.0
        else:
            dt = (int(stamp_ns) - self._xr_filter_last_stamp_ns) * 1e-9
            if not 1e-4 <= dt <= 0.2:
                dt = 1.0 / 30.0
        self._xr_filter_last_stamp_ns = int(stamp_ns)
        self._xr_filter_dt = dt
        out = {}
        for name, pose in poses.items():
            pose = np.asarray(pose, dtype=float)[:7].copy()
            state = self._xr_filter_state.get(name)
            if state is None:
                self._xr_filter_state[name] = {
                    "pose": pose.copy(), "derivative": np.zeros(3), "stamp_ns": int(stamp_ns),
                    "dt": dt,
                }
                out[name] = pose
                continue
            prev = state["pose"]
            if (np.linalg.norm(pose[:3] - prev[:3]) > self._XR_JUMP_POS_M
                    or np.linalg.norm((R.from_quat(prev[3:7]).inv() * R.from_quat(pose[3:7])).as_rotvec())
                    > self._XR_JUMP_ROT_RAD):
                # Jump detector accepted a persistent relocation after its
                # reject window; restart filter state instead of easing across
                # the whole discontinuity.
                self._xr_filter_state[name] = {
                    "pose": pose.copy(), "derivative": np.zeros(3), "stamp_ns": int(stamp_ns),
                    "dt": dt,
                }
                out[name] = pose
                continue
            filter_dt = max(float(state.get("dt", dt)), 1e-4)
            raw_velocity = (pose[:3] - prev[:3]) / filter_dt
            alpha_d = self._one_euro_alpha(filter_dt, 1.0)
            derivative = alpha_d * raw_velocity + (1.0 - alpha_d) * state["derivative"]
            derivative_norm = float(np.linalg.norm(derivative))
            if derivative_norm > 20.0:
                derivative *= 20.0 / derivative_norm
            speed = float(np.linalg.norm(derivative))
            alpha = self._one_euro_alpha(filter_dt, cutoff + beta * speed)
            filtered = pose.copy()
            filtered[:3] = alpha * pose[:3] + (1.0 - alpha) * prev[:3]
            rotations = R.from_quat(np.vstack((prev[3:7], pose[3:7])))
            filtered[3:7] = Slerp([0.0, 1.0], rotations)([alpha]).as_quat()[0]
            self._xr_filter_state[name] = {
                "pose": filtered.copy(), "derivative": derivative, "stamp_ns": int(stamp_ns),
                "dt": dt,
            }
            out[name] = filtered
        return out

    @staticmethod
    def _one_euro_alpha(dt: float, cutoff: float) -> float:
        tau = 1.0 / (2.0 * math.pi * max(float(cutoff), 1e-3))
        return 1.0 / (1.0 + tau / max(float(dt), 1e-6))

    # P1: XR 位姿跳变检测。跟踪丢失/重捕获的瞬间，XR 会上报一个跟上一帧
    # 完全无关的位姿（几厘米甚至几米的单帧跳变）。这种跳变直接进 IK 会产
    # 生一帧大步目标——被 clamp 后录进 action 的轨迹仍然是"从未执行过的
    # 意图跳变"。逐帧位置/姿态差超过阈值时判定为跳变：保留上一帧位姿，
    # 并清空跟踪参考（下一拍 grip 仍按住时会重新锚定相对位姿，而不是把
    # 跳变后的 delta 累加进去）。
    #
    # 毛刺 vs 搬位：单帧尖峰（回到原位）hold 是对的；但手柄被真实放到新
    # 位置（放下/交接/换手）时，新位姿会持续稳定存在——连续 N 帧都在同一
    # 个"新位置"附近，说明这不是噪声而是真实搬位。持续拒绝会让参考永久
    # 冻结、目标卡死、日志每帧刷屏（真机日志实测：连刷 50+ 行 0.4m 拒绝）。
    # 连续拒绝 ≥ N 帧后接受新位置为现实，清锚重捕获。
    _XR_JUMP_POS_M = 0.30  # 单帧 30cm 位移 = 跳变（人手 50Hz 下限 ~0.5m/s）
    _XR_JUMP_ROT_RAD = 1.0  # 单帧 57° 转动 = 跳变
    _XR_JUMP_MAX_REJECTS = 10  # 连续拒绝上限：超过 = 真实搬位，接受新位置

    def _reject_xr_jumps(self, poses: dict) -> dict:
        prev = getattr(self, "_xr_prev_poses", None)
        if prev is None:
            self._xr_prev_poses = {k: v.copy() for k, v in poses.items()}
            self._xr_jump_rejects = 0
            return poses
        rejects = getattr(self, "_xr_jump_rejects", 0)
        jumped = []
        out = {}
        for name, pose in poses.items():
            ref = prev.get(name)
            if ref is None:
                out[name] = pose
                continue
            dpos = float(np.linalg.norm(pose[:3] - ref[:3]))
            drot = R.from_quat(pose[3:7]).inv() * R.from_quat(ref[3:7])
            drot = float(np.linalg.norm(drot.as_rotvec()))
            if dpos > self._XR_JUMP_POS_M or drot > self._XR_JUMP_ROT_RAD:
                if rejects + 1 >= self._XR_JUMP_MAX_REJECTS:
                    # 持续偏离 = 真实搬位（手柄放下/挪动），不是单帧毛刺：
                    # 接受新位置，参考随后重建。
                    out[name] = pose
                    continue
                jumped.append(f"{name}({dpos:.2f}m/{np.degrees(drot):.0f}deg)")
                out[name] = ref  # hold last valid pose
            else:
                out[name] = pose
        self._xr_prev_poses = {k: v.copy() for k, v in out.items()}
        if jumped:
            self._xr_jump_rejects = rejects + 1
            print(f"[XR] rejected jump: {', '.join(jumped)}; holding previous pose")
            # 跳变后旧参考已不可信：清锚，grip 重锚逻辑（_track_side 的
            # ref_ee_pos is None 分支）会在下一拍用当前位姿重建参考。
            for st in self.track.values():
                st.clear()
        elif rejects > 0 or any(
            np.linalg.norm(out[n][:3] - prev.get(n, out[n])[:3]) > self._XR_JUMP_POS_M
            for n in out
        ):
            # 没有拒绝但上一轮处于拒绝状态 / 帧刚被接受为搬位：计数清零、
            # 清锚让 grip 用新位置重建参考（不打印——正常搬位不是事件）。
            if rejects > 0:
                self._xr_jump_rejects = 0
                for st in self.track.values():
                    st.clear()
        else:
            self._xr_jump_rejects = 0
        return out

    def get_grip_state(self, side: str | None = None) -> float:
        side = side or self.control_side
        if self.mock_xr or self.xr_client is None:
            left_grip, right_grip, _, _ = make_mock_grip_values(self.t)
            return left_grip if side == "left" else right_grip
        return get_key_value(self.xr_client, f"{side}_grip")

    def get_trigger_sample(self, side: str | None = None) -> tuple[float, bool]:
        side = side or self.control_side
        if self.mock_xr or self.xr_client is None:
            _, _, left_trigger, right_trigger = make_mock_grip_values(self.t)
            value = left_trigger if side == "left" else right_trigger
            self._trigger_samples[side] = (float(value), True, time.monotonic())
            return float(value), True
        try:
            value = float(self.xr_client.get_key_value_by_name(f"{side}_trigger"))
            valid = np.isfinite(value) and 0.0 <= value <= 1.0
        except Exception:
            value, valid = self._trigger_samples.get(side, (0.0, False, 0.0))[0], False
        if valid:
            self._trigger_samples[side] = (float(value), True, time.monotonic())
        else:
            value = self._trigger_samples.get(side, (0.0, False, 0.0))[0]
        return float(value), bool(valid)

    def get_trigger_state(self, side: str | None = None) -> float:
        return self.get_trigger_sample(side)[0]

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
        trigger, _ = self.get_trigger_sample(side)
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

    def _step_grippers(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self.last_gripper_update)
        self.last_gripper_update = now
        for side in self.control_sides:
            self._update_dex1_side(side, dt)
        self._send_dex1_all()

    def _track_side(self, side: str, controller_poses: dict) -> None:
        st = self.track[side]
        st.active = self.get_grip_state(side) > 0.9
        if not st.active:
            self._last_ee_intent[side] = {}
        controller_pose = controller_poses.get(f"{side}_controller")
        q = self._q_for_side(side)
        self._ensure_trajectory_state(side, q)

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
                    # Re-anchor the posture reference at the pose the arm is
                    # actually in when tracking starts. A stale reference (e.g.
                    # the dangling startup pose) otherwise pulls the solver one
                    # integrator step toward it on the very first solve,
                    # producing a single-frame action jump that never ran on
                    # the hardware.
                    solver = self._placo_for_side(side)
                    if self.use_placo and solver is not None:
                        solver.set_regularization_reference(q)
                    self._last_ee_intent[side] = {
                        "position": ee_pos.copy(),
                        "quaternion_xyzw": ee_quat.copy() if ee_quat is not None else None,
                        "frame": "robot_base",
                    }
                    self._ensure_trajectory_state(side, q)
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

                    self._last_ee_intent[side] = {
                        "position": target_pos.copy(),
                        "quaternion_xyzw": target_quat.copy() if target_quat is not None else None,
                        "frame": "robot_base",
                    }

                    self._solve(side, target_pos, target_quat)
        else:
            st.clear()
            self._last_ik_raw[side] = np.array([q[i] for i in self.arm_q_indices])
            self._last_ik_solver[side] = "stopping" if np.max(np.abs(self._trajectory_v_cmd[side])) > 1e-4 else "hold"
            stop_target = q.copy()
            for idx, value in zip(self.arm_q_indices, self._trajectory_q_cmd[side]):
                stop_target[idx] = value
            q_stopped = self._trajectory_step(side, q, stop_target)
            self._set_q_for_side(side, q_stopped)
            self._solver_seeds[side] = q_stopped.copy()

    def step(self, dt: float) -> np.ndarray:
        self._step_dt = float(dt)
        self.t += dt
        # Trigger/DDS control is independent of pose validity and reset state.
        self._step_grippers()

        if self._reset_blending:
            self._step_reset_blend()
            self._output_command()
            return self.q

        controller_poses, valid = self.read_controller_poses()
        if not valid:
            for side in self.control_sides:
                self._track_side(side, {})
            self._publish_ik_diagnostics({}, xr_valid=False)
            self.active = any(self.track[side].active for side in self.control_sides)
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
        self._publish_ik_diagnostics(controller_poses, xr_valid=True)
        self.active = any(self.track[s].active for s in self.control_sides)

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
        # The native XR service owns background threads. Closing it explicitly
        # before interpreter teardown avoids occasional shutdown segfaults.
        xr_client = self.xr_client
        self.xr_client = None
        if xr_client is not None:
            try:
                xr_client.close()
            except Exception as exc:
                if self.debug:
                    print(f"XR close warning: {exc}")

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
        self.rate_hz = float(rate)
        if self.dual_arm:
            print(f"Starting R1Arm dual-arm teleoperation (rate={rate} Hz)")
            print("Hold left/right grip to activate corresponding arm")
            print("Hold left/right trigger to close corresponding Dex1")
        else:
            print(f"Starting R1Arm teleoperation (rate={rate} Hz, side={self.control_side})")
            print(f"Hold {self.control_side} grip to activate arm tracking")
            print(f"Hold {self.control_side} trigger to close Dex1 gripper")
        if self.enable_a_button_reset:
            print(
                f"Press A button to move to home through q/v/a trajectory "
                f"({self.reset_blend_frames} nominal frames minimum)"
            )
        else:
            print("A-button home reset is disabled")
        print("Press Ctrl+C to stop")

        dt = 1.0 / rate
        try:
            self._on_start()
            previous_start = time.monotonic()
            while self.running:
                loop_start = time.monotonic()
                actual_dt = loop_start - previous_start
                previous_start = loop_start
                self.step(actual_dt if 0.001 <= actual_dt <= 0.1 else dt)
                elapsed = time.monotonic() - loop_start
                sleep_t = dt - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.running = False
            self._on_stop()
