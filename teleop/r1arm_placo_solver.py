"""单臂 Placo QP IK（复用 A7 同款 frame task + manipulability）。"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.r1arm_config import ARM_JOINTS, DEX1_JOINTS


def joint_delta_bounds(
    previous_velocity: np.ndarray,
    velocity_limits: np.ndarray,
    acceleration_limit: float,
    dt: float,
    previous_acceleration: np.ndarray | None = None,
    jerk_limit: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return feasible Δq bounds for velocity, acceleration, and optional jerk."""
    dt = float(dt)
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("IK dt must be finite and positive")
    previous_velocity = np.asarray(previous_velocity, dtype=float)
    velocity_limits = np.asarray(velocity_limits, dtype=float)
    if previous_velocity.shape != velocity_limits.shape:
        raise ValueError("previous velocity and velocity limits must have equal shape")
    if not np.all(np.isfinite(previous_velocity)) or not np.all(velocity_limits > 0.0):
        raise ValueError("IK velocity state/limits must be finite and positive")
    acceleration_step = abs(float(acceleration_limit)) * dt
    velocity_lower = np.maximum(-velocity_limits, previous_velocity - acceleration_step)
    velocity_upper = np.minimum(velocity_limits, previous_velocity + acceleration_step)
    if jerk_limit is not None and jerk_limit > 0.0:
        previous_acceleration = np.asarray(previous_acceleration, dtype=float)
        if previous_acceleration.shape != previous_velocity.shape or not np.all(
            np.isfinite(previous_acceleration)
        ):
            raise ValueError("jerk-constrained IK requires finite previous acceleration")
        jerk_step = float(jerk_limit) * dt
        velocity_lower = np.maximum(
            velocity_lower, previous_velocity + dt * (previous_acceleration - jerk_step)
        )
        velocity_upper = np.minimum(
            velocity_upper, previous_velocity + dt * (previous_acceleration + jerk_step)
        )
    if np.any(velocity_lower > velocity_upper):
        raise ValueError("previous command velocity is outside the feasible IK limits")
    return dt * velocity_lower, dt * velocity_upper


class PlacoArmIKSolver:
    """单末端 frame task 的 Placo QP 封装；夹爪关节 hold 住。"""

    def __init__(
        self,
        urdf_path,
        q_init: np.ndarray,
        ee_frame: str,
        joint_indices: dict[str, int],
        arm_joints: list[str] | None = None,
        hold_joints: list[str] | None = None,
        arm_task_weight: float = 1.0,
        control_rate_hz: float = 50.0,
        joint_velocity_limits: dict[str, float] | None = None,
        default_velocity_limit: float = 1.0,
        default_acceleration_limit: float | None = None,
        default_jerk_limit: float | None = None,
    ):
        import placo

        self.ee_frame = ee_frame
        self.joint_indices = joint_indices
        self.arm_joints = list(arm_joints or ARM_JOINTS)
        self.hold_joints = list(hold_joints if hold_joints is not None else DEX1_JOINTS)
        self.arm_task_weight = arm_task_weight
        # Placo uses dt to interpret solved joint increments as velocity and
        # enforce configured velocity limits; without those limits, changing dt
        # does not alter this pose-IK solution.
        self.control_rate_hz = float(control_rate_hz)
        self.solver_dt = 1.0 / max(self.control_rate_hz, 1.0)

        self.placo_robot = placo.RobotWrapper(str(urdf_path))
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = self.solver_dt
        self.solver.mask_fbase(True)
        self.solver.enable_joint_limits(True)

        # QP 内关节速度上限（P2 源头驯服）：solver 的解本身限速，原始
        # 意图的速度/加速度显著下降（实测无限制时 solver 原始 a_max 可达
        # 219 rad/s²）——action 流更"原生"，输出端整形器退为兜底。
        # 默认 1.0 rad/s 对齐输出端整形器上限；逐关节经
        # joint_velocity_limits 覆盖，None = 不启用。
        if default_velocity_limit is not None and default_velocity_limit > 0:
            per_joint = joint_velocity_limits or {}
            self.velocity_limits = np.array([
                float(per_joint.get(name, default_velocity_limit))
                for name in self.arm_joints
            ])
            self.velocity_limits = np.where(
                self.velocity_limits > 0.0, self.velocity_limits, float(default_velocity_limit)
            )
            for name, limit in zip(self.arm_joints, self.velocity_limits):
                self.placo_robot.set_velocity_limit(name, float(limit))
            self.solver.enable_velocity_limits(True)
        else:
            self.velocity_limits = np.full(len(self.arm_joints), np.inf)

        self.acceleration_limit = (
            float(default_acceleration_limit)
            if default_acceleration_limit is not None and default_acceleration_limit > 0
            else None
        )
        self.jerk_limit = (
            float(default_jerk_limit)
            if default_jerk_limit is not None and default_jerk_limit > 0
            else None
        )
        self.acceleration_constraint = None
        if self.acceleration_limit is not None or self.jerk_limit is not None:
            A = np.zeros((2 * len(self.arm_joints), len(self.placo_robot.state.q)))
            for i, name in enumerate(self.arm_joints):
                joint_offset = self.placo_robot.get_joint_offset(name)
                A[2 * i, joint_offset] = 1.0
                A[2 * i + 1, joint_offset] = -1.0
            self.acceleration_constraint = self.solver.add_joint_space_half_spaces_constraint(
                A, np.zeros(2 * len(self.arm_joints))
            )
            self.acceleration_constraint.configure("joint_motion_bounds", "hard")

        self.placo_robot.state.q[7:] = np.asarray(q_init, dtype=float).copy()
        self.placo_robot.update_kinematics()

        T = self.placo_robot.get_T_world_frame(ee_frame).copy()
        self.effector_task = self.solver.add_frame_task(ee_frame, T)
        self.effector_task.configure("ee", "soft", self.arm_task_weight)

        m_task = self.solver.add_manipulability_task(ee_frame, "both", 1.0)
        m_task.configure("ee_manip", "soft", 1e-2)

        self.hold_task = self.solver.add_joints_task()
        self.hold_task.configure("hold", "soft", 1.0)

        self.regularization_task = self.solver.add_joints_task()
        default_arm_joints = {}
        for name in self.arm_joints:
            idx = joint_indices.get(name)
            if idx is not None:
                default_arm_joints[name] = float(q_init[idx])
        self._default_arm_joints = default_arm_joints
        self.regularization_task.set_joints(default_arm_joints)
        self.regularization_task.configure("joints_regularization", "soft", 1e-2)

    def set_regularization_reference(self, q: np.ndarray) -> None:
        """Use the measured startup pose as the posture preference for real teleop."""
        reference = {}
        for name in self.arm_joints:
            idx = self.joint_indices.get(name)
            if idx is not None:
                reference[name] = float(q[idx])
        self._default_arm_joints = reference
        self.regularization_task.set_joints(reference)

    def delta_bounds(
        self, dt: float, previous_velocity: np.ndarray,
        previous_acceleration: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if self.acceleration_limit is None:
            raise RuntimeError("acceleration limits are not enabled for this Placo solver")
        return joint_delta_bounds(
            previous_velocity, self.velocity_limits,
            self.acceleration_limit or np.inf, dt,
            previous_acceleration=previous_acceleration, jerk_limit=self.jerk_limit,
        )

    def solve(
        self,
        q: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray | None,
        *,
        dt: float | None = None,
        previous_velocity: np.ndarray | None = None,
        previous_acceleration: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """求解单臂 IK，成功返回 q（URDF 序），失败返回 None。"""
        if dt is not None:
            self.solver.dt = float(dt)
        if self.acceleration_constraint is not None:
            if previous_velocity is None or dt is None:
                raise ValueError("constrained IK requires dt and previous_velocity")
            lower, upper = self.delta_bounds(dt, previous_velocity, previous_acceleration)
            q_reference = np.asarray(q, dtype=float)
            bounds = np.empty(2 * len(self.arm_joints))
            arm_q = np.array([q_reference[self.joint_indices[name]] for name in self.arm_joints])
            bounds[0::2] = arm_q + upper
            bounds[1::2] = -(arm_q + lower)
            self.acceleration_constraint.b = bounds
        self.placo_robot.state.q[7:] = np.asarray(q, dtype=float).copy()
        self.placo_robot.update_kinematics()

        T = np.eye(4)
        if target_quat is not None:
            T[:3, :3] = R.from_quat(target_quat).as_matrix()
        else:
            T[:3, :3] = self.placo_robot.get_T_world_frame(self.ee_frame)[:3, :3]
        T[:3, 3] = target_pos
        self.effector_task.T_world_frame = T

        hold = {}
        for name in self.hold_joints:
            idx = self.joint_indices.get(name)
            if idx is not None:
                hold[name] = float(q[idx])
        self.hold_task.set_joints(hold)
        self.regularization_task.set_joints(self._default_arm_joints)

        try:
            self.solver.solve(True)
        except RuntimeError as exc:
            print(f"Placo IK failed: {exc}")
            return None

        return self.placo_robot.state.q[7:].copy()
