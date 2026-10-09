"""单臂 Placo QP IK（复用 A7 同款 frame task + manipulability）。"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.r1arm_config import ARM_JOINTS, DEX1_JOINTS


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
    ):
        import placo

        self.ee_frame = ee_frame
        self.joint_indices = joint_indices
        self.arm_joints = list(arm_joints or ARM_JOINTS)
        self.hold_joints = list(hold_joints if hold_joints is not None else DEX1_JOINTS)
        self.arm_task_weight = arm_task_weight
        # dt 必须等于实际控制周期：Placo 把 frame task 误差当速度积分，
        # dt 与实际周期不一致时等效增益错误（dt 偏大 → 每步增量偏小 →
        # 跟踪变肉、滞后增大）。
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
            for name in self.arm_joints:
                limit = float(per_joint.get(name, default_velocity_limit))
                if limit > 0:
                    self.placo_robot.set_velocity_limit(name, limit)
            self.solver.enable_velocity_limits(True)

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

    def solve(self, q: np.ndarray, target_pos: np.ndarray, target_quat: np.ndarray | None) -> np.ndarray | None:
        """求解单臂 IK，成功返回 q（URDF 序），失败返回 None。"""
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
