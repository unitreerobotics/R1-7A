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

        # Baseline pose IK: dynamic limits are applied by the joint shaper.
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

    def solve(
        self,
        q: np.ndarray,
        target_pos: np.ndarray,
        target_quat: np.ndarray | None,
        *,
        dt: float | None = None,
    ) -> np.ndarray | None:
        """求解单臂 IK，成功返回 q（URDF 序），失败返回 None。"""
        if dt is not None:
            self.solver.dt = float(dt)
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
