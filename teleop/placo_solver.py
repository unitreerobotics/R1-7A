"""Placo QP inverse-kinematics solver wrapper for R1-A7 (same as teleop_r1a7.py)."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.config import HEAD_JOINTS, LEFT_ARM_JOINTS, LEFT_DEX1_JOINTS, RIGHT_ARM_JOINTS, RIGHT_DEX1_JOINTS


class PlacoIKSolver:
    """Thin wrapper around placo.KinematicsSolver for the R1-A7 dual-arm task.

    Holds the robot wrapper, QP solver, and per-arm frame tasks. ``solve`` sets
    the effector targets for active arms, holds the waist/head/gripper joints at
    their commanded values, and returns the solved joint configuration.
    """

    def __init__(
        self,
        urdf_path,
        q_init: np.ndarray,
        ee_frames: dict[str, str],
        joint_indices: dict[str, int],
        waist_joint: str = "waist_yaw_joint",
        hold_joints=tuple(HEAD_JOINTS + LEFT_DEX1_JOINTS + RIGHT_DEX1_JOINTS),
        arm_task_weight: float = 1.0,
    ):
        import placo

        self.ee_frames = ee_frames
        self.joint_indices = joint_indices
        self.waist_joint = waist_joint
        self.hold_joints = hold_joints
        self.arm_task_weight = arm_task_weight

        self.placo_robot = placo.RobotWrapper(str(urdf_path))
        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = 0.02
        self.solver.mask_fbase(True)
        self.solver.enable_joint_limits(True)
        self.placo_robot.state.q[7:] = np.asarray(q_init, dtype=float).copy()
        self.placo_robot.update_kinematics()

        self.effector_task = {}
        for side, link in self.ee_frames.items():
            T = self.placo_robot.get_T_world_frame(link).copy()
            task = self.solver.add_frame_task(link, T)
            task.configure(side, "soft", self.arm_task_weight)
            self.effector_task[side] = task

            # Manipulability task keeps the arm away from singularities
            # (same as teleop_sim_placo.py and teleop_r1a7.py).
            m_task = self.solver.add_manipulability_task(link, "both", 1.0)
            m_task.configure(f"{side}_manip", "soft", 1e-2)

        # Hold waist strongly so the arm task cannot borrow torso motion.
        self.waist_hold_task = self.solver.add_joints_task()
        self.waist_hold_task.configure("waist_hold", "soft", 10.0)

        # Hold head and gripper joints at their commanded values.
        self.hold_task = self.solver.add_joints_task()
        self.hold_task.configure("hold", "soft", 1.0)

        # Joints regularization: pull arm joints toward their initial values (same
        # strategy as teleop_r1a7.py's "joints_regularization" task, weight=1e-2).
        self.regularization_task = self.solver.add_joints_task()
        default_arm_joints = {}
        for name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
            idx = joint_indices.get(name)
            if idx is not None:
                default_arm_joints[name] = float(q_init[idx])
        self._default_arm_joints = default_arm_joints
        self.regularization_task.set_joints(default_arm_joints)
        self.regularization_task.configure("joints_regularization", "soft", 1e-2)

    def solve(self, q: np.ndarray, targets: dict) -> np.ndarray | None:
        """Solve IK for both arms.

        Args:
            q: Current joint configuration (URDF joint order).
            targets: Map side -> (target_pos, target_quat) for active arms. Arms
                absent from ``targets`` are held at their current pose.

        Returns:
            Solved joint configuration (same URDF order as ``q``), or ``None`` if
            the QP solve failed (caller keeps the previous configuration).
        """
        self.placo_robot.state.q[7:] = np.asarray(q, dtype=float).copy()
        self.placo_robot.update_kinematics()

        for side, link in self.ee_frames.items():
            tgt = targets.get(side)
            if tgt is None:
                self.effector_task[side].T_world_frame = self.placo_robot.get_T_world_frame(link).copy()
                continue
            pos, quat = tgt
            T = np.eye(4)
            if quat is not None:
                T[:3, :3] = R.from_quat(quat).as_matrix()
            else:
                T[:3, :3] = self.placo_robot.get_T_world_frame(link)[:3, :3]
            T[:3, 3] = pos
            self.effector_task[side].T_world_frame = T

        waist_idx = self.joint_indices.get(self.waist_joint)
        if waist_idx is not None:
            self.waist_hold_task.set_joints({self.waist_joint: float(q[waist_idx])})

        hold = {}
        for name in self.hold_joints:
            idx = self.joint_indices.get(name)
            if idx is not None:
                hold[name] = float(q[idx])
        self.hold_task.set_joints(hold)

        # Keep arm joints anchored to their initial values (constant reference).
        self.regularization_task.set_joints(self._default_arm_joints)

        try:
            self.solver.solve(True)
        except RuntimeError as exc:
            print(f"Placo IK failed: {exc}")
            return None

        # Placo q[7:] uses the same URDF joint order as q, so indices match.
        return self.placo_robot.state.q[7:].copy()
