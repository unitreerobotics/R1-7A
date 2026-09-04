"""Real-robot teleoperation controller for R1-A7.

Shares the tracking + IK core with ``SimTeleopController`` (see
``teleop.r1a7_controller_base``) but sends the solved configuration to the real
robot instead of meshcat:
- arms -> ``RealArmController.ctrl_dual_arm`` -> DDS ``rt/lowcmd``
- gripper -> ``Dex1.set_gripper_ratios`` -> DDS ``rt/dex1/*/cmd``

The waist is solved for IK/viz only and is NOT sent (``REAL_JOINTS`` has no
``waist_yaw_joint``; the appended value is dropped by ``ctrl_dual_arm[:14]``).
"""

from __future__ import annotations

import numpy as np

from teleop.r1a7_controller_base import R1A7TeleopControllerBase
from robot.arm_control import create_arm_controller
from robot.config import (
    DEX1_SIM_CLOSE,
    DEX1_SIM_OPEN,
    LEFT_DEX1_JOINTS,
    RIGHT_DEX1_JOINTS,
    R1A7Config,
)


class RealTeleopController(R1A7TeleopControllerBase):
    """Teleoperation controller that drives the real R1-A7 via DDS."""

    def __init__(
        self,
        config: R1A7Config,
        interface: str = "eth0",
        motion_mode: bool = False,
        enable_real_hand: bool = False,
        mock_xr: bool = False,
        visualize: bool = False,
        debug: bool = False,
        scale_factor: float = 1.0,
    ):
        super().__init__(
            config,
            visualize=visualize,
            mock_xr=mock_xr,
            debug=debug,
            scale_factor=scale_factor,
        )

        self.motion_mode = bool(motion_mode)
        self.interface = interface

        # Arm controller: real DDS bridge in motion mode, sim stand-in otherwise
        # (lets the whole pipeline run without hardware for a dry-run).
        self.arm_ctrl = create_arm_controller(
            simulation=not self.motion_mode,
            config=config,
            interface=interface,
            enable_motion=True,
        )

        # Dex1 gripper (real hand). Only touched when explicitly requested.
        self.hand_ctrl = None
        if enable_real_hand:
            from robot.eef.dex1 import Dex1

            self.hand_ctrl = Dex1()
            print("Dex1 real hand controller enabled")

    def _output_command(self):
        # Arms in REAL_JOINTS order (left 7, right 7) + a trailing waist that
        # ctrl_dual_arm drops. Waist is solved for IK/viz only.
        arms = np.hstack([self.q[self.left_arm_q_indices], self.q[self.right_arm_q_indices]])
        waist = float(self.q[self.joint_indices["waist_yaw_joint"]])
        self.arm_ctrl.ctrl_dual_arm(np.hstack([arms, waist]))

        if self.hand_ctrl is not None:
            left_sim = float(self.q[self.joint_indices[LEFT_DEX1_JOINTS[0]]])
            right_sim = float(self.q[self.joint_indices[RIGHT_DEX1_JOINTS[0]]])
            self.hand_ctrl.set_gripper_ratios(
                self._sim_to_closed_ratio(left_sim),
                self._sim_to_closed_ratio(right_sim),
            )

    @staticmethod
    def _sim_to_closed_ratio(sim_value: float) -> float:
        """Map a sim-space Dex1 value to a 0..1 closed ratio (0=open, 1=closed)."""
        denom = DEX1_SIM_OPEN - DEX1_SIM_CLOSE
        ratio = 0.0 if abs(denom) < 1e-9 else (DEX1_SIM_OPEN - sim_value) / denom
        return float(np.clip(ratio, 0.0, 1.0))

    def _on_stop(self):
        if self.hand_ctrl is not None:
            self.hand_ctrl.stop()
        if self.arm_ctrl is not None:
            self.arm_ctrl.stop()
