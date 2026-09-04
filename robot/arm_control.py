"""Arm controller with simulation and real robot modes."""

from __future__ import annotations

import abc
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from robot.config import R1A7Config


class BaseArmController(abc.ABC):
    """Abstract base class for arm controllers."""

    arm_joint_count: int = 14  # 7 joints per arm
    arm_velocity_limit: float = 50.0

    @abc.abstractmethod
    def ctrl_dual_arm(self, q_target, tauff_target=None, gravity_feedforward_tau=None):
        """Set target joint positions for both arms."""
        pass

    @abc.abstractmethod
    def get_current_dual_arm_q(self) -> np.ndarray:
        """Get current dual arm joint positions."""
        pass

    @abc.abstractmethod
    def get_current_motor_q(self) -> np.ndarray:
        """Get current motor positions (31 motors for R1)."""
        pass

    def stop(self):
        """Stop the controller."""
        pass


class SimulationArmController(BaseArmController):
    """Stand-in controller for simulation without DDS connection."""

    def __init__(self, arm_joint_count: int = 14):
        self.arm_joint_count = arm_joint_count
        self.arm_velocity_limit = 50
        self.q = np.zeros(self.arm_joint_count, dtype=float)

    def ctrl_dual_arm(self, q_target, tauff_target=None, gravity_feedforward_tau=None):
        del tauff_target, gravity_feedforward_tau
        q_arr = np.asarray(q_target, dtype=float).reshape(-1)
        self.q[:] = q_arr[: self.arm_joint_count]

    def get_current_dual_arm_q(self) -> np.ndarray:
        return self.q.copy()

    def get_current_motor_q(self) -> np.ndarray:
        return self.q.copy()

    def stop(self):
        pass


class RealArmController(BaseArmController):
    """Real robot controller via DDS."""

    def __init__(self, config: "R1A7Config", interface: str = "eth0", enable_motion: bool = True):
        from robot.r1_a7_real import (
            REAL_JOINTS,
            R1A7LowCmdConfig,
            R1A7LowCmdTeleop,
        )

        self.REAL_JOINTS = REAL_JOINTS
        self.arm_joint_count = 14
        self.arm_velocity_limit = 50

        joint_indices = {name: idx for idx, (name, _, _, _) in enumerate(REAL_JOINTS)}
        dds_config = R1A7LowCmdConfig(
            interface=interface,
            enable_motion=enable_motion,
            max_joint_delta=0.04,
            control_dt=0.02,
        )
        self.real = R1A7LowCmdTeleop(dds_config, joint_indices)
        self.real.init_dds()
        self.real.wait_for_lowstate()
        self.real.start()

    def ctrl_dual_arm(self, q_target, tauff_target=None, gravity_feedforward_tau=None):
        del tauff_target, gravity_feedforward_tau
        q_arr = np.asarray(q_target, dtype=float).reshape(-1)
        if q_arr.size < self.arm_joint_count:
            raise ValueError(f"Need at least {self.arm_joint_count} joints, got {q_arr.size}")
        target = self.real.target_q.copy()
        target[: self.arm_joint_count] = q_arr[: self.arm_joint_count]
        with self.real.lock:
            self.real.target_q[:] = target

    def get_current_dual_arm_q(self) -> np.ndarray:
        with self.real.lock:
            msg = self.real.low_state
            if msg is None:
                return self.real.command_q[: self.arm_joint_count].copy()
            return np.array(
                [
                    float(msg.motor_state[idl_i].q)
                    for _, idl_i, _, _ in self.REAL_JOINTS[: self.arm_joint_count]
                ],
                dtype=float,
            )

    def get_current_motor_q(self) -> np.ndarray:
        with self.real.lock:
            msg = self.real.low_state
            if msg is None:
                return np.zeros(31, dtype=float)
            q = np.zeros(31, dtype=float)
            for _, idl_i, _, _ in self.REAL_JOINTS:
                q[idl_i] = float(msg.motor_state[idl_i].q)
            return q

    def stop(self):
        self.real.stop()


def create_arm_controller(
    simulation: bool = True,
    config: "R1A7Config | None" = None,
    interface: str = "eth0",
    enable_motion: bool = True,
) -> BaseArmController:
    """Factory function to create appropriate arm controller."""
    if simulation:
        return SimulationArmController()
    else:
        if config is None:
            from robot.config import R1A7Config
            config = R1A7Config()
        return RealArmController(config, interface, enable_motion)
