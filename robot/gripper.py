"""Dex1 gripper control for R1-A7."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


GRIPPER_JOINTS = {
    "left": ["left_dex1_finger_joint_1", "left_dex1_finger_joint_2"],
    "right": ["right_dex1_finger_joint_1", "right_dex1_finger_joint_2"],
}


@dataclass
class GripperCommand:
    """Gripper command for left and right hands."""
    left: float
    right: float


def all_gripper_joint_names() -> list[str]:
    """Get all gripper joint names."""
    return GRIPPER_JOINTS["left"] + GRIPPER_JOINTS["right"]


def side_gripper_joint_names(side: str) -> list[str]:
    """Get gripper joint names for a given side."""
    return list(GRIPPER_JOINTS[side])


def gripper_q_indices(joint_indices: dict[str, int], side: str) -> list[int]:
    """Get q indices for gripper joints."""
    return [joint_indices[name] for name in GRIPPER_JOINTS[side] if name in joint_indices]


def has_grippers(joint_indices: dict[str, int]) -> bool:
    """Check if the robot has grippers."""
    return any(name in joint_indices for name in all_gripper_joint_names())


def gripper_limits(
    model: pin.Model,
    joint_indices: dict[str, int],
    side: str,
    open_value: float | None = None,
    close_value: float | None = None,
) -> tuple[float, float]:
    """Get gripper open and close positions."""
    indices = gripper_q_indices(joint_indices, side)
    if not indices:
        return 0.0, 0.0
    lower = max(float(model.lowerPositionLimit[idx]) for idx in indices)
    upper = min(float(model.upperPositionLimit[idx]) for idx in indices)
    # Dex1 opens at upper limit and closes at lower limit
    open_q = upper if open_value is None else float(np.clip(open_value, lower, upper))
    close_q = lower if close_value is None else float(np.clip(close_value, lower, upper))
    return open_q, close_q


def set_gripper_side(q: np.ndarray, joint_indices: dict[str, int], side: str, value: float) -> None:
    """Set gripper position for a given side."""
    for q_idx in gripper_q_indices(joint_indices, side):
        q[q_idx] = value


def reset_grippers(
    q: np.ndarray,
    model: pin.Model,
    joint_indices: dict[str, int],
    open_value: float | None = None,
    close_value: float | None = None,
) -> GripperCommand:
    """Reset both grippers to open position."""
    left_open, _ = gripper_limits(model, joint_indices, "left", open_value, close_value)
    right_open, _ = gripper_limits(model, joint_indices, "right", open_value, close_value)
    set_gripper_side(q, joint_indices, "left", left_open)
    set_gripper_side(q, joint_indices, "right", right_open)
    return GripperCommand(left=left_open, right=right_open)


def update_gripper_command(
    current: float,
    trigger_value: float,
    open_q: float,
    close_q: float,
    speed: float,
    dt: float,
) -> float:
    """Update gripper position based on trigger value with speed limiting."""
    trigger_value = float(np.clip(trigger_value, 0.0, 1.0))
    desired = open_q + (close_q - open_q) * trigger_value
    max_delta = max(0.0, float(speed) * max(0.0, float(dt)))
    if max_delta <= 0.0:
        return desired
    return float(current + np.clip(desired - current, -max_delta, max_delta))


def apply_gripper_command(q: np.ndarray, joint_indices: dict[str, int], command: GripperCommand) -> None:
    """Apply gripper command to joint configuration."""
    set_gripper_side(q, joint_indices, "left", command.left)
    set_gripper_side(q, joint_indices, "right", command.right)


class GripperController:
    """High-level gripper controller with trigger-based control."""

    def __init__(
        self,
        model: pin.Model,
        joint_indices: dict[str, int],
        open_value: float | None = None,
        close_value: float | None = None,
        speed: float = 0.08,
    ):
        self.model = model
        self.joint_indices = joint_indices
        self.speed = speed

        self.left_open, self.left_close = gripper_limits(
            model, joint_indices, "left", open_value, close_value
        )
        self.right_open, self.right_close = gripper_limits(
            model, joint_indices, "right", open_value, close_value
        )

        self.left_current = self.left_open
        self.right_current = self.right_open

    def update(self, left_trigger: float, right_trigger: float, dt: float) -> GripperCommand:
        """Update gripper positions based on trigger values."""
        self.left_current = update_gripper_command(
            self.left_current, left_trigger, self.left_open, self.left_close, self.speed, dt
        )
        self.right_current = update_gripper_command(
            self.right_current, right_trigger, self.right_open, self.right_close, self.speed, dt
        )
        return GripperCommand(left=self.left_current, right=self.right_current)

    def reset(self):
        """Reset grippers to open position."""
        self.left_current = self.left_open
        self.right_current = self.right_open
