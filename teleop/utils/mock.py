"""Mock XR and body-pose generators for testing without PICO hardware."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.config import BODY

__all__ = [
    "BODY",
    "pose_is_valid",
    "xr_pose_is_valid",
    "normalize_xr_pose",
    "make_mock_poses",
    "make_mock_controller_poses",
    "make_mock_grip_values",
]


def pose_is_valid(poses: np.ndarray, key: str) -> bool:
    """Check if a body pose landmark is valid."""
    index = BODY[key]
    return index < poses.shape[0] and poses.shape[1] >= 7 and np.isfinite(poses[index, :7]).all()


def xr_pose_is_valid(pose: np.ndarray | None) -> bool:
    """Check if an XR pose is valid."""
    if pose is None:
        return False
    arr = np.asarray(pose, dtype=float).reshape(-1)
    if arr.size < 7 or not np.isfinite(arr[:7]).all():
        return False
    return float(np.linalg.norm(arr[3:7])) > 1e-5


def normalize_xr_pose(pose: np.ndarray) -> np.ndarray:
    """Normalize XR pose quaternion."""
    arr = np.asarray(pose, dtype=float).reshape(-1)[:7].copy()
    quat_norm = float(np.linalg.norm(arr[3:7]))
    if quat_norm > 1e-8:
        arr[3:7] /= quat_norm
    return arr


# ----------------------------------------------------------------------
# Mock body poses
# ----------------------------------------------------------------------
def make_mock_poses(t: float) -> np.ndarray:
    """Generate mock body poses for testing without PICO."""
    poses = np.zeros((24, 7), dtype=float)
    poses[:, 6] = 1.0

    poses[BODY["pelvis"], :3] = [0.0, 0.0, 1.0]
    poses[BODY["neck"], :3] = [0.0, 0.0, 1.55]
    poses[BODY["head"], :3] = [0.0, 0.0, 1.75]
    poses[BODY["left_shoulder"], :3] = [-0.22, 0.0, 1.48]
    poses[BODY["right_shoulder"], :3] = [0.22, 0.0, 1.48]

    lift = 0.35 + 0.25 * math.sin(t)
    spread = 0.10 * math.sin(0.7 * t)
    poses[BODY["left_elbow"], :3] = [-0.35 - spread, 0.05 + lift, 1.20]
    poses[BODY["left_wrist"], :3] = [-0.42 - spread, 0.20 + lift, 1.02]
    poses[BODY["left_hand"], :3] = poses[BODY["left_wrist"], :3]
    poses[BODY["right_elbow"], :3] = [0.35 + spread, 0.05 + lift, 1.20]
    poses[BODY["right_wrist"], :3] = [0.42 + spread, 0.20 + lift, 1.02]
    poses[BODY["right_hand"], :3] = poses[BODY["right_wrist"], :3]

    yaw = 0.25 * math.sin(0.5 * t)
    q_head = R.from_euler("z", yaw).as_quat()
    poses[BODY["head"], 3:7] = q_head
    poses[BODY["left_hand"], 3:7] = R.from_euler("xyz", [0.3 * math.sin(t), 0.0, 0.0]).as_quat()
    poses[BODY["right_hand"], 3:7] = R.from_euler("xyz", [-0.3 * math.sin(t), 0.0, 0.0]).as_quat()
    return poses


# ----------------------------------------------------------------------
# Mock XR controller poses
# ----------------------------------------------------------------------
def make_mock_controller_poses(t: float) -> dict[str, np.ndarray]:
    """Generate mock controller poses for testing without PICO.

    These poses are in PICO headset/controller frame (raw coords).
    The teleop code will convert them to world frame:
    world_x = -h_z, world_y = -h_x, world_z = h_y
    """
    poses: dict[str, np.ndarray] = {}

    # Headset at eye level
    head_yaw = 0.15 * math.sin(0.35 * t)
    poses["headset"] = np.array(
        [0.0, 0.4, 1.5, *R.from_euler("z", head_yaw).as_quat()],
        dtype=float,
    )

    # Controller movements (in PICO frame)
    # PICO frame: x=left/right, y=forward/back, z=up/down
    lift = 0.18 * math.sin(t)
    reach = 0.12 * math.sin(0.7 * t)

    # Left controller position (PICO frame)
    # world_x = -h_z, so h_z = -world_x
    # world_y = -h_x, so h_x = -world_y
    # world_z = h_y
    poses["left_controller"] = np.array(
        [0.15 + reach, 0.15 + lift, 0.0, *R.from_euler("xyz", [0.3 * math.sin(t), 0.0, 0.15]).as_quat()],
        dtype=float,
    )

    # Right controller position (PICO frame)
    poses["right_controller"] = np.array(
        [-0.15 - reach, 0.15 - lift, 0.0, *R.from_euler("xyz", [-0.3 * math.sin(t), 0.0, -0.15]).as_quat()],
        dtype=float,
    )

    return poses


# ----------------------------------------------------------------------
# Mock grip / trigger values
# ----------------------------------------------------------------------
def make_mock_grip_values(t: float) -> tuple[float, float, float, float]:
    """Generate mock grip/trigger values for testing without PICO.

    Returns (left_grip, right_grip, left_trigger, right_trigger) matching
    the PICO button/key semantics used by r1a7_controller_base.py:
    - left_grip/right_grip: binary (>0.5 = arm tracking active)
    - left_trigger/right_trigger: continuous 0..1 (gripper open/close)

    Mock cycle:
    - phase 0-3s: inactive (grip=0)
    - phase 3-8s: left active (left_grip=1, right_grip=0)
    - phase 8-13s: right active (left_grip=0, right_grip=1)
    - phase 13-18s: both active (grip=1)
    - then repeat

    Trigger is a slow sine wave (0.5Hz) so gripper opens and closes continuously.
    """
    cycle_t = t % 18.0

    if cycle_t < 3.0:
        # Inactive
        left_grip = 0.0
        right_grip = 0.0
    elif cycle_t < 8.0:
        # Left active
        left_grip = 1.0
        right_grip = 0.0
    elif cycle_t < 13.0:
        # Right active
        left_grip = 0.0
        right_grip = 1.0
    else:
        # Both active
        left_grip = 1.0
        right_grip = 1.0

    # Slow sine wave for trigger (gripper opens/closes)
    trigger = max(0.0, math.sin(0.5 * t))

    return left_grip, right_grip, trigger, trigger
