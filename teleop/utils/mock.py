"""Mock PICO controller input and pose validation for R1Arm development."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as R


def xr_pose_is_valid(pose: np.ndarray | None) -> bool:
    """Return whether an XR position/quaternion pose is finite and usable."""
    if pose is None:
        return False
    arr = np.asarray(pose, dtype=float).reshape(-1)
    return arr.size >= 7 and bool(np.isfinite(arr[:7]).all()) and float(np.linalg.norm(arr[3:7])) > 1e-5


def normalize_xr_pose(pose: np.ndarray) -> np.ndarray:
    """Normalize an XR pose quaternion."""
    arr = np.asarray(pose, dtype=float).reshape(-1)[:7].copy()
    quat_norm = float(np.linalg.norm(arr[3:7]))
    if quat_norm > 1e-8:
        arr[3:7] /= quat_norm
    return arr


def make_mock_controller_poses(t: float) -> dict[str, np.ndarray]:
    """Generate PICO-frame poses for local simulation without a headset."""
    head_yaw = 0.15 * math.sin(0.35 * t)
    lift = 0.18 * math.sin(t)
    reach = 0.12 * math.sin(0.7 * t)
    return {
        "headset": np.array([0.0, 0.4, 1.5, *R.from_euler("z", head_yaw).as_quat()]),
        "left_controller": np.array([
            0.15 + reach, 0.15 + lift, 0.0,
            *R.from_euler("xyz", [0.3 * math.sin(t), 0.0, 0.15]).as_quat(),
        ]),
        "right_controller": np.array([
            -0.15 - reach, 0.15 - lift, 0.0,
            *R.from_euler("xyz", [-0.3 * math.sin(t), 0.0, -0.15]).as_quat(),
        ]),
    }


def make_mock_grip_values(t: float) -> tuple[float, float, float, float]:
    """Generate grip and trigger values for simulation without a headset."""
    phase = t % 18.0
    left_grip = float(3.0 <= phase < 8.0 or phase >= 13.0)
    right_grip = float(8.0 <= phase < 13.0 or phase >= 13.0)
    trigger = max(0.0, math.sin(0.5 * t))
    return left_grip, right_grip, trigger, trigger
