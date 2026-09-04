"""Headset/controller frame transforms and quaternion utilities."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as R

__all__ = [
    "R_HEADSET_TO_WORLD",
    "quaternion_to_world",
    "quaternion_to_euler",
]

# PICO/headset frame -> robot world frame (same matrix as
# xrobotoolkit_teleop.utils.geometry.R_HEADSET_TO_WORLD).
R_HEADSET_TO_WORLD = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def quaternion_to_world(quat_xyzw) -> R:
    """Rotate a controller quaternion from the PICO/headset frame to world frame.

    Mirrors BaseTeleopController._process_xr_pose: the PICO-frame orientation is
    conjugated by R_HEADSET_TO_WORLD so position and orientation share one frame.
    Without this, roll/pitch/yaw are scrambled while xyz still looks correct.
    """
    # 创建 PICO 坐标系下的旋转矩阵 r_pico
    r_pico = R.from_quat(np.asarray(quat_xyzw, dtype=float))
    # 将 PICO 坐标系下的旋转矩阵转换到世界坐标系下
    r_world = R_HEADSET_TO_WORLD @ r_pico.as_matrix() @ R_HEADSET_TO_WORLD.T
    return R.from_matrix(r_world)


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])
