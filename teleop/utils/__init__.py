"""Shared R1Arm pose, geometry, and mock-input helpers."""

from teleop.utils.geometry import R_HEADSET_TO_WORLD, quaternion_to_world, quaternion_to_euler
from teleop.utils.mock import (
    make_mock_controller_poses,
    make_mock_grip_values,
    normalize_xr_pose,
    xr_pose_is_valid,
)
