"""Utility modules for teleoperation."""
from teleop.utils.geometry import R_HEADSET_TO_WORLD, quaternion_to_world, quaternion_to_euler
from teleop.utils.mock import (
    BODY,
    pose_is_valid,
    xr_pose_is_valid,
    normalize_xr_pose,
    make_mock_poses,
    make_mock_controller_poses,
    make_mock_grip_values,
)
