"""XRoboToolkit/PICO XR interface for R1-A7 teleoperation.

Uses ``teleop.xr_client.XrClient`` — a thin wrapper over the native
``xrobotoolkit_sdk`` pybind module.  Zero dependency on xrobotoolkit_teleop.
"""

from __future__ import annotations

import numpy as np


def make_xr_client():
    """Create an XR client for PICO device.

    ``XrClient.__init__`` handles native-library preloading and
    ``xrobotoolkit_sdk`` import internally.
    """
    from teleop.xr_client import XrClient

    return XrClient()


def get_joystick_state(xr_client, controller: str) -> np.ndarray:
    """Get joystick state for a controller."""
    try:
        axis = xr_client.get_joystick_state(controller)
    except Exception:
        return np.zeros(2)
    axis = np.asarray(axis, dtype=float).reshape(-1)
    if axis.size < 2 or not np.isfinite(axis[:2]).all():
        return np.zeros(2)
    return axis[:2]


def get_key_value(xr_client, name: str) -> float:
    """Get a named key value (e.g., trigger)."""
    try:
        value = xr_client.get_key_value_by_name(name)
    except Exception:
        return 0.0
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not np.isfinite(value):
        return 0.0
    return float(np.clip(value, 0.0, 1.0))


def read_controller_poses(xr_client) -> dict[str, np.ndarray] | None:
    """Read headset and controller poses."""
    from teleop.utils.mock import normalize_xr_pose, xr_pose_is_valid

    poses: dict[str, np.ndarray] = {}
    for name in ["headset", "left_controller", "right_controller"]:
        try:
            pose = xr_client.get_pose_by_name(name)
        except Exception:
            return None
        if not xr_pose_is_valid(pose):
            return None
        poses[name] = normalize_xr_pose(pose)
    return poses


def button_edge(xr_client, name: str, last: dict[str, bool]) -> bool:
    """Detect button rising edge."""
    try:
        pressed = bool(xr_client.get_button_state_by_name(name))
    except Exception:
        pressed = False
    edge = pressed and not last.get(name, False)
    last[name] = pressed
    return edge


def update_button_states(xr_client, names: list[str], last: dict[str, bool]) -> dict[str, bool]:
    """Update button states and return which ones just pressed."""
    return {name: button_edge(xr_client, name, last) for name in names}


def read_upper_body_poses(xr_client) -> np.ndarray | None:
    """Read PICO body tracking data."""
    from robot.config import BODY, REQUIRED_UPPER_BODY
    from teleop.retargeting import pose_is_valid

    body = xr_client.get_body_tracking_data()
    if body is None:
        return None
    poses = np.asarray(body["poses"], dtype=float)
    if poses.ndim != 2 or poses.shape[1] < 7:
        return None
    max_required_index = max(BODY[name] for name in REQUIRED_UPPER_BODY)
    if poses.shape[0] <= max_required_index:
        return None
    for name in REQUIRED_UPPER_BODY:
        if not pose_is_valid(poses, name):
            return None

    poses = poses.copy()
    if not pose_is_valid(poses, "head") and BODY["head"] < poses.shape[0]:
        poses[BODY["head"]] = poses[BODY["neck"]]
    for side in ["left", "right"]:
        hand_key = f"{side}_hand"
        wrist_key = f"{side}_wrist"
        if BODY[hand_key] < poses.shape[0] and not pose_is_valid(poses, hand_key):
            poses[BODY[hand_key]] = poses[BODY[wrist_key]]
    return poses
