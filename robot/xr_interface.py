"""XRoboToolkit/PICO XR interface for R1Arm teleoperation.

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
