"""Thin wrapper over xrobotoolkit_sdk — zero dependency on xrobotoolkit_teleop.

Drop-in replacement for ``xrobotoolkit_teleop.common.xr_client.XrClient``,
backed directly by the native pybind11 module from
``third_party/XRoboToolkit-PC-Service-Pybind``.
"""

from __future__ import annotations

import ctypes
from pathlib import Path
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# Preload the native .so before importing the pybind module
# ---------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parents[1]
_NATIVE_LIB = _THIS_DIR / "third_party" / "XRoboToolkit-PC-Service-Pybind" / "lib" / "libPXREARobotSDK.so"

_preloaded = False


def preload_native_lib() -> None:
    """Preload libPXREARobotSDK.so so the pybind module can link against it."""
    global _preloaded
    if _preloaded:
        return
    if _NATIVE_LIB.exists():
        try:
            ctypes.CDLL(str(_NATIVE_LIB), mode=ctypes.RTLD_GLOBAL)
            _preloaded = True
        except OSError:
            pass


# ---------------------------------------------------------------------------
# XrClient
# ---------------------------------------------------------------------------


class XrClient:
    """Client for the XRoboToolkit native SDK (``xrobotoolkit_sdk``).

    Same interface as ``xrobotoolkit_teleop.common.xr_client.XrClient`` so
    existing code can switch with a one-line import change.
    """

    # ---- lifecycle ---------------------------------------------------------

    def __init__(self) -> None:
        preload_native_lib()
        import xrobotoolkit_sdk as _xrt  # noqa: E402

        self._xrt = _xrt
        self._xrt.init()

    def close(self) -> None:
        self._xrt.close()

    # ---- controller / headset poses (7-element: x,y,z,qx,qy,qz,qw) -------

    def get_pose_by_name(self, name: str) -> np.ndarray:
        if name == "headset":
            return np.asarray(self._xrt.get_headset_pose(), dtype=float)
        if name == "left_controller":
            return np.asarray(self._xrt.get_left_controller_pose(), dtype=float)
        if name == "right_controller":
            return np.asarray(self._xrt.get_right_controller_pose(), dtype=float)
        raise ValueError(
            f"Invalid name: {name!r}. Valid: 'headset', 'left_controller', 'right_controller'."
        )

    # ---- triggers / grips (float 0…1) ------------------------------------

    def get_key_value_by_name(self, name: str) -> float:
        if name == "left_trigger":
            return float(self._xrt.get_left_trigger())
        if name == "right_trigger":
            return float(self._xrt.get_right_trigger())
        if name == "left_grip":
            return float(self._xrt.get_left_grip())
        if name == "right_grip":
            return float(self._xrt.get_right_grip())
        raise ValueError(f"Invalid name: {name!r}. Valid: triggers/grips.")

    # ---- buttons (bool) ---------------------------------------------------

    # Mapping (from py_bindings.cpp):
    #   X = left primary   A = right primary
    #   Y = left secondary B = right secondary
    _BUTTON_MAP: dict[str, str] = {
        "X": "get_X_button",
        "Y": "get_Y_button",
        "A": "get_A_button",
        "B": "get_B_button",
        "left_menu_button": "get_left_menu_button",
        "right_menu_button": "get_right_menu_button",
        "left_axis_click": "get_left_axis_click",
        "right_axis_click": "get_right_axis_click",
    }

    def get_button_state_by_name(self, name: str) -> bool:
        method = self._BUTTON_MAP.get(name)
        if method is None:
            raise ValueError(
                f"Invalid button: {name!r}. Valid: {sorted(self._BUTTON_MAP)}."
            )
        return bool(getattr(self._xrt, method)())

    # ---- joystick axes ----------------------------------------------------

    def get_joystick_state(self, controller: str) -> list[float]:
        if controller == "left":
            return list(self._xrt.get_left_axis())
        if controller == "right":
            return list(self._xrt.get_right_axis())
        raise ValueError(f"Invalid controller: {controller!r}. Valid: 'left', 'right'.")

    # ---- timestamp ---------------------------------------------------------

    def get_timestamp_ns(self) -> int:
        return int(self._xrt.get_time_stamp_ns())

    # ---- hand tracking (26×7 per hand) -------------------------------------

    def get_hand_tracking_state(self, hand: str) -> np.ndarray | None:
        if hand == "left":
            if not self._xrt.get_left_hand_is_active():
                return None
            return np.asarray(self._xrt.get_left_hand_tracking_state(), dtype=float)
        if hand == "right":
            if not self._xrt.get_right_hand_is_active():
                return None
            return np.asarray(self._xrt.get_right_hand_tracking_state(), dtype=float)
        raise ValueError(f"Invalid hand: {hand!r}. Valid: 'left', 'right'.")

    # ---- motion tracker (up to 3) -----------------------------------------

    def get_motion_tracker_data(self) -> dict[str, dict[str, Any]]:
        import xrobotoolkit_sdk as _xrt

        n = _xrt.num_motion_data_available()
        if n == 0:
            return {}

        poses = _xrt.get_motion_tracker_pose()
        velocities = _xrt.get_motion_tracker_velocity()
        accelerations = _xrt.get_motion_tracker_acceleration()
        serials = _xrt.get_motion_tracker_serial_numbers()

        data: dict[str, dict[str, Any]] = {}
        for i in range(n):
            serial = serials[i]
            data[serial] = {
                "pose": np.asarray(poses[i], dtype=float),
                "velocity": np.asarray(velocities[i], dtype=float),
                "acceleration": np.asarray(accelerations[i], dtype=float),
            }
        return data

    # ---- body tracking (24 joints) ----------------------------------------

    def get_body_tracking_data(self) -> dict[str, np.ndarray] | None:
        """Return body tracking data or None if unavailable.

        Returns dict with keys:
          "poses"         — (24, 7)  [x, y, z, qx, qy, qz, qw]
          "velocities"    — (24, 6)  [vx, vy, vz, wx, wy, wz]
          "accelerations" — (24, 6)  [ax, ay, az, wax, way, waz]
        """
        if not self._xrt.is_body_data_available():
            return None
        return {
            "poses": np.asarray(self._xrt.get_body_joints_pose(), dtype=float),
            "velocities": np.asarray(self._xrt.get_body_joints_velocity(), dtype=float),
            "accelerations": np.asarray(self._xrt.get_body_joints_acceleration(), dtype=float),
        }
