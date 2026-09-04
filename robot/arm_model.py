"""Pinocchio model helpers and joint utilities for R1-A7."""

from __future__ import annotations

import numpy as np
import pinocchio as pin


def joint_ids_by_name(model: pin.Model) -> dict[str, int]:
    """Build a mapping from joint name to joint ID."""
    return {model.names[joint_id]: joint_id for joint_id in range(1, len(model.joints))}


def joint_indices_by_name(model: pin.Model) -> dict[str, int]:
    """Build a mapping from joint name to q index."""
    result: dict[str, int] = {}
    for joint_id in range(1, len(model.joints)):
        joint = model.joints[joint_id]
        if joint.nq == 1:
            result[model.names[joint_id]] = joint.idx_q
    return result


def clamp_configuration(model: pin.Model, q: np.ndarray) -> np.ndarray:
    """Clamp joint configuration to URDF limits."""
    lower = model.lowerPositionLimit
    upper = model.upperPositionLimit
    finite = np.isfinite(lower) & np.isfinite(upper)
    q[finite] = np.minimum(np.maximum(q[finite], lower[finite]), upper[finite])
    return q


def neutral_configuration(model: pin.Model) -> np.ndarray:
    """Get neutral configuration respecting limits."""
    return clamp_configuration(model, pin.neutral(model))


def load_robot_model(urdf_path: str) -> tuple[pin.Model, pin.Data]:
    """Load a Pinocchio model and data from a URDF."""
    from pathlib import Path

    urdf_path = Path(urdf_path)
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    return model, data


def link_pose(
    model: pin.Model, data: pin.Data, q: np.ndarray, frame_name: str
) -> tuple[np.ndarray, np.ndarray]:
    """Get a frame's world pose (position, xyzw quaternion) at configuration q.

    Recomputes forward kinematics for ``q`` before reading the frame placement, so
    the returned pose always matches ``q`` (updateFramePlacements alone would
    reuse a stale configuration — the cause of the "jump to origin" when
    re-gripping an arm).
    """
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    frame = data.oMf[model.getFrameId(frame_name)]
    quat = pin.Quaternion(frame.rotation).coeffs()  # [x, y, z, w]
    return frame.translation.copy(), quat
