"""PICO body/controller pose retargeting for R1-A7."""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as R

from robot.config import BODY, REQUIRED_UPPER_BODY, OPTIONAL_UPPER_BODY

__all__ = [
    "BODY",
    "REQUIRED_UPPER_BODY",
    "OPTIONAL_UPPER_BODY",
    "derive_features",
    "wrist_local_position",
    "elbow_local_position",
    "hand_rotation_body",
    "refresh_body_neutral",
    "refresh_controller_neutral",
    "pose_is_valid",
    "pose_rotation",
    "local_vector",
    "body_frame",
]


def wrist_local_position(poses: np.ndarray, side: str) -> np.ndarray:
    """Get wrist position relative to body frame."""
    frame = body_frame(poses)
    wrist = poses[BODY[f"{side}_wrist"], :3]
    return local_vector(frame, wrist - poses[BODY["pelvis"], :3])


def hand_rotation_body(poses: np.ndarray, side: str) -> np.ndarray:
    """Get hand rotation relative to body frame."""
    frame = body_frame(poses)
    hand_key = f"{side}_hand"
    wrist_key = f"{side}_wrist"
    pose = poses[BODY[hand_key]] if pose_is_valid(poses, hand_key) else poses[BODY[wrist_key]]
    return frame.T @ pose_rotation(pose).as_matrix()


def elbow_local_position(poses: np.ndarray, side: str) -> np.ndarray:
    """Get elbow position relative to body frame."""
    frame = body_frame(poses)
    elbow = poses[BODY[f"{side}_elbow"], :3]
    return local_vector(frame, elbow - poses[BODY["pelvis"], :3])


def derive_features(poses: np.ndarray) -> dict[str, float]:
    """
    Derive R1-A7 joint features from PICO body tracking data.

    Args:
        poses: (24, 7) array of body poses [x, y, z, qx, qy, qz, qw]

    Returns:
        Dictionary mapping joint names to target angles
    """
    frame = body_frame(poses)
    features: dict[str, float] = {}

    # Torso and head
    torso_yaw = math.atan2(frame[1, 1], frame[0, 1])
    features["waist_yaw_joint"] = float(torso_yaw)

    if pose_is_valid(poses, "head"):
        head_rot_body = frame.T @ pose_rotation(poses[BODY["head"]]).as_matrix()
        head_euler = R.from_matrix(head_rot_body).as_euler("xyz", degrees=False)
        features["head_pitch_joint"] = float(head_euler[1])
        features["head_yaw_joint"] = float(head_euler[2])

    body_forward = np.array([0.0, 1.0, 0.0])

    for side in ["left", "right"]:
        shoulder = poses[BODY[f"{side}_shoulder"], :3]
        elbow = poses[BODY[f"{side}_elbow"], :3]
        wrist = poses[BODY[f"{side}_wrist"], :3]

        upper = local_vector(frame, elbow - shoulder)
        forearm = local_vector(frame, wrist - elbow)

        # Arm-down neutral gives pitch ~= 0 and roll ~= 0
        shoulder_pitch = math.atan2(float(upper[1]), float(-upper[2]))
        shoulder_roll = -math.atan2(float(upper[0]), float(-upper[2]))

        upper_axis = normalize(upper)
        ref = project_on_plane(body_forward, upper_axis)
        fore_proj = project_on_plane(forearm, upper_axis)
        shoulder_yaw = signed_angle(ref, fore_proj, upper_axis)
        elbow_flex = angle_between(upper, forearm)

        wrist_euler = wrist_euler_in_body(poses, frame, side)

        prefix = f"{side}_"
        features[f"{prefix}shoulder_pitch_joint"] = float(shoulder_pitch)
        features[f"{prefix}shoulder_roll_joint"] = float(shoulder_roll)
        features[f"{prefix}shoulder_yaw_joint"] = float(shoulder_yaw)
        features[f"{prefix}elbow_joint"] = float(elbow_flex)
        features[f"{prefix}wrist_roll_joint"] = float(wrist_euler[0])
        features[f"{prefix}wrist_pitch_joint"] = float(wrist_euler[1])
        features[f"{prefix}wrist_yaw_joint"] = float(wrist_euler[2])

    return features


def pose_rotation(pose: np.ndarray) -> R:
    """Extract rotation from pose quaternion."""
    return R.from_quat([pose[3], pose[4], pose[5], pose[6]])


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Compute angle between two vectors."""
    a = normalize(a)
    b = normalize(b)
    if np.linalg.norm(a) < 1e-8 or np.linalg.norm(b) < 1e-8:
        return 0.0
    return float(math.acos(np.clip(np.dot(a, b), -1.0, 1.0)))


def signed_angle(a: np.ndarray, b: np.ndarray, axis: np.ndarray) -> float:
    """Compute signed angle between two vectors around an axis."""
    a = normalize(a)
    b = normalize(b)
    axis = normalize(axis)
    if np.linalg.norm(a) < 1e-8 or np.linalg.norm(b) < 1e-8 or np.linalg.norm(axis) < 1e-8:
        return 0.0
    return float(math.atan2(np.dot(axis, np.cross(a, b)), np.dot(a, b)))


def normalize(v: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    """Normalize a vector."""
    n = float(np.linalg.norm(v))
    if n < eps or not np.isfinite(n):
        return np.zeros_like(v)
    return v / n


def project_on_plane(v: np.ndarray, normal: np.ndarray) -> np.ndarray:
    """Project vector onto a plane defined by its normal."""
    normal = normalize(normal)
    return v - normal * float(np.dot(v, normal))


def body_frame(poses: np.ndarray) -> np.ndarray:
    """Compute body reference frame from pelvis and shoulders."""
    pelvis = poses[BODY["pelvis"], :3]
    neck = poses[BODY["neck"], :3]
    left_shoulder = poses[BODY["left_shoulder"], :3]
    right_shoulder = poses[BODY["right_shoulder"], :3]

    right = normalize(right_shoulder - left_shoulder)
    up = normalize(neck - pelvis)
    forward = normalize(np.cross(up, right))
    right = normalize(np.cross(forward, up))

    if np.linalg.norm(right) < 1e-8 or np.linalg.norm(forward) < 1e-8 or np.linalg.norm(up) < 1e-8:
        return np.eye(3)
    return np.column_stack([right, forward, up])


def wrist_euler_in_body(poses: np.ndarray, frame_body_to_world: np.ndarray, side: str) -> np.ndarray:
    """Get wrist Euler angles in body frame."""
    hand_key = f"{side}_hand"
    wrist_key = f"{side}_wrist"
    pose = poses[BODY[hand_key]] if pose_is_valid(poses, hand_key) else poses[BODY[wrist_key]]
    rot_world = pose_rotation(pose).as_matrix()
    rot_body = frame_body_to_world.T @ rot_world
    return R.from_matrix(rot_body).as_euler("xyz", degrees=False)


def local_vector(frame_body_to_world: np.ndarray, v_world: np.ndarray) -> np.ndarray:
    """Transform world vector to body local frame."""
    return frame_body_to_world.T @ v_world


def headset_reference_frame(headset_pose: np.ndarray, mode: str = "full") -> np.ndarray:
    """Compute headset reference frame."""
    rot = pose_rotation(headset_pose).as_matrix()
    if mode == "full":
        return rot

    world_up = np.array([0.0, 0.0, 1.0])
    # Use headset local +Y as forward, projected onto horizontal plane
    forward = project_on_plane(rot[:, 1], world_up)
    if np.linalg.norm(forward) < 1e-8:
        forward = np.array([0.0, 1.0, 0.0])
    forward = normalize(forward)
    right = normalize(np.cross(forward, world_up))
    forward = normalize(np.cross(world_up, right))
    return np.column_stack([right, forward, world_up])


def controller_local_pose(
    poses: dict[str, np.ndarray],
    side: str,
    head_reference_mode: str = "full",
) -> tuple[np.ndarray, np.ndarray]:
    """Get controller pose relative to headset."""
    headset_pose = poses["headset"]
    controller_pose = poses[f"{side}_controller"]
    frame = headset_reference_frame(headset_pose, head_reference_mode)
    local_pos = frame.T @ (controller_pose[:3] - headset_pose[:3])
    local_rot = frame.T @ pose_rotation(controller_pose).as_matrix()
    return local_pos, local_rot


def refresh_body_neutral(poses: np.ndarray) -> tuple[dict, dict, dict]:
    """Capture neutral body pose for retargeting calibration."""
    wrist_local = {
        "left": wrist_local_position(poses, "left"),
        "right": wrist_local_position(poses, "right"),
    }
    elbow_local = {
        "left": elbow_local_position(poses, "left"),
        "right": elbow_local_position(poses, "right"),
    }
    hand_rot_body = {
        "left": hand_rotation_body(poses, "left"),
        "right": hand_rotation_body(poses, "right"),
    }
    return wrist_local, elbow_local, hand_rot_body


def refresh_controller_neutral(
    controller_poses: dict[str, np.ndarray],
    head_reference_mode: str = "full",
) -> tuple[dict, dict]:
    """Capture neutral controller pose."""
    local_pos: dict = {}
    local_rot: dict = {}
    for side in ["left", "right"]:
        local_pos[side], local_rot[side] = controller_local_pose(
            controller_poses, side, head_reference_mode
        )
    return local_pos, local_rot


def pose_is_valid(poses: np.ndarray, key: str) -> bool:
    """Check if a body pose landmark is valid."""
    index = BODY[key]
    return index < poses.shape[0] and poses.shape[1] >= 7 and np.isfinite(poses[index, :7]).all()
