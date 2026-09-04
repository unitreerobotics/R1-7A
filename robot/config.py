"""R1-A7 robot configuration and constants."""

from __future__ import annotations

import json
import pathlib
from dataclasses import dataclass

import numpy as np


# URDF and asset paths (relative to package root)
DEFAULT_ASSETS_DIR = pathlib.Path(__file__).resolve().parents[1] / "assets"
DEFAULT_URDF_PATH = DEFAULT_ASSETS_DIR / "r1_a7" / "A7.urdf"

# Joint definitions
WAIST_JOINTS = ["waist_yaw_joint"]
LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
HEAD_JOINTS = ["head_pitch_joint", "head_yaw_joint"]
LEFT_DEX1_JOINTS = ["left_dex1_finger_joint_1", "left_dex1_finger_joint_2"]
RIGHT_DEX1_JOINTS = ["right_dex1_finger_joint_1", "right_dex1_finger_joint_2"]

# All arm joints
ARM_JOINTS = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

# Placo uses q[7:] for non-floating joints in URDF order
WAIST_SLICE = slice(0, 1)
LEFT_ARM_SLICE = slice(3, 10)
RIGHT_ARM_SLICE = slice(12, 19)
HEAD_SLICE = slice(1, 3)
LEFT_DEX1_SLICE = slice(10, 12)
RIGHT_DEX1_SLICE = slice(19, 21)

# Joint limits
WAIST_LOWER = -2.618
WAIST_UPPER = 2.618

ARM_LIMITS = {
    # Shoulder
    "left_shoulder_pitch_joint": (-3.1416, 2.0944),
    "left_shoulder_roll_joint": (-0.22689, 2.4784),
    "left_shoulder_yaw_joint": (-1.9199, 1.9199),
    # Elbow
    "left_elbow_joint": (-0.97564, 2.1852),
    # Wrist
    "left_wrist_roll_joint": (-1.9199, 1.9199),
    "left_wrist_pitch_joint": (-1.6144, 1.6144),
    "left_wrist_yaw_joint": (-1.9199, 1.9199),
    # Right shoulder
    "right_shoulder_pitch_joint": (-3.1416, 2.0944),
    "right_shoulder_roll_joint": (-2.47849, 0.2268),
    "right_shoulder_yaw_joint": (-1.9199, 1.9199),
    # Right elbow
    "right_elbow_joint": (-0.97564, 2.1852),
    # Right wrist
    "right_wrist_roll_joint": (-1.9199, 1.9199),
    "right_wrist_pitch_joint": (-1.6144, 1.6144),
    "right_wrist_yaw_joint": (-1.9199, 1.9199),
}

# Safe operating range: 85% of hard limits.  IK tries to keep joints here to
# avoid triggering hard-limit clamping and entering the singularity zone.
ARM_SAFE_RANGE_FRAC = 0.85

ARM_SAFE_LIMITS = {
    name: (float(lo + (hi - lo) * (1.0 - ARM_SAFE_RANGE_FRAC) / 2.0),
           float(hi - (hi - lo) * (1.0 - ARM_SAFE_RANGE_FRAC) / 2.0))
    for name, (lo, hi) in ARM_LIMITS.items()
}

# Initial (home) arm/waist configurations used by teleop controllers.
LEFT_ARM_INIT = np.array([0.3, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0], dtype=float)
RIGHT_ARM_INIT = np.array([0.3, 0.0, 0.0, -0.3, 0.0, 0.0, 0.0], dtype=float)
WAIST_INIT = np.array([0.0], dtype=float)

# Dex1 gripper parameters
# A7.urdf prismatic limits: upper=open, lower=close
DEX1_SIM_OPEN = 0.0245
DEX1_SIM_CLOSE = -0.02
DEX1_SIM_SPEED = 0.08

# Real Dex1 motor range (measured 2026-08-02)
# closed end: left +0.031, right -0.088
# open end stop: left 5.498, right 5.333
DEX1_REAL_OPEN = 5.0  # ~0.33 rad short of the tighter (right) stop
DEX1_REAL_CLOSE = 0.0  # fully closed
DEX1_TRIGGER_DEADBAND = 0.05  # PICO triggers rest slightly above zero

# EEF (end-effector) types
EEF_ALIASES = {
    "dex1": "dex1",
    "brainco": "brainco",
    "brainco_hand": "brainco",
    "inspire": "inspire",
    "inspire_hand": "inspire",
}
EEF_DOF = {"dex1": 2, "brainco": 6, "inspire": 6}
BRAINCO_CMD_MAX = 0.7
BRAINCO_THUMB_AUX_OPEN = 0.8

# PICO body-tracking landmark indices (24-key PICO body tracking API)
BODY = {
    "pelvis": 0,
    "neck": 12,
    "head": 15,
    "left_shoulder": 16,
    "right_shoulder": 17,
    "left_elbow": 18,
    "right_elbow": 19,
    "left_wrist": 20,
    "right_wrist": 21,
    "left_hand": 22,
    "right_hand": 23,
}
REQUIRED_UPPER_BODY = [
    "pelvis",
    "neck",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
]
OPTIONAL_UPPER_BODY = ["head", "left_hand", "right_hand"]


@dataclass
class R1A7Config:
    """R1-A7 robot configuration."""

    urdf_path: str = str(DEFAULT_URDF_PATH)
    head_pose_file: str = ""

    # Joint control parameters
    waist_lower: float = WAIST_LOWER
    waist_upper: float = WAIST_UPPER
    elbow_lower_override: float = -1.30  # Override URDF elbow limit

    # Dex1 gripper parameters
    dex1_open: float = DEX1_SIM_OPEN
    dex1_close: float = DEX1_SIM_CLOSE
    dex1_speed: float = DEX1_SIM_SPEED
    trigger_deadband: float = DEX1_TRIGGER_DEADBAND

    # IK parameters
    scale_factor: float = 1.0
    arm_control_mode: str = "pose"  # "pose" or "position"
    arm_task_weight: float = 1.0


def load_head_pose(path: str | pathlib.Path) -> np.ndarray:
    """Load head pose from JSON file."""
    pose_path = pathlib.Path(path).expanduser()
    try:
        data = json.loads(pose_path.read_text(encoding="utf-8"))
        pose = np.array([float(data["head_pitch_rad"]), float(data["head_yaw_rad"])], dtype=float)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise RuntimeError(f"Cannot load head pose file {pose_path}: {exc}") from exc

    lower = np.array([-0.62832, -2.0071])
    upper = np.array([0.62832, 2.0071])
    if np.any(pose < lower) or np.any(pose > upper):
        raise ValueError(f"Head pose {pose.tolist()} is outside A7 URDF joint limits")
    return pose


def normalize_eef_name(eef: str) -> str:
    """Normalize end-effector name to canonical form."""
    key = str(eef).strip().lower()
    if key not in EEF_ALIASES:
        choices = ", ".join(sorted(EEF_ALIASES))
        raise ValueError(f"Unknown eef {eef!r}. Choices: {choices}")
    return EEF_ALIASES[key]
