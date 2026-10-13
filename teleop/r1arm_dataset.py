"""R1Arm MuJoCo dataset helpers using the simulator's compact action space.

The MCAP container and camera keys retain the established episode schema. The
vector itself contains only joints controlled by R1Arm:

* single arm: arm(7) + one logical Dex1 gripper DoF = 8
* dual arm: right(7) + right gripper(1) + left(7) + left gripper(1) = 16

MuJoCo models both Dex1 fingers as separate slide joints. They receive the
same command, so recording both would duplicate one physical control DoF.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from robot.r1arm_config import ARM_JOINTS, LEFT_ARM_JOINTS
from teleop.mcap_writer import FourCameraMcapRecorder


ARM_DOF_PER_SIDE = 7
MUJOCO_FINGER_DOF_PER_SIDE = 2
LOGICAL_GRIPPER_DOF_PER_SIDE = 1
SINGLE_ARM_DATASET_DOF = ARM_DOF_PER_SIDE + LOGICAL_GRIPPER_DOF_PER_SIDE
DUAL_ARM_DATASET_DOF = 2 * SINGLE_ARM_DATASET_DOF
CAMERA_VIEW_TO_COLOR_KEY = {
    "head_left": "color_0_head_left",
    "head_right": "color_1_head_right",
    "wrist_left": "color_2_wrist_left",
    "wrist_right": "color_3_wrist_right",
}
CAMERA_VIEW_CHOICES = tuple(CAMERA_VIEW_TO_COLOR_KEY)


@dataclass(frozen=True)
class _ViewSpec:
    lookat: tuple[float, float, float]
    distance: float
    azimuth: float
    elevation: float


class R1ArmObservationRenderer:
    """Render synchronized MuJoCo views under the stable MCAP camera keys."""

    CAMERA_ORDER = FourCameraMcapRecorder.CAMERA_ORDER

    def __init__(
        self,
        sim,
        *,
        width: int = 320,
        height: int = 240,
        dual_arm: bool = False,
        camera_views: tuple[str, ...] | list[str] | None = None,
    ):
        import mujoco

        self._mujoco = mujoco
        self._renderer = mujoco.Renderer(sim.model, height=int(height), width=int(width))
        self._sim = sim
        if dual_arm:
            lookat = (0.0, -0.30, 0.42)
            view_specs = {
                "head_left": _ViewSpec(lookat, 1.35, 145.0, -24.0),
                "head_right": _ViewSpec(lookat, 1.35, 215.0, -24.0),
                "wrist_left": _ViewSpec((-0.26, -0.35, 0.34), 0.82, 75.0, -18.0),
                "wrist_right": _ViewSpec((0.26, -0.35, 0.34), 0.82, 285.0, -18.0),
            }
        else:
            lookat = (0.0, -0.20, 0.30)
            view_specs = {
                "head_left": _ViewSpec(lookat, 0.92, 140.0, -22.0),
                "head_right": _ViewSpec(lookat, 0.92, 220.0, -22.0),
                "wrist_left": _ViewSpec((0.0, -0.30, 0.30), 0.68, 82.0, -16.0),
                "wrist_right": _ViewSpec((0.0, -0.30, 0.30), 0.68, 278.0, -16.0),
            }
        selected = tuple(camera_views or CAMERA_VIEW_CHOICES)
        unknown = [name for name in selected if name not in view_specs]
        if unknown:
            raise ValueError(f"unknown camera views: {unknown}; choices={CAMERA_VIEW_CHOICES}")
        if not selected or len(set(selected)) != len(selected):
            raise ValueError(f"camera views must be non-empty and unique: {selected}")
        self.camera_views = selected
        self.color_keys = tuple(CAMERA_VIEW_TO_COLOR_KEY[name] for name in selected)
        self._views = tuple(view_specs[name] for name in selected)

    def render(self) -> tuple[np.ndarray, ...]:
        frames = []
        for spec in self._views:
            camera = self._mujoco.MjvCamera()
            camera.type = self._mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = spec.lookat
            camera.distance = spec.distance
            camera.azimuth = spec.azimuth
            camera.elevation = spec.elevation
            self._renderer.update_scene(self._sim.data, camera=camera)
            rgb = self._renderer.render().copy()
            frames.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        return tuple(frames)

    def close(self) -> None:
        self._renderer.close()


class R1ArmDatasetAdapter:
    """Translate physical MuJoCo joints to the compact dataset action space."""

    def __init__(self, *, dual_arm: bool, control_side: str = "right") -> None:
        self.dual_arm = bool(dual_arm)
        self.control_side = str(control_side).lower()
        if self.control_side not in {"left", "right"}:
            raise ValueError(f"control_side must be left|right, got {control_side!r}")

    @property
    def vector_size(self) -> int:
        return DUAL_ARM_DATASET_DOF if self.dual_arm else SINGLE_ARM_DATASET_DOF

    @property
    def joint_names(self) -> tuple[str, ...]:
        if not self.dual_arm:
            return (*ARM_JOINTS, "dex1_gripper_joint")
        # This is the exact order used by R1ArmMujocoTeleopController.sim_q().
        return (
            *ARM_JOINTS,
            "dex1_gripper_joint",
            *LEFT_ARM_JOINTS,
            "left_dex1_gripper_joint",
        )

    @staticmethod
    def _compress_side(side_q: np.ndarray) -> np.ndarray:
        side_q = np.asarray(side_q, dtype=float).reshape(-1)
        if side_q.size != ARM_DOF_PER_SIDE + MUJOCO_FINGER_DOF_PER_SIDE:
            raise ValueError(f"one MuJoCo arm side must have 9 values, got {side_q.size}")
        # Contact can make the measured fingers differ slightly. Their mean is
        # the logical one-DoF gripper state; commands are identical.
        gripper = float(np.mean(side_q[ARM_DOF_PER_SIDE:]))
        return np.hstack([side_q[:ARM_DOF_PER_SIDE], gripper])

    @staticmethod
    def _expand_side(side_q: np.ndarray) -> np.ndarray:
        side_q = np.asarray(side_q, dtype=float).reshape(-1)
        if side_q.size != SINGLE_ARM_DATASET_DOF:
            raise ValueError(f"one compact arm side must have 8 values, got {side_q.size}")
        return np.hstack([side_q[:ARM_DOF_PER_SIDE], [side_q[-1], side_q[-1]]])

    def compact_sim_q(self, sim_q: np.ndarray) -> np.ndarray:
        """Compress sim order without adding body or inactive-arm padding."""
        sim_q = np.asarray(sim_q, dtype=float).reshape(-1)
        expected = 18 if self.dual_arm else 9
        if sim_q.size != expected:
            raise ValueError(f"R1Arm sim vector has {sim_q.size} values; expected {expected}")
        if not self.dual_arm:
            return self._compress_side(sim_q)
        # MuJoCo/controller order is right(9), then left(9).
        return np.hstack([self._compress_side(sim_q[:9]), self._compress_side(sim_q[9:18])])

    def _section(self, sim_q: np.ndarray, *, desired: bool) -> dict[str, Any]:
        compact = self.compact_sim_q(sim_q)
        if self.dual_arm:
            arms = np.hstack([compact[:7], compact[8:15]])
            grippers = np.array([compact[7], compact[15]], dtype=float)
        else:
            arms = compact[:7]
            grippers = compact[7:8]
        if desired:
            return {
                "action": compact.tolist(),
                "robot_q_desired": compact.tolist(),
                "arms": arms.tolist(),
                "hand_cmd": grippers.tolist(),
            }
        return {
            "joint_state": compact.tolist(),
            "robot_q_current": compact.tolist(),
            "arms": arms.tolist(),
            "hand_state": grippers.tolist(),
        }

    def frame_data(
        self, measured_sim_q: np.ndarray, commanded_sim_q: np.ndarray
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        return (
            self._section(measured_sim_q, desired=False),
            self._section(commanded_sim_q, desired=True),
        )

    def sim_q_from_frame(self, frame: dict[str, Any], mode: str = "action") -> np.ndarray:
        if mode not in {"action", "state"}:
            raise ValueError(f"mode must be action|state, got {mode!r}")
        section = frame.get("actions" if mode == "action" else "states", {})
        primary_key = "action" if mode == "action" else "joint_state"
        compatibility_key = "robot_q_desired" if mode == "action" else "robot_q_current"
        compact = np.asarray(
            section.get(primary_key, section.get(compatibility_key, [])), dtype=float
        ).reshape(-1)
        if compact.size != self.vector_size:
            raise ValueError(
                f"recorded {primary_key} has {compact.size} values; expected {self.vector_size}"
            )
        if not self.dual_arm:
            return self._expand_side(compact)
        return np.hstack([self._expand_side(compact[:8]), self._expand_side(compact[8:16])])


def validate_r1arm_frame(
    frame: dict[str, Any],
    *,
    expected_dof: int,
    expected_camera_keys: tuple[str, ...] | list[str] | None = None,
) -> None:
    """Raise ValueError unless a frame matches the compact R1Arm contract."""

    if expected_dof not in (SINGLE_ARM_DATASET_DOF, DUAL_ARM_DATASET_DOF):
        raise ValueError(f"R1Arm expected_dof must be 8 or 16, got {expected_dof}")
    colors = frame.get("colors", {})
    camera_keys = tuple(
        FourCameraMcapRecorder.CAMERA_ORDER if expected_camera_keys is None else expected_camera_keys
    )
    # An explicit empty tuple means "state/action-only episode" — skip the
    # camera checks instead of falling back to the four-camera default.
    missing = [key for key in camera_keys if key not in colors]
    if missing:
        raise ValueError(f"frame is missing camera observations: {missing}")
    unexpected = [key for key in colors if key not in camera_keys]
    if unexpected:
        raise ValueError(f"frame has unexpected camera observations: {unexpected}")
    for key in camera_keys:
        image = colors[key]
        if not isinstance(image, dict) or image.get("encoding") != "jpeg":
            raise ValueError(f"camera {key} is not a JPEG observation")

    arms_dof = 14 if expected_dof == DUAL_ARM_DATASET_DOF else 7
    gripper_dof = 2 if expected_dof == DUAL_ARM_DATASET_DOF else 1
    checks = (
        ("states", "joint_state", "robot_q_current", "hand_state"),
        ("actions", "action", "robot_q_desired", "hand_cmd"),
    )
    for section_name, primary_key, compatibility_key, hand_key in checks:
        section = frame.get(section_name, {})
        for key, size in (
            (primary_key, expected_dof),
            (compatibility_key, expected_dof),
            ("arms", arms_dof),
            (hand_key, gripper_dof),
        ):
            actual = np.asarray(section.get(key, [])).size
            if actual != size:
                raise ValueError(f"{section_name}.{key} has {actual} values; expected {size}")
        primary = np.asarray(section[primary_key], dtype=float)
        compatibility = np.asarray(section[compatibility_key], dtype=float)
        if not np.allclose(primary, compatibility):
            raise ValueError(
                f"{section_name}.{primary_key} and {compatibility_key} must be identical"
            )
