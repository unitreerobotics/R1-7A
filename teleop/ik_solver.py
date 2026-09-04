"""Damped least-squares IK solver for R1-A7 arms."""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from robot.arm_model import clamp_configuration
from robot.config import ARM_SAFE_LIMITS


def solve_upper_arm_ik(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    ee_frame_name: str,
    ee_target_pos: np.ndarray,
    ee_target_rot: np.ndarray | None,
    elbow_frame_name: str | None,
    elbow_target_pos: np.ndarray | None,
    q_indices: list[int],
    q_reference: np.ndarray,
    control_mode: str,
    position_weight: float = 1.0,
    orientation_weight: float = 1.0,
    elbow_weight: float = 1.0,
    regularization_weight: float = 0.05,
    damping: float = 0.02,
    damping_max: float = 0.5,
    manipulability_threshold: float = 0.05,
    joint_limit_margin: float = 0.15,
    joint_limit_weight: float = 2.0,
    max_step: float = 0.03,
    iterations: int = 100,
    debug: bool = False,
) -> tuple[np.ndarray, dict]:
    """
    Solve inverse kinematics for an arm using damped least-squares.

    Args:
        model: Pinocchio model
        data: Pinocchio data
        q: Current joint configuration
        ee_frame_name: End-effector frame name
        ee_target_pos: Target position for end-effector
        ee_target_rot: Target rotation matrix for end-effector
        elbow_frame_name: Elbow frame name (for pose-elbow mode)
        elbow_target_pos: Target position for elbow
        q_indices: Joint indices to control
        q_reference: Reference configuration for regularization
        control_mode: "position", "pose", or "pose-elbow"
        position_weight: Weight for position error
        orientation_weight: Weight for orientation error
        elbow_weight: Weight for elbow position error
        regularization_weight: Weight for configuration regularization
        damping: Damping factor for pseudo-inverse
        max_step: Maximum step per iteration
        iterations: Maximum iterations

    Returns:
        (q_next, diagnostics): Solved configuration and diagnostics
    """
    q_next = q.copy()
    q_start = q.copy()
    ee_frame_id = model.getFrameId(ee_frame_name)
    elbow_frame_id = model.getFrameId(elbow_frame_name) if elbow_frame_name else None
    q_indices_array = np.asarray(q_indices, dtype=int)
    first_jac = None
    first_error_norm = 0.0
    _iter_count = 0

    for _ in range(max(1, iterations)):
        _iter_count += 1
        pin.forwardKinematics(model, data, q_next)
        pin.updateFramePlacements(model, data)

        rows = []
        errors = []

        # Position error
        ee_pose = data.oMf[ee_frame_id]
        pos_error = ee_target_pos - ee_pose.translation
        rows.append(
            position_weight
            * pin.computeFrameJacobian(
                model,
                data,
                q_next,
                ee_frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )[:3, q_indices_array]
        )
        errors.append(position_weight * pos_error)

        # Orientation error
        if control_mode in {"pose", "pose-elbow"} and ee_target_rot is not None:
            rot_error_local = pin.log3(ee_pose.rotation.T @ ee_target_rot)
            rot_error = ee_pose.rotation @ rot_error_local
            rows.append(
                orientation_weight
                * pin.computeFrameJacobian(
                    model,
                    data,
                    q_next,
                    ee_frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )[3:6, q_indices_array]
            )
            errors.append(orientation_weight * rot_error)

        # Elbow position error
        if (
            control_mode == "pose-elbow"
            and elbow_frame_id is not None
            and elbow_target_pos is not None
        ):
            elbow_pose = data.oMf[elbow_frame_id]
            elbow_error = elbow_target_pos - elbow_pose.translation
            rows.append(
                elbow_weight
                * pin.computeFrameJacobian(
                    model,
                    data,
                    q_next,
                    elbow_frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )[:3, q_indices_array]
            )
            errors.append(elbow_weight * elbow_error)

        # Regularization
        if regularization_weight > 0.0:
            rows.append(regularization_weight * np.eye(len(q_indices)))
            errors.append(regularization_weight * (q_reference[q_indices_array] - q_next[q_indices_array]))

        # Joint-limit potential field: when a joint is within `margin` of its
        # safe limit, add a large error term that pushes it back toward centre.
        # This prevents the arm from reaching the URDF hard-limit where clamping
        # would cause violent oscillation.
        if joint_limit_weight > 0.0:
            for i, name in enumerate(q_indices):
                if name not in ARM_SAFE_LIMITS:
                    continue
                lo, hi = ARM_SAFE_LIMITS[name]
                mid = (lo + hi) * 0.5
                half = (hi - lo) * 0.5
                q_val = q_next[q_indices_array[i]]
                # Distance from centre of safe range (normalised 0..1)
                dist_norm = abs(q_val - mid) / half  # 0 = centre, 1 = at limit
                if dist_norm > 1.0 - joint_limit_margin:
                    # Quadratic repulsion: stronger closer to the hard limit
                    strength = joint_limit_weight * ((dist_norm - (1.0 - joint_limit_margin)) / joint_limit_margin) ** 2
                    err_dir = -1.0 if q_val > mid else 1.0
                    err = strength * err_dir
                    limit_jac = np.zeros(len(q_indices))
                    limit_jac[i] = 1.0
                    rows.append(limit_jac)
                    errors.append(np.array([err]))

        jac = np.vstack(rows)
        error = np.concatenate(errors)

        if first_jac is None:
            first_jac = jac.copy()
            first_error_norm = float(np.linalg.norm(error))
            first_jac_man = float(np.sqrt(max(0.0, np.linalg.det(jac[:3] @ jac[:3].T))))

        # Manipulability-aware damping: near singularities (low manipulability)
        # the damping is smoothly ramped up to damping_max to suppress oscillation.
        man = float(np.sqrt(max(0.0, np.linalg.det(jac[:3] @ jac[:3].T))))
        d = damping if man > manipulability_threshold else float(
            damping + (damping_max - damping) * (1.0 - man / max(man, 1e-9))
        )

        # Damped least-squares solve
        lhs = jac @ jac.T + d * np.eye(jac.shape[0])
        dq = jac.T @ np.linalg.solve(lhs, error)
        dq = np.clip(dq, -max_step, max_step)
        q_next[q_indices_array] += dq
        q_next = clamp_configuration(model, q_next)

        if np.linalg.norm(error) < 1e-4:
            break

    # Compute final diagnostics
    pin.forwardKinematics(model, data, q_next)
    pin.updateFramePlacements(model, data)
    final_ee_pose = data.oMf[ee_frame_id]
    final_pos_error = ee_target_pos - final_ee_pose.translation

    diagnostics = {
        "dq_total": q_next[q_indices_array] - q_start[q_indices_array],
        "jac_col_norm": np.linalg.norm(first_jac, axis=0) if first_jac is not None else np.zeros(len(q_indices)),
        "start_error_norm": first_error_norm,
        "final_error_norm": float(np.linalg.norm(error)),
        "target_pos": ee_target_pos.copy(),
        "final_pos": final_ee_pose.translation.copy(),
        "ee_pos_error": final_pos_error.copy(),
        "ee_pos_error_norm": float(np.linalg.norm(final_pos_error)),
        "q_indices": q_indices_array.copy(),
        "manipulability": float(man),
    }

    if control_mode in {"pose", "pose-elbow"} and ee_target_rot is not None:
        final_rot_local = pin.log3(final_ee_pose.rotation.T @ ee_target_rot)
        final_rot_error = final_ee_pose.rotation @ final_rot_local
        diagnostics["rot_error_norm"] = float(np.linalg.norm(final_rot_error))

    if elbow_target_pos is not None:
        final_elbow_pos = data.oMf[elbow_frame_id].translation.copy()
        final_elbow_error = elbow_target_pos - final_elbow_pos
        diagnostics.update({
            "elbow_target_pos": elbow_target_pos.copy(),
            "final_elbow_pos": final_elbow_pos,
            "elbow_pos_error": final_elbow_error,
            "elbow_pos_error_norm": float(np.linalg.norm(final_elbow_error)),
        })

        if debug:
            dist = np.linalg.norm(final_pos_error)
            reached = dist < 0.01
            print(
                f"[DLS IK {'✓' if reached else '✗'}] "
                f"pos_err={dist:.4f}m  man={man:.4f}  d={d:.4f}  "
                f"iter={_iter_count}  ee={ee_frame_name}"
            )

        return q_next, diagnostics


def solve_position_ik(
    model: pin.Model,
    data: pin.Data,
    q: np.ndarray,
    frame_name: str,
    target_pos: np.ndarray,
    q_indices: list[int],
    damping: float = 0.01,
    max_step: float = 0.1,
    iterations: int = 100,
) -> tuple[np.ndarray, dict]:
    """Solve position-only inverse kinematics."""
    q_next = q.copy()
    q_start = q.copy()
    frame_id = model.getFrameId(frame_name)
    q_indices_array = np.asarray(q_indices, dtype=int)
    first_jac = None
    first_error_norm = 0.0
    _iter_count = 0

    for _ in range(max(1, iterations)):
        _iter_count += 1
        pin.forwardKinematics(model, data, q_next)
        pin.updateFramePlacements(model, data)
        current = data.oMf[frame_id].translation.copy()
        error = target_pos - current

        jac = pin.computeFrameJacobian(
            model,
            data,
            q_next,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )[:3, q_indices_array]

        if first_jac is None:
            first_error_norm = float(np.linalg.norm(error))
            first_jac = jac.copy()

        lhs = jac @ jac.T + damping * np.eye(3)
        dq = jac.T @ np.linalg.solve(lhs, error)
        dq = np.clip(dq, -max_step, max_step)
        q_next[q_indices_array] += dq
        q_next = clamp_configuration(model, q_next)

        if np.linalg.norm(error) < 1e-4:
            break

    pin.forwardKinematics(model, data, q_next)
    pin.updateFramePlacements(model, data)
    final_error = target_pos - data.oMf[frame_id].translation.copy()

    diagnostics = {
        "dq_total": q_next[q_indices_array] - q_start[q_indices_array],
        "jac_col_norm": np.linalg.norm(first_jac, axis=0) if first_jac is not None else np.zeros(len(q_indices)),
        "start_error_norm": first_error_norm,
        "final_error_norm": float(np.linalg.norm(final_error)),
        "target_pos": target_pos.copy(),
        "final_pos": data.oMf[frame_id].translation.copy(),
        "q_indices": q_indices_array.copy(),
    }
    return q_next, diagnostics
