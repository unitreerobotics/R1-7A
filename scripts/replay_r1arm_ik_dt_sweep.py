#!/usr/bin/env python3
"""Replay one recorded EE-intent trace through several Placo dt values."""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.arm_model import link_pose
from robot.r1arm_config import R1ArmConfig
from teleop.r1arm_mujoco_controller import R1ArmMujocoTeleopController


def load_longest_intent_trace(path: Path) -> tuple[dict, list[dict]]:
    from mcap.reader import make_reader

    metadata = {}
    segments: list[list[dict]] = []
    current: list[dict] = []
    with path.open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            frame = json.loads(message.data)
            if channel.topic == "/episode/meta":
                metadata = frame
                continue
            if channel.topic != "/whole_body/frame":
                continue
            actions = frame.get("actions", {})
            ee = actions.get("ee_intent", {})
            position = np.asarray(ee.get("position", []), dtype=float).reshape(-1)
            quaternion = np.asarray(ee.get("quaternion_xyzw", []), dtype=float).reshape(-1)
            valid = (
                actions.get("xr_valid", False)
                and position.size == 3
                and np.all(np.isfinite(position))
                and (quaternion.size == 0 or (
                    quaternion.size == 4
                    and np.all(np.isfinite(quaternion))
                    and np.linalg.norm(quaternion) > 1e-6
                ))
            )
            if valid:
                stamp_ns = int(frame.get("sample_time_ns", message.log_time))
                current.append({
                    "source_idx": int(frame.get("idx", len(current))),
                    "stamp_ns": stamp_ns,
                    "position": position,
                    "quaternion": quaternion / np.linalg.norm(quaternion) if quaternion.size else None,
                })
            elif current:
                segments.append(current)
                current = []
    if current:
        segments.append(current)
    trace = max(segments, key=len, default=[])
    if len(trace) < 2:
        raise ValueError("Episode has no continuous valid ee_intent segment with at least 2 frames")
    return metadata, trace


def aligned_targets(controller, trace: list[dict]) -> list[dict]:
    base_pos, base_quat = link_pose(
        controller.model, controller.data, controller.q, controller.ee_frame
    )
    source_pos0 = trace[0]["position"]
    source_rot0 = R.from_quat(trace[0]["quaternion"]) if trace[0]["quaternion"] is not None else None
    base_rot = R.from_quat(base_quat)
    targets = []
    for sample in trace:
        target = {
            **sample,
            "target_pos": base_pos + sample["position"] - source_pos0,
            "target_quat": None,
        }
        if source_rot0 is not None and sample["quaternion"] is not None:
            target["target_quat"] = (
                R.from_quat(sample["quaternion"]) * source_rot0.inv() * base_rot
            ).as_quat()
        targets.append(target)
    return targets


def set_arm_q(controller, q_model: np.ndarray, arm_q: np.ndarray) -> np.ndarray:
    result = q_model.copy()
    for idx, value in zip(controller.arm_q_indices, arm_q):
        result[idx] = value
    return result


def replay_pass(controller, targets: list[dict], ik_dt: float, writer: csv.DictWriter,
                *, realtime: bool) -> dict:
    mujoco = controller.sim._mujoco
    controller.q = controller._create_initial_q()
    controller._reset_trajectory_state("right", controller.q)
    controller._solver_seeds["right"] = controller.q.copy()
    controller.track["right"].clear()
    controller._last_ik_raw["right"] = controller.arm_q("right")
    controller._last_ik_solver["right"] = "hold"
    controller.placo_solver.set_regularization_reference(controller.q)
    controller.placo_solver.solver.dt = float(ik_dt)
    controller.placo_solver.solver_dt = float(ik_dt)
    controller.t = 0.0
    mujoco.mj_resetData(controller.sim.model, controller.sim.data)
    controller.sim.set_q(controller.sim_q(), also_ctrl=True)

    raw_steps: list[float] = []
    purpose_steps: list[float] = []
    raw_pos_errors: list[float] = []
    purpose_pos_errors: list[float] = []
    saturated = 0
    previous_raw = None
    previous_purpose = None
    previous_stamp = targets[0]["stamp_ns"]
    t = 0.0

    for index, sample in enumerate(targets):
        step_dt = (targets[index + 1]["stamp_ns"] - sample["stamp_ns"]) * 1e-9 \
            if index + 1 < len(targets) else (sample["stamp_ns"] - previous_stamp) * 1e-9
        if not 1e-4 <= step_dt <= 0.1:
            step_dt = 1.0 / 30.0
        controller._step_dt = float(step_dt)
        controller._solve("right", sample["target_pos"], sample["target_quat"])

        raw = controller._last_ik_raw["right"].copy()
        purpose, velocity, acceleration = controller._trajectory_state("right", controller.q)
        raw_model = set_arm_q(controller, controller.q, raw)
        raw_pos, raw_quat = link_pose(controller.model, controller.data, raw_model, controller.ee_frame)
        purpose_pos, purpose_quat = link_pose(
            controller.model, controller.data, controller.q, controller.ee_frame
        )
        raw_pos_error = float(np.linalg.norm(raw_pos - sample["target_pos"]))
        purpose_pos_error = float(np.linalg.norm(purpose_pos - sample["target_pos"]))
        raw_pos_errors.append(raw_pos_error)
        purpose_pos_errors.append(purpose_pos_error)
        raw_step = float(np.max(np.abs(raw - previous_raw))) if previous_raw is not None else 0.0
        purpose_step = float(np.max(np.abs(purpose - previous_purpose))) if previous_purpose is not None else 0.0
        if previous_raw is not None:
            raw_steps.append(raw_step)
            purpose_steps.append(purpose_step)
        if np.max(np.abs(velocity)) >= 0.98:
            saturated += 1

        row = {
            "ik_dt_s": ik_dt,
            "source_idx": sample["source_idx"],
            "time_s": t,
            "step_dt_s": step_dt,
            "ik_solver": controller._last_ik_solver["right"],
            "target_x_m": sample["target_pos"][0],
            "target_y_m": sample["target_pos"][1],
            "target_z_m": sample["target_pos"][2],
            "raw_target_pos_error_m": raw_pos_error,
            "purpose_target_pos_error_m": purpose_pos_error,
            "raw_target_rot_error_rad": (
                float((R.from_quat(raw_quat).inv() * R.from_quat(sample["target_quat"])).magnitude())
                if sample["target_quat"] is not None else ""
            ),
            "purpose_target_rot_error_rad": (
                float((R.from_quat(purpose_quat).inv() * R.from_quat(sample["target_quat"])).magnitude())
                if sample["target_quat"] is not None else ""
            ),
            "raw_max_joint_step_rad": raw_step,
            "purpose_max_joint_step_rad": purpose_step,
            "sim_q_measured": json.dumps(controller.sim.get_q()[:7].tolist()),
        }
        for joint, value in enumerate(raw, 1):
            row[f"q_ik_raw_{joint}"] = value
        for joint, value in enumerate(purpose, 1):
            row[f"q_purpose_{joint}"] = value
        for joint, value in enumerate(velocity, 1):
            row[f"dq_purpose_{joint}"] = value
        for joint, value in enumerate(acceleration, 1):
            row[f"ddq_purpose_{joint}"] = value
        writer.writerow(row)

        controller.sim.set_ctrl(controller.sim_q())
        nsteps = max(1, int(round(step_dt / controller.sim.model.opt.timestep)))
        loop_start = time.monotonic()
        controller.sim.step(nsteps)
        if realtime:
            time.sleep(max(0.0, step_dt - (time.monotonic() - loop_start)))
        previous_raw = raw
        previous_purpose = purpose
        previous_stamp = sample["stamp_ns"]
        t += step_dt

    def stats(values):
        return {
            "p50": float(np.percentile(values, 50)) if values else 0.0,
            "p95": float(np.percentile(values, 95)) if values else 0.0,
            "max": float(np.max(values)) if values else 0.0,
        }

    return {
        "ik_dt_s": ik_dt,
        "samples": len(targets),
        "duration_s": t,
        "raw_max_joint_step_rad": stats(raw_steps),
        "purpose_max_joint_step_rad": stats(purpose_steps),
        "raw_target_pos_error_m": stats(raw_pos_errors),
        "purpose_target_pos_error_m": stats(purpose_pos_errors),
        "shaper_speed_saturated_fraction": saturated / len(targets),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path, help="Recorded sim MCAP containing ee_intent")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--ik-dt-values", nargs="+", type=float, default=[0.02, 1.0 / 30.0, 0.05])
    parser.add_argument("--max-joint-speed", type=float, default=1.0)
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()

    episode = args.episode.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not episode.is_file():
        raise SystemExit(f"MCAP episode not found: {episode}")
    if args.max_joint_speed <= 0.0 or any(dt <= 0.0 for dt in args.ik_dt_values):
        raise SystemExit("speed limit and IK dt values must be positive")
    metadata, trace = load_longest_intent_trace(episode)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_csvs = [output_dir / f"ik_dt_{dt:.6f}.csv" for dt in args.ik_dt_values]
    summary_path = output_dir / "summary.json"
    collisions = [path for path in [*output_csvs, summary_path] if path.exists()]
    if collisions:
        raise SystemExit(f"Refusing to overwrite existing output(s): {collisions}")

    config = R1ArmConfig(trajectory_max_speed_rad_s=args.max_joint_speed)
    velocity_limits_enabled = bool(
        (config.ik_qp_velocity_limit is not None and config.ik_qp_velocity_limit > 0.0)
        or config.ik_joint_velocity_limits
    )
    if not velocity_limits_enabled:
        print(
            "[IK sweep] NOTE: Placo joint-velocity limits are disabled in this config; "
            "solver.dt is not expected to change the raw IK solution. This run checks "
            "the installed solver behavior, not a tuned dt optimum."
        )
    controller = R1ArmMujocoTeleopController(
        config=config, mock_xr=True, show_viewer=not args.no_viewer, sim_substeps=1,
    )
    try:
        source_rate = float(metadata.get("config", {}).get("record_frequency_hz", 30.0))
        controller.rate_hz = source_rate
        targets = aligned_targets(controller, trace)
        median_dt = float(np.median(np.diff([s["stamp_ns"] for s in targets])) * 1e-9)
        print(
            f"[IK sweep] source={episode} segment={trace[0]['source_idx']}.."
            f"{trace[-1]['source_idx']} samples={len(trace)} "
            f"duration≈{median_dt * len(trace):.1f}s"
        )
        fields = [
            "ik_dt_s", "source_idx", "time_s", "step_dt_s", "ik_solver",
            "target_x_m", "target_y_m", "target_z_m",
            "raw_target_pos_error_m", "purpose_target_pos_error_m",
            "raw_target_rot_error_rad", "purpose_target_rot_error_rad",
            "raw_max_joint_step_rad", "purpose_max_joint_step_rad", "sim_q_measured",
            *[f"q_ik_raw_{j}" for j in range(1, 8)],
            *[f"q_purpose_{j}" for j in range(1, 8)],
            *[f"dq_purpose_{j}" for j in range(1, 8)],
            *[f"ddq_purpose_{j}" for j in range(1, 8)],
        ]
        summaries = []
        for ik_dt, csv_path in zip(args.ik_dt_values, output_csvs):
            print(f"[IK sweep] replaying same trace with solver dt={ik_dt:g}s")
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                result = replay_pass(
                    controller, targets, float(ik_dt), writer,
                    realtime=not args.no_viewer,
                )
            result["csv"] = str(csv_path)
            summaries.append(result)
            print(json.dumps(result, ensure_ascii=False))
        summary = {
            "source_episode": str(episode),
            "source_metadata": metadata.get("config", {}),
            "source_segment": [trace[0]["source_idx"], trace[-1]["source_idx"]],
            "trajectory_position_gain": config.trajectory_position_gain,
            "trajectory_velocity_gain": config.trajectory_velocity_gain,
            "trajectory_max_speed_rad_s": args.max_joint_speed,
            "ik_max_acc_per_s2": config.ik_max_acc_per_s2,
            "ik_max_jerk_per_s3": config.ik_max_jerk_per_s3,
            "placo_velocity_limits_enabled": velocity_limits_enabled,
            "results": summaries,
        }
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"[IK sweep] summary saved: {summary_path}")
    finally:
        controller._on_stop()


if __name__ == "__main__":
    main()
