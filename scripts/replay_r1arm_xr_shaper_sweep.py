#!/usr/bin/env python3
"""Replay recorded XR poses through XR filtering, teleop mapping, IK and shaper."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.arm_model import link_pose
from robot.r1arm_config import R1ArmConfig
from teleop.r1arm_mujoco_controller import R1ArmMujocoTeleopController


PARAMETERS = {
    "speed": ("trajectory_max_speed_rad_s", float),
    "kp": ("trajectory_position_gain", float),
    "kd": ("trajectory_velocity_gain", float),
    "acc": ("ik_max_acc_per_s2", float),
    "jerk": ("ik_max_jerk_per_s3", float),
    "cutoff": ("xr_filter_min_cutoff", float),
    "beta": ("xr_filter_beta", float),
}


def load_episode(path: Path) -> tuple[dict, list[dict]]:
    from mcap.reader import make_reader

    metadata = {}
    frames = []
    with path.open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            frame = json.loads(message.data)
            if channel.topic == "/episode/meta":
                metadata = frame
            elif channel.topic == "/whole_body/frame":
                actions = frame.get("actions", {})
                if not {"xr_poses_raw", "teleop_anchor", "xr_filter_state",
                        "xr_poses_filtered", "xr_poses_jump_rejected"}.issubset(actions):
                    raise ValueError(
                        "Episode lacks full XR replay fields; record a new simulation episode "
                        "with the updated scripts/record_r1arm_sim.py"
                    )
                item = {
                    "idx": int(frame.get("idx", len(frames))),
                    "sample_time_ns": int(frame.get("sample_time_ns", message.log_time)),
                    "control_seq": actions.get("control_seq"),
                    "control_time_ns": actions.get("control_time_ns"),
                    "xr_frame_stamp_ns": actions.get("xr_frame_stamp_ns"),
                    "xr_valid": bool(actions.get("xr_valid", False)),
                    "grip_active": bool(actions.get("grip_active", False)),
                    "xr_poses_raw": actions.get("xr_poses_raw") or {},
                    "xr_poses_filtered": actions.get("xr_poses_filtered") or {},
                    "xr_poses_jump_rejected": actions.get("xr_poses_jump_rejected") or {},
                    "xr_filter_state": actions.get("xr_filter_state") or {},
                    "xr_jump_reject_count": int(actions.get("xr_jump_reject_count", 0)),
                    "teleop_anchor": actions.get("teleop_anchor") or {},
                    "ee_intent": actions.get("ee_intent") or {},
                    "joint_state": frame.get("states", {}).get("joint_state", []),
                }
                if item["control_seq"] is not None and frames and frames[-1]["control_seq"] == item["control_seq"]:
                    frames[-1] = item
                else:
                    frames.append(item)
    if len(frames) < 2:
        raise ValueError("Episode must contain at least two recorded control samples")
    source_config = metadata.get("config", {})
    side = str(source_config.get("control_side", metadata.get("control_side", "right")))
    required_poses = {"headset", "left_controller", "right_controller"}
    valid_frames = [frame for frame in frames if frame["xr_valid"]]
    if not valid_frames or not any(f"{side}_controller" in frame["xr_poses_raw"] for frame in valid_frames):
        raise ValueError("Episode contains no valid XR samples for its selected teleop side")
    if any(not required_poses.issubset(frame["xr_poses_raw"]) for frame in valid_frames):
        raise ValueError("A valid XR sample is missing headset or controller pose data")
    return metadata, frames


def as_pose_map(poses: dict) -> dict[str, np.ndarray]:
    result = {}
    for name, value in poses.items():
        pose = np.asarray(value, dtype=float).reshape(-1)
        if pose.size == 7 and np.all(np.isfinite(pose)) and np.linalg.norm(pose[3:7]) > 1e-6:
            pose = pose.copy()
            pose[3:7] /= np.linalg.norm(pose[3:7])
            result[name] = pose
    return result


def restore_anchor(track, anchor: dict) -> None:
    def vector(name, size):
        value = anchor.get(name)
        if value is None:
            return None
        array = np.asarray(value, dtype=float).reshape(-1)
        return array.copy() if array.size == size and np.all(np.isfinite(array)) else None

    track.ref_ee_pos = vector("ee_position", 3)
    track.ref_ee_quat = vector("ee_quaternion_xyzw", 4)
    track.ref_controller_pos = vector("controller_position", 3)
    rotation = vector("controller_rotation_xyzw", 4)
    track.ref_controller_rot = R.from_quat(rotation) if rotation is not None else None
    yaw = anchor.get("heading_yaw")
    track.ref_heading_yaw = float(yaw) if yaw is not None else None


def initial_q(controller, frames: list[dict]) -> np.ndarray:
    for frame in frames:
        state = np.asarray(frame["joint_state"], dtype=float).reshape(-1)
        if state.size >= 7 and np.all(np.isfinite(state[:7])):
            q = controller._create_initial_q()
            for index, value in zip(controller.arm_q_indices, state[:7]):
                q[index] = value
            return q
    raise ValueError("Episode has no valid initial seven-joint state")


def prepare_pass(controller, q0: np.ndarray, side: str) -> None:
    controller.q = q0.copy()
    controller.t = 0.0
    controller._reset_trajectory_state(side, controller.q)
    controller._solver_seeds[side] = controller.q.copy()
    controller.track[side].clear()
    controller._last_ee_intent[side] = {}
    controller._last_ik_raw[side] = controller.arm_q(side)
    controller._last_ik_solver[side] = "hold"
    controller.placo_solver.set_regularization_reference(controller.q)
    controller._xr_last_stamp = None
    controller._xr_pose_cache = None
    controller._xr_pose_raw = None
    controller._xr_filter_state = {}
    controller._xr_filter_last_stamp_ns = None
    controller._xr_prev_poses = None
    controller._xr_jump_rejects = 0
    controller.sim._mujoco.mj_resetData(controller.sim.model, controller.sim.data)
    controller.sim.set_q(controller.sim_q(), also_ctrl=True)


def resample_control_frames(frames: list[dict], rate_hz: float) -> list[dict]:
    """Zero-order-hold recorded XR/control state on a new control clock."""
    rate_hz = float(rate_hz)
    if not np.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("control rate must be finite and positive")
    source_ns = [
        int(frame["control_time_ns"] if frame["control_time_ns"] is not None
            else frame["sample_time_ns"])
        for frame in frames
    ]
    if any(later < earlier for earlier, later in zip(source_ns, source_ns[1:])):
        raise ValueError("recorded control timestamps must be monotonic")
    duration_s = (source_ns[-1] - source_ns[0]) * 1e-9
    period_s = 1.0 / rate_hz
    tick_times = list(np.arange(0.0, duration_s, period_s))
    if not tick_times or duration_s - tick_times[-1] > 1e-9:
        tick_times.append(duration_s)
    if len(tick_times) == 1:
        tick_times.append(duration_s + period_s)

    replay = []
    source_index = 0
    previous_t = 0.0
    for tick_index, time_s in enumerate(tick_times):
        target_ns = source_ns[0] + int(round(time_s * 1e9))
        while source_index + 1 < len(frames) and source_ns[source_index + 1] <= target_ns:
            source_index += 1
        item = dict(frames[source_index])
        item["_replay_time_s"] = float(time_s)
        item["_replay_dt_s"] = float(time_s - previous_t) if tick_index else period_s
        replay.append(item)
        previous_t = time_s
    return replay


def stats(values: list[float]) -> dict:
    values = np.asarray(values, dtype=float)
    return {
        "p50": float(np.percentile(values, 50)) if values.size else 0.0,
        "p95": float(np.percentile(values, 95)) if values.size else 0.0,
        "max": float(np.max(values)) if values.size else 0.0,
        "mean": float(np.mean(values)) if values.size else 0.0,
    }


def replay_candidate(controller, frames: list[dict], q0: np.ndarray, writer,
                     *, rate_hz: float, ik_dt_s: float,
                     restore_recorded_anchors: bool) -> dict:
    side = controller.control_side
    controller._replay_grip_active = False
    controller.get_grip_state = lambda _side=None: float(controller._replay_grip_active)
    prepare_pass(controller, q0, side)
    elapsed_s = 0.0
    physics_elapsed_s = 0.0
    seeded_filter = False
    previous_raw = previous_purpose = previous_acc = None
    arrays = {key: [] for key in (
        "raw_step", "purpose_step", "raw_pos_error", "purpose_pos_error",
        "raw_rot_error", "purpose_rot_error", "sim_tracking_error", "speed", "acc", "jerk",
    )}
    saturated = active_samples = invalid_samples = 0
    out_fields = writer.fieldnames

    for index, frame in enumerate(frames):
        control_ns = frame["control_time_ns"]
        dt = float(frame.get("_replay_dt_s", 0.0))
        if dt <= 0.0:
            if control_ns is not None and index > 0:
                previous_ns = frames[index - 1]["control_time_ns"]
                dt = (int(control_ns) - int(previous_ns)) * 1e-9 if previous_ns is not None else 0.0
            if dt <= 0.0:
                dt = 1.0 / max(rate_hz, 1.0)
        if not 1e-4 <= dt <= 0.1:
            dt = 1.0 / max(rate_hz, 1.0)
        controller._step_dt = dt
        controller.t += dt
        for solver in (controller.placo_solver, controller.placo_solver_left):
            if solver is not None:
                solver.control_rate_hz = 1.0 / float(ik_dt_s)
                solver.solver_dt = float(ik_dt_s)
                solver.solver.dt = float(ik_dt_s)

        raw_poses = as_pose_map(frame["xr_poses_raw"]) if frame["xr_valid"] else {}
        controller._xr_pose_raw = raw_poses
        stamp = frame["xr_frame_stamp_ns"]
        if stamp is None:
            stamp = control_ns or frame["sample_time_ns"]
        stamp = int(stamp)
        if frame["xr_valid"]:
            if restore_recorded_anchors and not seeded_filter:
                filtered = as_pose_map(frame["xr_poses_filtered"])
                controller._xr_filter_state = {
                    name: {
                        "pose": np.asarray(state["pose"], dtype=float),
                        "derivative": np.asarray(state["derivative"], dtype=float),
                        "dt": float(state["dt"]),
                        "stamp_ns": int(state["stamp_ns"]),
                    }
                    for name, state in frame["xr_filter_state"].items()
                }
                controller._xr_prev_poses = as_pose_map(frame["xr_poses_jump_rejected"])
                controller._xr_jump_rejects = frame["xr_jump_reject_count"]
                controller._xr_filter_last_stamp_ns = stamp
                controller._xr_pose_cache = filtered
                controller._xr_last_stamp = stamp
                seeded_filter = True
            elif stamp != controller._xr_last_stamp or controller._xr_pose_cache is None:
                filtered = controller._reject_xr_jumps(raw_poses)
                filtered = controller._filter_xr_poses(filtered, stamp)
                controller._xr_pose_cache = filtered
                controller._xr_last_stamp = stamp
            else:
                filtered = controller._xr_pose_cache
        else:
            filtered = {}
            invalid_samples += 1

        controller._replay_grip_active = frame["grip_active"]
        if restore_recorded_anchors:
            restore_anchor(controller.track[side], frame["teleop_anchor"])
        controller._track_side(side, filtered)
        raw = controller._last_ik_raw[side].copy()
        purpose, velocity, acceleration = controller._trajectory_state(side, controller.q)
        raw_model = controller.q.copy()
        for qidx, value in zip(controller.arm_q_indices, raw):
            raw_model[qidx] = value
        raw_pos, raw_quat = link_pose(controller.model, controller.data, raw_model, controller.ee_frame)
        purpose_pos, purpose_quat = link_pose(
            controller.model, controller.data, controller.q, controller.ee_frame
        )
        intent = controller._last_ee_intent.get(side, {})
        target_pos = np.asarray(intent.get("position", []), dtype=float).reshape(-1)
        target_quat = np.asarray(intent.get("quaternion_xyzw", []), dtype=float).reshape(-1)
        target_valid = frame["xr_valid"] and frame["grip_active"] and target_pos.size == 3
        replay_delta = None
        recorded_pos = np.asarray(frame["ee_intent"].get("position", []), dtype=float).reshape(-1)
        if recorded_pos.size == 3 and target_pos.size == 3:
            replay_delta = float(np.linalg.norm(target_pos - recorded_pos))

        raw_pos_error = float(np.linalg.norm(raw_pos - target_pos)) if target_valid else ""
        purpose_pos_error = float(np.linalg.norm(purpose_pos - target_pos)) if target_valid else ""
        raw_rot_error = purpose_rot_error = ""
        if target_valid and target_quat.size == 4:
            raw_rot_error = float((R.from_quat(raw_quat).inv() * R.from_quat(target_quat)).magnitude())
            purpose_rot_error = float((R.from_quat(purpose_quat).inv() * R.from_quat(target_quat)).magnitude())
        raw_step = float(np.max(np.abs(raw - previous_raw))) if previous_raw is not None else 0.0
        purpose_step = float(np.max(np.abs(purpose - previous_purpose))) if previous_purpose is not None else 0.0
        speed = float(np.max(np.abs(velocity)))
        accel = float(np.max(np.abs(acceleration)))
        jerk = float(np.max(np.abs(acceleration - previous_acc)) / dt) if previous_acc is not None else 0.0
        if previous_raw is not None:
            arrays["raw_step"].append(raw_step)
            arrays["purpose_step"].append(purpose_step)
        if target_valid:
            active_samples += 1
            arrays["raw_pos_error"].append(raw_pos_error)
            arrays["purpose_pos_error"].append(purpose_pos_error)
            if raw_rot_error != "":
                arrays["raw_rot_error"].append(raw_rot_error)
                arrays["purpose_rot_error"].append(purpose_rot_error)
        if replay_delta is not None:
            arrays.setdefault("replay_intent_mismatch", []).append(replay_delta)
        arrays["speed"].append(speed)
        arrays["acc"].append(accel)
        arrays["jerk"].append(jerk)

        controller.sim.set_ctrl(controller.sim_q())
        sim_dt = float(controller.sim.model.opt.timestep)
        target_physics_steps = int(round((physics_elapsed_s + dt) / sim_dt))
        previous_physics_steps = int(round(physics_elapsed_s / sim_dt))
        nsteps = max(1, target_physics_steps - previous_physics_steps)
        controller.sim.step(nsteps)
        physics_elapsed_s += dt
        measured = controller.sim.get_q()[:7]
        arrays["sim_tracking_error"].append(float(np.max(np.abs(measured - purpose))))

        row = {
            "source_idx": frame["idx"], "control_seq": frame["control_seq"],
            "time_s": float(frame.get("_replay_time_s", elapsed_s)),
            "step_dt_s": dt, "xr_frame_stamp_ns": stamp,
            "xr_valid": frame["xr_valid"], "grip_active": frame["grip_active"],
            "ik_solver": controller._last_ik_solver[side],
            "target_x_m": target_pos[0] if target_valid else "",
            "target_y_m": target_pos[1] if target_valid else "",
            "target_z_m": target_pos[2] if target_valid else "",
            "replay_vs_recorded_intent_m": replay_delta if replay_delta is not None else "",
            "raw_target_pos_error_m": raw_pos_error,
            "purpose_target_pos_error_m": purpose_pos_error,
            "raw_target_rot_error_rad": raw_rot_error,
            "purpose_target_rot_error_rad": purpose_rot_error,
            "raw_max_joint_step_rad": raw_step,
            "purpose_max_joint_step_rad": purpose_step,
            "max_abs_dq_purpose_rad_s": speed,
            "max_abs_ddq_purpose_rad_s2": accel,
            "max_abs_jerk_purpose_rad_s3": jerk,
            "sim_max_joint_tracking_error_rad": arrays["sim_tracking_error"][-1],
            "q_ik_raw": json.dumps(raw.tolist()),
            "q_purpose": json.dumps(purpose.tolist()),
            "q_sim_measured": json.dumps(measured.tolist()),
        }
        writer.writerow({key: row.get(key, "") for key in out_fields})
        previous_raw, previous_purpose, previous_acc = raw, purpose, acceleration
        elapsed_s += dt
        if speed >= 0.98 * float(controller.config.trajectory_max_speed_rad_s):
            saturated += 1

    return {
        "samples": len(frames), "active_samples": active_samples,
        "invalid_xr_samples": invalid_samples,
        "speed_saturated_fraction": saturated / max(len(frames), 1),
        "raw_max_joint_step_rad": stats(arrays["raw_step"]),
        "purpose_max_joint_step_rad": stats(arrays["purpose_step"]),
        "raw_target_pos_error_m": stats(arrays["raw_pos_error"]),
        "purpose_target_pos_error_m": stats(arrays["purpose_pos_error"]),
        "raw_target_rot_error_rad": stats(arrays["raw_rot_error"]),
        "purpose_target_rot_error_rad": stats(arrays["purpose_rot_error"]),
        "sim_joint_tracking_error_rad": stats(arrays["sim_tracking_error"]),
        "max_abs_speed_rad_s": stats(arrays["speed"]),
        "max_abs_acceleration_rad_s2": stats(arrays["acc"]),
        "max_abs_jerk_rad_s3": stats(arrays["jerk"]),
        "replay_intent_reproduction_error_m": stats(arrays.get("replay_intent_mismatch", [])),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    for option, flag in (
        ("speed", "--speed-values"), ("kp", "--kp-values"), ("kd", "--kd-values"),
        ("acc", "--acc-values"), ("jerk", "--jerk-values"),
        ("cutoff", "--cutoff-values"), ("beta", "--beta-values"),
    ):
        parser.add_argument(flag, nargs="+", type=float, default=None,
                            help=f"one-factor-at-a-time {option} candidates")
    parser.add_argument("--rate", type=float, default=0.0,
                        help="fallback control rate; defaults to recorded teleop rate")
    parser.add_argument("--control-rates", nargs="+", type=float, default=None,
                        help="replay the same held XR trace at each control-loop rate (Hz)")
    parser.add_argument("--ik-dt-values", nargs="+", type=float, default=None,
                        help="Placo solver dt values (s), crossed with --control-rates")
    parser.add_argument("--viewer", action="store_true", help="show MuJoCo viewer (headless by default)")
    args = parser.parse_args()

    episode = args.episode.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not episode.is_file():
        raise SystemExit(f"MCAP episode not found: {episode}")
    metadata, frames = load_episode(episode)
    source_config = metadata.get("config", {})
    if bool(source_config.get("dual_arm", metadata.get("dual_arm", False))):
        raise SystemExit("This XR sweep currently supports single-arm simulation episodes only")
    baseline = {
        "speed": float(source_config.get("trajectory_max_speed_rad_s", 1.0)),
        "kp": float(source_config.get("trajectory_position_gain", 20.0)),
        "kd": float(source_config.get("trajectory_velocity_gain", 9.0)),
        "acc": float(source_config.get("ik_max_acc_per_s2", 6.0)),
        "jerk": float(source_config.get("ik_max_jerk_per_s3", 80.0)),
        "cutoff": float(source_config.get("xr_filter_min_cutoff", 8.0)),
        "beta": float(source_config.get("xr_filter_beta", 1.0)),
    }
    default_rate = float(args.rate or source_config.get("teleop_rate_hz", 30.0))
    candidates = [("baseline", {**baseline, "control_rate": default_rate, "ik_dt": 0.0})]
    for short, (field, _) in PARAMETERS.items():
        values = getattr(args, f"{short}_values")
        if values is not None:
            if any(not np.isfinite(value) or value < 0.0 for value in values):
                raise SystemExit(f"{short} values must be finite and non-negative")
            for value in values:
                if short in {"speed", "kp", "kd", "acc", "jerk", "cutoff"} and value <= 0.0:
                    raise SystemExit(f"{short} values must be positive")
                candidate = baseline.copy()
                candidate[short] = float(value)
                candidate["control_rate"] = default_rate
                candidate["ik_dt"] = 0.0
                candidates.append((f"{short}_{value:g}", candidate))
    if args.control_rates is not None or args.ik_dt_values is not None:
        rates = args.control_rates or [default_rate]
        if any(not np.isfinite(value) or value <= 0.0 for value in rates):
            raise SystemExit("control rates must be finite and positive")
        if args.ik_dt_values is None:
            timing_pairs = [(rate, 1.0 / rate) for rate in rates]
        else:
            if any(not np.isfinite(value) or value <= 0.0 for value in args.ik_dt_values):
                raise SystemExit("IK dt values must be finite and positive")
            timing_pairs = [(rate, dt) for rate in rates for dt in args.ik_dt_values]
        for rate, ik_dt in timing_pairs:
            if np.isclose(rate, default_rate) and np.isclose(ik_dt, 1.0 / default_rate):
                continue
            candidate = {**baseline, "control_rate": float(rate), "ik_dt": float(ik_dt)}
            candidates.append((f"rate_{rate:g}_ikdt_{ik_dt:g}", candidate))
    if len(candidates) == 1:
        raise SystemExit("Provide parameter candidates or --control-rates/--ik-dt-values")
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    output_paths = [summary_path, *(
        output_dir / f"{label.replace('.', 'p')}.csv" for label, _ in candidates
    )]
    if len(set(output_paths)) != len(output_paths):
        raise SystemExit("Candidate labels produce duplicate output filenames")
    collisions = [path for path in output_paths if path.exists()]
    if collisions:
        raise SystemExit(f"Refusing to overwrite existing outputs: {collisions}")

    rate_hz = default_rate
    config = R1ArmConfig(
        control_side=str(source_config.get("control_side", metadata.get("control_side", "right"))),
        trajectory_max_speed_rad_s=baseline["speed"],
        trajectory_position_gain=baseline["kp"],
        trajectory_velocity_gain=baseline["kd"],
        ik_max_acc_per_s2=baseline["acc"],
        ik_max_jerk_per_s3=baseline["jerk"],
        xr_filter_min_cutoff=baseline["cutoff"],
        xr_filter_beta=baseline["beta"],
    )
    controller = R1ArmMujocoTeleopController(
        config=config, mock_xr=True, show_viewer=args.viewer, sim_substeps=1,
        ik_dt=1.0 / max(rate_hz, 1.0),
    )
    controller.rate_hz = rate_hz
    side = controller.control_side
    q0 = initial_q(controller, frames)
    fields = [
        "source_idx", "control_seq", "time_s", "step_dt_s", "xr_frame_stamp_ns",
        "xr_valid", "grip_active", "ik_solver", "target_x_m", "target_y_m", "target_z_m",
        "replay_vs_recorded_intent_m", "raw_target_pos_error_m", "purpose_target_pos_error_m",
        "raw_target_rot_error_rad", "purpose_target_rot_error_rad", "raw_max_joint_step_rad",
        "purpose_max_joint_step_rad", "max_abs_dq_purpose_rad_s",
        "max_abs_ddq_purpose_rad_s2", "max_abs_jerk_purpose_rad_s3",
        "sim_max_joint_tracking_error_rad", "q_ik_raw", "q_purpose", "q_sim_measured",
    ]
    results = []
    try:
        for label, candidate in candidates:
            for short, (field, _) in PARAMETERS.items():
                setattr(config, field, candidate[short])
            candidate_rate = float(candidate["control_rate"])
            candidate_ik_dt = float(candidate["ik_dt"] or 1.0 / candidate_rate)
            controller.rate_hz = candidate_rate
            replay_frames = resample_control_frames(frames, candidate_rate)
            csv_path = output_dir / f"{label.replace('.', 'p')}.csv"
            if csv_path.exists():
                raise SystemExit(f"Refusing to overwrite existing output: {csv_path}")
            print(f"[XR sweep] {label}: {candidate}")
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                result = replay_candidate(
                    controller, replay_frames, q0, writer,
                    rate_hz=candidate_rate, ik_dt_s=candidate_ik_dt,
                    restore_recorded_anchors=(
                        candidate["cutoff"] == baseline["cutoff"]
                        and candidate["beta"] == baseline["beta"]
                    ),
                )
            results.append({"label": label, "parameters": candidate,
                            "ik_dt_s_effective": candidate_ik_dt,
                            "control_rate_hz": candidate_rate,
                            "csv": str(csv_path), **result})
            print(json.dumps(results[-1], ensure_ascii=False))
        summary_path.write_text(json.dumps({
            "source_episode": str(episode), "source_metadata": source_config,
            "samples_after_control_seq_dedup": len(frames),
            "timing_sweep": bool(args.control_rates or args.ik_dt_values),
            "xr_filter_is_recomputed_from_raw": True,
            "results": results,
        }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"[XR sweep] summary saved: {summary_path}")
    finally:
        controller._on_stop()


if __name__ == "__main__":
    main()
