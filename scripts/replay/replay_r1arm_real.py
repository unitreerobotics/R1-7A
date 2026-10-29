#!/usr/bin/env python3
"""Replay recorded R1Arm state trajectories on the real arm.

Reads ``actions.q_purpose`` (the already-shaped action) or measured
``states.joint_state`` from an MCAP episode written by ``scripts/teleop.py --record``.
By default, the action trajectory is replayed without another trajectory
shaper. The baseline replays the recorded shaped command directly.

The arm is driven in FOC position mode through ``R1ArmSdkBus`` with SDK-side
slew, lead, and joint-limit guards. Frames use recorded ``sample_time_ns``;
``--section action`` replays the shaped action directly, with no second v/a
shaper. A per-run CSV records targets and motor feedback.

The 7 arm joints are always replayed. With ``--enable-dex1`` the recorded Dex1
gripper value (index 7, a sim-space position) is also converted back to a closed
ratio and commanded through DDS, so grasp motions are reproduced too.

Safety: ``--motion-mode`` is required to touch the arm. Without it the script
only loads and reports the trajectory (dry run). Keep the emergency stop
reachable and support the arm during startup probing.
"""

from __future__ import annotations

import argparse
import os
import base64
import csv
import json
import sys
import time
from pathlib import Path

# The OpenCV wheel in r1_teleop ships the xcb Qt plugin, not the Wayland one.
if os.environ.get("QT_QPA_PLATFORM") == "wayland":
    os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mcap.reader import make_reader

from robot.eef.dex1_single import Dex1Single
from robot.r1arm_config import (
    ARM_JOINTS, DEFAULT_URDF_PATH, DEX1_SIM_CLOSE, DEX1_SIM_OPEN, sim_to_closed_ratio,
)
from robot.arm_model import load_robot_model
from robot.r1arm_sdk import (
    DEFAULT_BRIDGE_LIBRARY,
    DEFAULT_SDK_ROOT,
    R1ArmSdkBus,
    format_gain,
    model_to_sdk_q,
    parse_gain,
    sdk_to_model_q,
    validate_calibration_file,
)
from teleop.r1arm_real_controller import (
    MODEL_LOWER,
    MODEL_UPPER,
    limit_sdk_position_command,
)
from teleop.replay_preview import ReplayPreview

JOINT_COUNT = 7
PREVIEW_WINDOW = "R1Arm real replay observations"


def _episode_id(path: Path) -> int:
    try:
        return int(path.stem.split("_")[1])
    except (IndexError, ValueError):
        return -1


def resolve_episode(target: Path) -> Path:
    target = target.expanduser().resolve()
    if target.is_dir():
        episodes = sorted(target.glob("episode_*.mcap"), key=_episode_id)
        if not episodes:
            raise SystemExit(f"No episode_*.mcap found in {target}")
        return episodes[-1]
    if not target.is_file():
        raise SystemExit(f"Episode not found: {target}")
    return target


def load_episode_states(
    path: Path, section_mode: str = "state", data_source: str = "auto"
) -> tuple[list[np.ndarray], list[dict], dict, dict, list[dict]]:
    """Return ordered 8-DoF vectors (arm 7 + gripper 1), raw frames, metadata.

    ``section_mode="state"`` reads measured ``states.joint_state``;
    ``"action"`` prefers the shaped ``actions.q_purpose`` trajectory.
    """
    frames: list[dict] = []
    control_samples: list[dict] = []
    info: dict = {}
    config: dict = {}
    with path.open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic not in (
                "/episode/meta", "/whole_body/frame", "/r1arm/control_sample"
            ):
                continue
            payload = json.loads(message.data.decode("utf-8"))
            if channel.topic == "/episode/meta":
                info = payload.get("info", {})
                config = payload.get("config", {})
            elif channel.topic == "/whole_body/frame":
                frames.append(payload)
            else:
                control_samples.append(payload)
    observation_frames = sorted(frames, key=lambda item: int(item.get("idx", 0)))
    if data_source not in {"auto", "frame", "control"}:
        raise SystemExit(f"Unknown replay data source: {data_source}")
    selected_source = (
        "control" if data_source == "auto" and control_samples else
        "frame" if data_source == "auto" else data_source
    )
    if selected_source == "control":
        if not control_samples:
            raise SystemExit("MCAP has no /r1arm/control_sample trajectory")
        frames = []
        for payload in sorted(control_samples, key=lambda item: int(item.get("control_seq", 0))):
            q_action = np.asarray(payload.get("q_purpose", []), dtype=float).reshape(-1)
            q_state = np.asarray(payload.get("q_measured", []), dtype=float).reshape(-1)
            if q_action.size < JOINT_COUNT or q_state.size < JOINT_COUNT:
                continue
            command_ratio = payload.get("dex1_command_open_ratio")
            state_ratio = payload.get("dex1_state_open_ratio")
            if command_ratio is None:
                command_ratio = 1.0
            if state_ratio is None or not payload.get("dex1_state_valid", False):
                state_ratio = command_ratio
            to_sim = lambda ratio: DEX1_SIM_CLOSE + float(np.clip(ratio, 0.0, 1.0)) * (
                DEX1_SIM_OPEN - DEX1_SIM_CLOSE
            )
            actions = dict(payload)
            actions["q_purpose"] = (q_action.tolist() if q_action.size == 8 else
                                     np.r_[q_action[:JOINT_COUNT], to_sim(command_ratio)].tolist())
            states = {
                "joint_state": (q_state.tolist() if q_state.size == 8 else
                                np.r_[q_state[:JOINT_COUNT], to_sim(state_ratio)].tolist()),
                "dq": payload.get("dq_measured", []),
                "tau_est": payload.get("tau_est", []),
                "control_seq": payload.get("control_seq"),
                "feedback_time_ns": payload.get("feedback_time_ns"),
            }
            frames.append({
                "idx": int(payload.get("control_seq", len(frames))),
                "sample_time_ns": int(payload.get("control_time_ns", 0)),
                "record_segment": int(payload.get("record_segment", 0)),
                "episode_id": payload.get("episode_id"),
                "states": states,
                "actions": actions,
            })
        if not frames:
            raise SystemExit("MCAP control topic contains no complete 7-joint samples")
        info = dict(info)
        info["frequency_hz"] = info.get("control_frequency_hz", info.get("frequency_hz", 30.0))
    info = dict(info)
    info["replay_data_source"] = selected_source
    if not frames:
        raise SystemExit("MCAP contains no /whole_body/frame messages")
    frames.sort(key=lambda item: int(item.get("idx", 0)))
    if section_mode == "action":
        section_name, primary_key, fallback_key = "actions", "q_purpose", "action"
    else:
        section_name, primary_key, fallback_key = "states", "joint_state", "robot_q_current"
    states: list[np.ndarray] = []
    for frame in frames:
        section = frame.get(section_name, {})
        vec = section.get(primary_key, section.get(fallback_key, []))
        if section_mode == "action" and not vec:
            vec = section.get("robot_q_desired", [])
        arr = np.asarray(vec, dtype=float).reshape(-1)
        if arr.size < JOINT_COUNT + 1 or not np.all(np.isfinite(arr[:JOINT_COUNT + 1])):
            raise SystemExit(
                f"frame {frame.get('idx')} {section_name}.{primary_key} must contain "
                f"at least {JOINT_COUNT + 1} finite arm/gripper values"
            )
        states.append(arr.copy())
    return states, frames, info, config, observation_frames


def observation_mosaic(
    frame: dict, index: int, total: int, camera_order: tuple[str, ...]
) -> np.ndarray:
    images = []
    colors = frame.get("colors", {})
    for key in camera_order:
        record = colors.get(key)
        if not record or record.get("encoding") != "jpeg":
            continue
        encoded = base64.b64decode(record["data"])
        image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue
        label = "_".join(key.split("_")[2:]) or key
        cv2.putText(image, f"{label} | {index + 1}/{total}", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, cv2.LINE_AA)
        images.append(image)
    if not images:
        raise ValueError(f"No decodable camera images in observation frame {index}")
    height = min(image.shape[0] for image in images)
    resized = [
        image if image.shape[0] == height else cv2.resize(
            image, (int(image.shape[1] * height / image.shape[0]), height),
            interpolation=cv2.INTER_AREA,
        )
        for image in images
    ]
    columns = 1 if len(resized) == 1 else 2
    while len(resized) % columns:
        resized.append(np.zeros_like(resized[0]))
    return np.vstack([
        np.hstack(resized[offset:offset + columns])
        for offset in range(0, len(resized), columns)
    ])


def recorded_sample_times(states: list[np.ndarray], frames: list[dict]) -> np.ndarray:
    """Per-frame wall-clock sample stamps, or zeros when unavailable."""
    stamps = np.zeros(len(states), dtype=float)
    if frames:
        for i, frame in enumerate(frames):
            stamp_ns = int(frame.get("sample_time_ns", 0) or 0)
            stamps[i] = stamp_ns / 1e9 if stamp_ns > 0 else 0.0
    return stamps


def plan_frame_intervals(
    stamps: np.ndarray, fallback_hz: float, max_gap_s: float
) -> np.ndarray:
    """Seconds to wait before each frame, from the recorded timestamps.

    Timestamp gaps shorter than ``max_gap_s`` reproduce the recorded pacing
    exactly (a mid-recording pause longer than the cap is compressed to it);
    zeros or gaps shorter than one control tick fall back to ``1/fallback_hz``.
    """
    tick = 1.0 / max(float(fallback_hz), 1.0)
    intervals = np.full(len(stamps), tick, dtype=float)
    for i in range(1, len(stamps)):
        if stamps[i] <= 0 or stamps[i - 1] <= 0:
            continue
        delta = stamps[i] - stamps[i - 1]
        if delta < tick * 0.5:
            continue  # jitter tighter than the control tick; keep the uniform tick
        intervals[i] = min(delta, max_gap_s)
    return intervals


def recorded_peak_speed(arm_traj: np.ndarray, intervals: np.ndarray) -> float:
    """Peak per-joint speed implied by the recorded trajectory itself."""
    if len(arm_traj) < 2 or np.all(intervals <= 0):
        return 0.0
    speeds = np.abs(np.diff(arm_traj, axis=0)) / np.maximum(intervals[1:], 1e-6)[:, None]
    return float(np.max(speeds)) if speeds.size else 0.0




def interp_trajectory(arm_traj: np.ndarray, gripper_traj: np.ndarray,
                       intervals: np.ndarray, control_hz: float,
                       speed: float = 1.0,
                       segment_ids: np.ndarray | None = None
                       ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Resample a paced trajectory onto a control-rate grid with interpolation.

    Within each recorded segment the trajectory is linear in time, so commands
    are issued at wall times ``k / control_hz`` plus the exact segment end
    (a partial tick). This preserves the segment wall duration
    (``intervals / speed``) and the implied joint speeds exactly, never
    commands faster than ``control_hz``, and every recorded frame's pose is
    still commanded (segment endpoints are kept). A 50 Hz recording at
    ``control_hz=100`` halves the per-cycle step: same trajectory, same
    timing, smoother commutation.

    Returns the resampled arm/gripper trajectories and the per-command
    timeline gaps (index 0 is a filler, matching ``intervals`` convention) —
    feed them back to the play loop's ``/speed`` scheduler unchanged.
    """
    control_hz = max(float(control_hz), 1.0)
    speed = max(float(speed), 1e-6)
    tick_t = 1.0 / control_hz * speed  # timeline units per control tick
    n = len(arm_traj)
    if n == 0 or len(intervals) < n:
        return arm_traj, gripper_traj, intervals
    out_arm = [arm_traj[0]]
    out_grip = [float(gripper_traj[0])]
    out_gaps = [0.0]
    t_cursor = 0.0  # timeline time of the last emitted command
    for i in range(n - 1):
        seg_t = float(intervals[i + 1])  # timeline duration of this segment
        q_a, q_b = arm_traj[i], arm_traj[i + 1]
        g_a, g_b = float(gripper_traj[i]), float(gripper_traj[i + 1])
        seg_start = t_cursor
        seg_end = seg_start + seg_t
        if segment_ids is not None and segment_ids[i] != segment_ids[i + 1]:
            # Pause/resume is a discontinuity: wait while refreshing the hold,
            # then command the next recorded point without synthetic blending.
            out_arm.append(q_b.copy())
            out_grip.append(g_b)
            out_gaps.append(seg_t)
            t_cursor = seg_end
            continue
        # interior commands at the uniform control tick; stop when within half
        # a tick of the segment end (the exact endpoint follows immediately —
        # a duplicate micro-step buys nothing)
        k = 1
        while seg_start + k * tick_t < seg_end - 0.5 * tick_t:
            frac = (k * tick_t) / seg_t if seg_t > 0 else 1.0
            out_arm.append(q_a + (q_b - q_a) * frac)
            out_grip.append(g_a + (g_b - g_a) * frac)
            out_gaps.append(tick_t)  # one control tick since the previous command
            k += 1
        # exact segment endpoint (always emitted: every recorded frame is commanded)
        out_arm.append(q_b.copy())
        out_grip.append(g_b)
        out_gaps.append(seg_end - (seg_start + (k - 1) * tick_t))
        t_cursor = seg_end
    return (
        np.asarray(out_arm, dtype=float),
        np.asarray(out_grip, dtype=float),
        np.asarray(out_gaps, dtype=float),
    )


def check_feedback_range(state) -> np.ndarray:
    if not np.all(state.ready):
        missing = np.flatnonzero(~state.ready).tolist()
        raise RuntimeError(f"R1Arm feedback incomplete; missing joints {missing}")
    if not np.all(np.isfinite(state.q)):
        raise RuntimeError("R1Arm feedback contains non-finite joint positions")
    model_q = sdk_to_model_q(state.q)
    tolerance = 0.10
    invalid = np.flatnonzero(
        (model_q < MODEL_LOWER - tolerance) | (model_q > MODEL_UPPER + tolerance)
    )
    if invalid.size:
        detail = ", ".join(f"{ARM_JOINTS[i]}={model_q[i]:+.3f}" for i in invalid)
        raise RuntimeError(
            "Feedback is inconsistent with the simulation URDF limits "
            f"({detail}). Check calibration and the J4 convention before enabling motion."
        )
    return model_q


def check_faults(state, require_foc: bool) -> None:
    if np.any(state.timeout):
        raise RuntimeError("R1Arm motor feedback timeout")
    faults = np.flatnonzero(state.error_code != 0)
    if faults.size:
        detail = ", ".join(f"J{i + 1}=0x{int(state.error_code[i]):x}" for i in faults)
        raise RuntimeError(f"R1Arm motor fault: {detail}")
    if require_foc:
        stopped = np.flatnonzero(state.mode != 1)
        if stopped.size:
            joints = ", ".join(f"J{i + 1}" for i in stopped)
            raise RuntimeError(f"R1Arm left FOC mode: {joints}")


def wait_for_feedback(bus: R1ArmSdkBus, rate: float, timeout: float, check_state=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        bus.probe_once()
        time.sleep(1.0 / rate)
        state = bus.read_state()
        if np.all(state.ready):
            (check_state or check_feedback_range)(state)
            check_faults(state, require_foc=False)
            return state
    state = bus.read_state()
    missing = np.flatnonzero(~state.ready).tolist()
    raise TimeoutError(
        f"No complete R1Arm feedback within {timeout:.1f}s; missing joints {missing}"
    )


def enable_foc_hold(
    bus: R1ArmSdkBus, command_sdk: np.ndarray, kp: "float | np.ndarray",
    kd: "float | np.ndarray", rate: float, timeout: float,
    tau_fn=None, check_state=None,
):
    deadline = time.monotonic() + timeout
    zero_dq = np.zeros(JOINT_COUNT, dtype=float)
    while time.monotonic() < deadline:
        tau = tau_fn(command_sdk) if tau_fn is not None else None
        _command_position_v3(bus, command_sdk, zero_dq, kp, kd, tau)
        time.sleep(1.0 / rate)
        state = bus.read_state()
        (check_state or check_feedback_range)(state)
        check_faults(state, require_foc=False)
        if np.all(state.mode == 1):
            return state
    state = bus.read_state()
    stopped = np.flatnonzero(state.mode != 1).tolist()
    raise TimeoutError(f"Joints did not enter FOC within {timeout:.1f}s: {stopped}")


def _command_position_v3(bus, q, dq, kp, kd, tau=None) -> None:
    """Keep every replay position command on the same q/dq SDK path."""
    bus.command_position(q, kp, kd, tau=tau, dq=dq)


def gravity_feedforward(model_pin, data_pin, q_arm: np.ndarray, clip: float) -> np.ndarray:
    """Gravity torque at ``q_arm`` (model frame) → SDK frame, clipped.

    与 goto/trajectory_test/real controller 同一套：URDF nq=9（7 臂 + 2 手指），
    只取前 7 项；J4 符号镜像与位置一致；clip 防模型失配下发无界力矩。
    """
    import pinocchio as pin

    q_full = np.zeros(model_pin.nq, dtype=float)
    q_full[: len(ARM_JOINTS)] = np.asarray(q_arm, dtype=float).reshape(-1)
    tau = pin.computeGeneralizedGravity(model_pin, data_pin, q_full)
    tau = np.asarray(tau, dtype=float).reshape(-1)[: len(ARM_JOINTS)]
    return np.clip(model_to_sdk_q(tau), -clip, clip)


def safe_stop(bus: R1ArmSdkBus, rate: float) -> None:
    try:
        if bus.is_open:
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                bus.stop()
                time.sleep(1.0 / rate)
    except Exception as exc:
        print(f"Warning: R1Arm stop failed: {exc}")
    finally:
        try:
            bus.close()
        except Exception as exc:
            print(f"Warning: R1Arm close failed: {exc}")


def confirm_motion(args: argparse.Namespace, calibration: Path) -> None:
    print("\nREAL R1Arm MOTION IS ABOUT TO BE ENABLED (replay)")
    print(f"  episode:      {args.episode}")
    print(f"  serial:       {args.port} @ {args.baud}")
    print(f"  calibration:  {calibration}")
    print(f"  gains:        kp={format_gain(args.kp)}, kd={format_gain(args.kd)}, "
          f"gravity_ff={'on' if args.gravity_ff is not False else 'off'}"
          f"{'' if args.gravity_ff is None else ' (user)'}")
    print("  replay shaping: off (q_purpose is already shaped)")
    print(f"  max speed:    {args.max_joint_speed if args.max_joint_speed is not None else 'auto (peak x 1.2, capped)'}")
    print(f"  approach speed: {args.approach_speed} rad/s")
    print(f"  max lead:     {args.max_command_lead} rad")
    print(f"  telemetry:    {args.telemetry_csv or 'auto-generated beside the episode'}")
    print("The arm will move through the recorded trajectory.")
    print("Keep the emergency stop reachable and support the arm during startup probing.")
    if input("Type MOVE to continue: ").strip() != "MOVE":
        raise SystemExit("Cancelled; no real motor command was sent")


def run(args: argparse.Namespace, calibration: Path) -> None:
    episode = resolve_episode(Path(args.episode))
    states, frames, info, config, observation_frames = load_episode_states(
        episode, args.section, data_source=args.data_source
    )
    camera_order = tuple(info.get("camera_order", ()))
    if not camera_order and observation_frames:
        camera_order = tuple(observation_frames[0].get("colors", {}).keys())
    observation_stamps = np.asarray([
        int(frame.get("sample_time_ns", 0) or 0) for frame in observation_frames
    ], dtype=np.int64)
    if args.show_observations and not observation_frames:
        print("[Replay] WARNING: episode has no camera frames; preview disabled")
        args.show_observations = False

    state_dim = int(config.get("state_dim", 8))
    if state_dim != 8:
        raise SystemExit(f"Replay supports single-arm episodes (state_dim=8); got {state_dim}")
    recorded_hz = float(info.get("frequency_hz", 30.0))
    freq = args.frequency if args.frequency > 0 else recorded_hz

    start = max(0, int(args.start_frame))
    end = len(states) if args.end_frame < 0 else min(len(states), int(args.end_frame) + 1)
    if start >= end:
        raise SystemExit("Selected frame range is empty")
    states = states[start:end]
    frames = frames[start:end]
    segment_ids = np.asarray([int(frame.get("record_segment", 0)) for frame in frames])
    arm_traj = np.array([s[:JOINT_COUNT] for s in states], dtype=float)
    gripper_traj = np.array([s[JOINT_COUNT] for s in states], dtype=float)

    # ③ Pace by recorded timestamps (pauses replay as pauses, capped).
    stamps = recorded_sample_times(states, frames)
    intervals = plan_frame_intervals(stamps, freq, args.max_frame_gap)
    paced = bool(np.any(intervals != 1.0 / freq))
    shaper_position_gain = float(config.get("trajectory_position_gain",64.0))
    shaper_velocity_gain = float(config.get("trajectory_velocity_gain",16.0))

    # ② Command-rate upsampling: resample the paced trajectory onto a
    # control-rate grid (e.g. a 50 Hz recording driven at 100 Hz with linear
    # interpolation). The motors then see roughly half the step per cycle at
    # double the rate, which smooths commutation noise without altering the
    # trajectory, its timing, or the recorded data. Segment wall duration
    # (intervals / --speed) and implied joint speeds are preserved exactly;
    # every recorded frame's pose is still commanded. The returned gaps stay
    # on the timeline so the play loop's /speed scheduling keeps working.
    control_hz = 0.0
    if args.control_hz > 0 and args.speed > 0:
        control_hz = float(args.control_hz)
        n_before = len(arm_traj)
        arm_traj, gripper_traj, intervals = interp_trajectory(
            arm_traj, gripper_traj, intervals, control_hz,
            speed=float(args.speed), segment_ids=segment_ids,
        )
        # play_delta is per control cycle now
        freq = control_hz * float(args.speed)
        paced = False
        print(
            f"[Replay] interpolated {n_before} recorded frames onto a "
            f"{control_hz:.0f} Hz control grid ({len(arm_traj)} commands)"
        )

    # ① The slew cap must dominate the recorded peak speed, otherwise fast
    # segments are rate-limited away and the command lags the trajectory.
    peak_speed = recorded_peak_speed(arm_traj, intervals)
    teleop_speed = float(config.get("teleop_max_joint_speed", 0.0) or 0.0)
    if args.max_joint_speed is None:
        if args.section == "action" and teleop_speed > 0:
            # q_purpose is already trajectory-shaped; this is only a final
            # SDK-side safety slew cap during replay.
            max_joint_speed = teleop_speed
            speed_source = f"recorded teleop slew limit {teleop_speed:.2f}"
            if peak_speed > teleop_speed:
                print(
                    f"[Replay] sampled q_purpose speed peak {peak_speed:.2f} rad/s exceeds "
                    f"the replay SDK safety slew cap {teleop_speed:.2f} rad/s; affected "
                    f"commands may be clipped (see telemetry CSV)"
                )
        else:
            # Auto = recorded peak x 1.2 so fast segments are not rate-limited
            # away, but hard-capped: IK transients (10+ rad/s) are not a speed
            # anyone should command the real arm at — replaying them at full
            # rate caused transient-undervoltage faults (0x8) on J3-J7.
            max_joint_speed = min(max(peak_speed * 1.2, 1.0), args.auto_speed_cap)
            speed_source = f"auto = peak {peak_speed:.2f} x 1.2, capped at {args.auto_speed_cap:.2f}"
            if peak_speed > args.auto_speed_cap:
                print(
                    f"[Replay] NOTE: recorded peak {peak_speed:.2f} rad/s exceeds the "
                    f"safety cap {args.auto_speed_cap:.2f} rad/s; fast segments will "
                    f"lag the recording instead of reproducing IK transients"
                )
            elif max_joint_speed <= 1.0:
                speed_source = "auto (peak below default; using 1.0)"
            if args.section == "action":
                print(
                    f"[Replay] episode predates teleop_max_joint_speed metadata; "
                    f"falling back to peak-based cap"
                )
    else:
        max_joint_speed = float(args.max_joint_speed)
        speed_source = "user-set"
        if peak_speed > max_joint_speed:
            print(
                f"[Replay] WARNING: --max-joint-speed {max_joint_speed:.2f} is below the "
                f"recorded peak {peak_speed:.2f} rad/s; fast segments will lag"
            )

    joint_names = config.get("joint_names") or list(ARM_JOINTS) + ["dex1_gripper_joint"]
    # FF 默认跟随录制：episode 记录 teleop_gravity_ff（新录制全 True），
    # 旧 episode 无字段时也默认开（P0 三重实测有效无害）。
    use_ff = args.gravity_ff if args.gravity_ff is not None else bool(
        config.get("teleop_gravity_ff", True))
    print(f"[Replay] episode    = {episode}")
    print(f"[Replay] data source= {info.get('replay_data_source', args.data_source)}")
    print(f"[Replay] section    = {args.section} "
          f"({'measured poses (gravity sag included)' if args.section == 'state' else 'teleop commands (sag-free)'})")
    print(f"[Replay] frames     = {len(arm_traj)} (recorded range {start}..{end - 1})")
    print(f"[Replay] joints     = {list(joint_names[:JOINT_COUNT])}")
    print(f"[Replay] recorded_hz= {recorded_hz:.1f}  replay_hz={freq:.1f}  speed={args.speed:.2f}")
    print(f"[Replay] peak speed = {peak_speed:.2f} rad/s; slew cap {max_joint_speed:.2f} ({speed_source})")
    shaping_summary = (
        ("off (q_purpose is already shaped)")
    )
    print(f"[Replay] gravity_ff = {'on' if use_ff else 'off'}"
          f"{' (episode meta)' if args.gravity_ff is None else ' (user)'}"
          f"; replay shaping = {shaping_summary}")
    print(f"[Replay] pacing     = {'recorded sample_time_ns' if paced else 'uniform frame interval'}"
          f" (gap cap {args.max_frame_gap:g}s)")
    print(f"[Replay] command path= SDK v3 q+dq for all position phases; absolute deadlines, "
          f"intersample refresh ≤{freq:.1f} Hz")
    print(f"[Replay] start state= {np.array2string(arm_traj[0], precision=4)}")
    print(f"[Replay] end   state= {np.array2string(arm_traj[-1], precision=4)}")
    print(f"[Replay] gripper   = start {gripper_traj[0]:+.4f} -> end {gripper_traj[-1]:+.4f} "
          f"(commanded only with --enable-dex1)")

    out_of_limits = np.any(
        (arm_traj < MODEL_LOWER - 0.05) | (arm_traj > MODEL_UPPER + 0.05), axis=0
    )
    if np.any(out_of_limits):
        bad = [ARM_JOINTS[i] for i in range(JOINT_COUNT) if out_of_limits[i]]
        print(f"[Replay] WARNING: recorded states exceed model limits for {bad}; they will be clipped")

    if not args.motion_mode:
        print("\nDRY RUN: no serial opened, no motor command sent.")
        print("Add --motion-mode to actually command the arm (keep the e-stop reachable).")
        return

    preview_worker = None
    bus: R1ArmSdkBus | None = None
    hand: Dex1Single | None = None
    command_sdk: np.ndarray | None = None
    telemetry_stream = None
    # FF 模型（同 goto/trajectory_test 的加载与用法）
    gravity: tuple | None = None
    if use_ff:
        model_pin, data_pin = load_robot_model(str(DEFAULT_URDF_PATH))
        gravity = (model_pin, data_pin, float(args.tau_clip))

        def tau_ff(cmd_sdk: np.ndarray) -> np.ndarray | None:
            return gravity_feedforward(model_pin, data_pin, sdk_to_model_q(cmd_sdk), float(args.tau_clip))
    else:
        def tau_ff(cmd_sdk: np.ndarray) -> np.ndarray | None:
            return None

    telemetry_path = (
        Path(args.telemetry_csv).expanduser().resolve()
        if args.telemetry_csv else episode.with_name(
            f"{episode.stem}_replay_{time.time_ns()}.csv"
        )
    )
    if telemetry_path.exists():
        raise SystemExit(f"Refusing to overwrite replay telemetry: {telemetry_path}")
    telemetry_path.parent.mkdir(parents=True, exist_ok=True)
    telemetry_stream = telemetry_path.open("x", newline="", encoding="utf-8")
    telemetry_fields = [
        "phase", "frame_index", "source_frame_idx", "sample_time_ns", "feedback_time_ns",
        "feedback_read_start_ns", "feedback_read_end_ns",
        "feedback_read_start_monotonic_ns", "feedback_read_end_monotonic_ns",
        "command_time_ns", "command_dt_s", "kp", "kd", "gravity_ff",
        "trajectory_position_gain", "trajectory_velocity_gain",
        "trajectory_max_speed_rad_s", "q_ik_raw_rad",
        "max_joint_speed_rad_s", "max_command_lead_rad", "replay_speed",
        "control_seq", "record_segment", "q_purpose_rad", "dq_purpose_rad_s", "q_motor_target_rad",
        "dq_target_rad_s", "sdk_limiter_delta_rad", "q_measured_rad",
        "dq_measured_rad_s", "tau_est_nm", "purpose_tracking_error_rad",
        "motor_tracking_error_rad",
    ]
    telemetry = csv.DictWriter(telemetry_stream, fieldnames=telemetry_fields)
    telemetry.writeheader()

    try:
        if args.show_observations and observation_frames and camera_order:
            preview_worker = ReplayPreview(
                episode, camera_order, scale=args.preview_scale, backend=args.preview_backend,
                window="R1Arm real replay observations",
            )
            preview_worker.update(int(observation_stamps[0]))
        if args.side == 'left':
            from dual.controller import LeftArmBus
            from dual.config import LEFT_BUS_SIGN
            bus = LeftArmBus(args.bridge_lib, sign=config.get('left_bus_sign', LEFT_BUS_SIGN))
        else:
            bus = R1ArmSdkBus(args.bridge_lib)
        bus.open(args.port, args.baud, calibration)

        print("Probing all 7 joints with zero-gain FOC; keep the arm supported...")
        state = wait_for_feedback(bus, args.rate, args.feedback_timeout)
        model_q = check_feedback_range(state)
        check_faults(state, require_foc=False)
        command_sdk = state.q.copy()
        print("Seeding command from measured pose:")
        print("  model q =", np.array2string(model_q, precision=3))
        state = enable_foc_hold(bus, command_sdk, args.kp, args.kd, args.rate, args.foc_timeout,
                                tau_fn=(tau_ff if use_ff else None))
        check_feedback_range(state)
        print("All 7 joints are in FOC and holding the measured startup pose")

        if args.enable_dex1:
            hand = Dex1Single(
                side=args.dex1_side,
                interface=args.dex1_net,
                domain=args.dex1_domain,
                grip_torque_limit=args.grip_torque_limit,
            )

        def command_gripper(gripper_sim: float) -> None:
            if hand is not None:
                hand.set_closed_ratio(sim_to_closed_ratio(float(gripper_sim)))

        # Approaching (dangling arm -> home -> trajectory endpoints) moves all
        # 7 joints simultaneously from a standstill. That current surge is
        # exactly what tripped the transient-undervoltage fault (0x8) when it
        # inherited the play slew cap. It gets its own conservative speed, and
        # never inherits an aggressive play cap.
        approach_speed = min(args.approach_speed, max_joint_speed)
        if approach_speed < args.approach_speed:
            print(
                f"  approach speed {approach_speed:.2f} rad/s (clamped to the "
                f"play slew cap {max_joint_speed:.2f})"
            )

        def approach(target_model: np.ndarray, label: str) -> np.ndarray:
            """Slew from wherever the arm is onto ``target_model``."""
            nonlocal command_sdk
            target_sdk = model_to_sdk_q(target_model)
            deadline = time.monotonic() + args.approach_timeout
            last_model = None
            last_tick = time.monotonic()
            while time.monotonic() < deadline:
                tick_now = time.monotonic()
                approach_dt = float(np.clip(tick_now - last_tick, 1e-4, 0.1))
                last_tick = tick_now
                st = bus.read_state()
                last_model = check_feedback_range(st)
                check_faults(st, require_foc=True)
                cmd = limit_sdk_position_command(
                    target_sdk, command_sdk, st.q,
                    max_delta=approach_speed * approach_dt,
                    max_lead=args.max_command_lead,
                )
                command_mono = time.monotonic()
                command_dt = approach_dt
                dq_target_sdk = (cmd - command_sdk) / command_dt
                _command_position_v3(bus, cmd, dq_target_sdk, args.kp, args.kd, tau_ff(cmd))
                command_time_ns = time.time_ns()
                measured = sdk_to_model_q(st.q)
                target = np.asarray(target_model, dtype=float)
                telemetry.writerow({
                    "phase": f"approach_{label}", "frame_index": "",
                    "source_frame_idx": "", "sample_time_ns": "",
                    "feedback_time_ns": command_time_ns,
                    "command_time_ns": command_time_ns, "command_dt_s": command_dt,
                    "kp": format_gain(args.kp), "kd": format_gain(args.kd),
                    "gravity_ff": int(use_ff), "max_joint_speed_rad_s": max_joint_speed,
                    "max_command_lead_rad": args.max_command_lead,
                    "replay_speed": args.speed,
                    "q_purpose_rad": json.dumps(target.tolist()),
                    "q_motor_target_rad": json.dumps(sdk_to_model_q(cmd).tolist()),
                    "dq_target_rad_s": json.dumps(sdk_to_model_q(dq_target_sdk).tolist()),
                    "q_measured_rad": json.dumps(measured.tolist()),
                    "purpose_tracking_error_rad": json.dumps((target - measured).tolist()),
                    "motor_tracking_error_rad": json.dumps((sdk_to_model_q(cmd) - measured).tolist()),
                })
                telemetry_stream.flush()
                command_sdk = cmd
                last_tick = command_mono
                if np.max(np.abs(last_model - target_model)) < args.approach_tol:
                    return last_model
                time.sleep(1.0 / args.rate)
            err = float(np.max(np.abs(last_model - target_model))) if last_model is not None else float("inf")
            raise TimeoutError(
                f"Replay approach to {label} failed after {args.approach_timeout:.1f}s; "
                f"max joint error={err:.3f} rad (tolerance {args.approach_tol:.3f})"
            )

        print(f"Approaching first recorded state ({np.array2string(arm_traj[0], precision=3)})...")
        model_q = approach(arm_traj[0], "start")
        print(f"  reached start pose (max err {np.max(np.abs(model_q - arm_traj[0])):.3f} rad)")
        command_gripper(gripper_traj[0])

        last_command_mono = time.monotonic()
        print(f"[Replay] telemetry CSV: {telemetry_path}")
        print(f"Playing {len(arm_traj)} frames at {freq:.1f} Hz x {args.speed:.2f} speed...")
        replay_start_ns = int(stamps[0] * 1e9) if len(stamps) and stamps[0] > 0 else 0
        if preview_worker is not None:
            print(f"[Replay] camera preview process: {list(camera_order)} (q/Esc stops replay)")
        while True:
            tick_origin = time.monotonic()
            for i, model_target in enumerate(arm_traj):
                target_sdk = model_to_sdk_q(model_target)
                feedback_read_start_ns = time.time_ns()
                feedback_read_start_mono = time.monotonic_ns()
                st = bus.read_state()
                feedback_read_end_ns = time.time_ns()
                feedback_read_end_mono = time.monotonic_ns()
                feedback_time_ns = feedback_read_end_ns
                model_q = check_feedback_range(st)
                check_faults(st, require_foc=True)
                command_mono = time.monotonic()
                command_dt = float(np.clip(command_mono - last_command_mono, 1e-4, 0.1))
                cmd = limit_sdk_position_command(
                    target_sdk, command_sdk, st.q,
                    max_delta=max_joint_speed * command_dt,
                    max_lead=args.max_command_lead,
                )
                dq_target_sdk = (cmd - command_sdk) / command_dt
                _command_position_v3(bus, cmd, dq_target_sdk, args.kp, args.kd, tau_ff(cmd))
                command_time_ns = time.time_ns()
                measured_model_q = sdk_to_model_q(st.q)
                measured_model_dq = sdk_to_model_q(st.dq)
                command_model_q = sdk_to_model_q(cmd)
                dq_target_model = sdk_to_model_q(dq_target_sdk)
                source_frame = frames[i] if control_hz <= 0.0 and i < len(frames) else {}
                telemetry.writerow({
                    "phase": "playback",
                    "frame_index": i,
                    "source_frame_idx": source_frame.get("idx", ""),
                    "sample_time_ns": source_frame.get("sample_time_ns", ""),
                    "feedback_time_ns": feedback_time_ns,
                    "feedback_read_start_ns": feedback_read_start_ns,
                    "feedback_read_end_ns": feedback_read_end_ns,
                    "feedback_read_start_monotonic_ns": feedback_read_start_mono,
                    "feedback_read_end_monotonic_ns": feedback_read_end_mono,
                    "command_time_ns": command_time_ns,
                    "command_dt_s": command_dt,
                    "kp": format_gain(args.kp), "kd": format_gain(args.kd),
                    "gravity_ff": int(use_ff), "max_joint_speed_rad_s": max_joint_speed,
                    "trajectory_position_gain": shaper_position_gain,
                    "trajectory_velocity_gain": shaper_velocity_gain,
                    "trajectory_max_speed_rad_s": "",
                    "q_ik_raw_rad": json.dumps(
                        source_frame.get("actions", {}).get("q_ik_raw", [])
                    ),
                    "max_command_lead_rad": args.max_command_lead,
                    "replay_speed": args.speed,
                    "control_seq": source_frame.get("actions", {}).get("control_seq", ""),
                    "record_segment": source_frame.get("record_segment", ""),
                    "q_purpose_rad": json.dumps(model_target.tolist()),
                    "dq_purpose_rad_s": json.dumps(
                        source_frame.get("actions", {}).get("dq_purpose", [])
                    ),
                    "q_motor_target_rad": json.dumps(command_model_q.tolist()),
                    "dq_target_rad_s": json.dumps(dq_target_model.tolist()),
                    "sdk_limiter_delta_rad": json.dumps((command_model_q - model_target).tolist()),
                    "q_measured_rad": json.dumps(measured_model_q.tolist()),
                    "dq_measured_rad_s": json.dumps(measured_model_dq.tolist()),
                    "tau_est_nm": json.dumps(sdk_to_model_q(st.tau_est).tolist()),
                    "purpose_tracking_error_rad": json.dumps((model_target - measured_model_q).tolist()),
                    "motor_tracking_error_rad": json.dumps((command_model_q - measured_model_q).tolist()),
                })
                if i % 30 == 0:
                    telemetry_stream.flush()
                last_command_mono = command_mono
                command_sdk = cmd
                command_gripper(gripper_traj[i])
                if args.print_actions:
                    print(f'[Action] frame={i} side={args.side} '
                          f'q_motor_target_rad={json.dumps(command_model_q.tolist())} '
                          f'gripper_slide={float(gripper_traj[i]):.6f}', flush=True)
                if preview_worker is not None:
                    replay_elapsed = float(np.sum(intervals[1 : i + 1]))
                    replay_stamp_ns = replay_start_ns + int(replay_elapsed * 1e9)
                    camera_index = max(0, int(np.searchsorted(
                        observation_stamps, replay_stamp_ns, side="right"
                    ) - 1))
                    if preview_worker.update(observation_stamps[camera_index]):
                        raise KeyboardInterrupt
                if args.debug and (i % 10 == 0 or i == len(arm_traj) - 1):
                    print(
                        f"  frame {i}/{len(arm_traj) - 1} "
                        f"target={np.array2string(model_target, precision=2)} "
                        f"measured={np.array2string(model_q, precision=2)}"
                    )
                # ③ Schedule against a running clock so control latency inside
                # a tick does not accumulate into drift; frame 0 starts right
                # after the approach, later frames wait the recorded gap.
                if i + 1 < len(arm_traj):
                    target_time = tick_origin + np.sum(intervals[1 : i + 2]) / max(float(args.speed), 1e-6)
                    remaining = target_time - time.monotonic()
                else:
                    remaining = 0.0
                if remaining > 0:
                    hold_tick = 1.0 / max(freq, 1.0)
                    hold_deadline = target_time
                    next_hold_time = time.monotonic() + hold_tick
                    while time.monotonic() < hold_deadline:
                        sleep_s = next_hold_time - time.monotonic()
                        if sleep_s > 0:
                            time.sleep(min(sleep_s, max(0.0, hold_deadline - time.monotonic())))
                        if time.monotonic() >= hold_deadline:
                            break
                        hold_state = bus.read_state()
                        check_feedback_range(hold_state)
                        check_faults(hold_state, require_foc=True)
                        _command_position_v3(
                            bus, command_sdk, np.zeros(JOINT_COUNT), args.kp, args.kd,
                            tau_ff(command_sdk),
                        )
                        next_hold_time += hold_tick
                        # Do not send catch-up bursts if feedback/serial work overran a tick.
                        if next_hold_time <= time.monotonic():
                            next_hold_time = time.monotonic() + hold_tick
            # ② Converge onto the final recorded state (the play loop's last
            # command may lag it after fast segments) before holding it.
            print("Converging onto the final recorded state...")
            model_q = approach(arm_traj[-1], "end")
            print(f"  final pose error {np.max(np.abs(model_q - arm_traj[-1])):.3f} rad")
            if not args.loop:
                break
            print("Looping: re-approaching start state...")
            model_q = approach(arm_traj[0], "start")
            command_gripper(gripper_traj[0])

        print(f"Holding last pose for {args.hold_time:.1f}s...")
        hold_until = time.monotonic() + args.hold_time
        while time.monotonic() < hold_until:
            feedback_read_start_ns = time.time_ns()
            feedback_read_start_mono = time.monotonic_ns()
            st = bus.read_state()
            feedback_read_end_ns = time.time_ns()
            feedback_read_end_mono = time.monotonic_ns()
            check_feedback_range(st)
            check_faults(st, require_foc=True)
            _command_position_v3(
                bus, command_sdk, np.zeros(JOINT_COUNT), args.kp, args.kd,
                tau_ff(command_sdk),
            )
            measured = sdk_to_model_q(st.q)
            target = np.asarray(arm_traj[-1], dtype=float)
            motor_target = sdk_to_model_q(command_sdk)
            telemetry.writerow({
                "phase": "hold", "frame_index": "", "source_frame_idx": "",
                "sample_time_ns": "", "feedback_time_ns": feedback_read_end_ns,
                "feedback_read_start_ns": feedback_read_start_ns,
                "feedback_read_end_ns": feedback_read_end_ns,
                "feedback_read_start_monotonic_ns": feedback_read_start_mono,
                "feedback_read_end_monotonic_ns": feedback_read_end_mono,
                "command_time_ns": time.time_ns(), "kp": format_gain(args.kp),
                "kd": format_gain(args.kd), "gravity_ff": int(use_ff),
                "max_joint_speed_rad_s": max_joint_speed,
                "max_command_lead_rad": args.max_command_lead,
                "replay_speed": args.speed,
                "record_segment": "",
                "q_purpose_rad": json.dumps(target.tolist()),
                "q_motor_target_rad": json.dumps(motor_target.tolist()),
                "q_measured_rad": json.dumps(measured.tolist()),
                "purpose_tracking_error_rad": json.dumps((target - measured).tolist()),
                "motor_tracking_error_rad": json.dumps((motor_target - measured).tolist()),
            })
            telemetry_stream.flush()
            time.sleep(1.0 / args.rate)
        print("[Replay] finished; releasing arm")
    except KeyboardInterrupt:
        print("\n[Replay] interrupted")
    finally:
        if hand is not None:
            try:
                hand.stop()
            except Exception as exc:
                print(f"Warning: Dex1 stop failed: {exc}")
        if bus is not None:
            safe_stop(bus, args.rate)
        if telemetry_stream is not None and not telemetry_stream.closed:
            telemetry_stream.close()
        if preview_worker is not None:
            preview_worker.close()
        print("[Replay] stopped")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--episode",
        required=True,
        help="Path to an episode_XXXX.mcap file, or a directory (uses the latest episode)",
    )
    parser.add_argument(
        "--data-source", choices=("auto", "control", "frame"), default="auto",
        help="trajectory topic: auto prefers full-rate /r1arm/control_sample, then low-rate frames",
    )
    parser.add_argument(
        "--section",
        choices=("state", "action"),
        default="action",
        help=(
            "Replay 'states.joint_state' (measured poses, gravity sag included) "
            "or 'actions.q_purpose' (the already-shaped teleop joint action)"
        ),
    )
    parser.add_argument(
        "--motion-mode", action="store_true", help="Actually open and command the arm"
    )
    parser.add_argument("--port", default="")
    parser.add_argument("--side", choices=('right', 'left'), help='Defaults to the recorded arm side')
    parser.add_argument("--baud", type=int, default=6_000_000)
    parser.add_argument("--sdk-root", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--calib", type=Path, help="joint calibration file")
    parser.add_argument("--bridge-lib", type=Path, default=DEFAULT_BRIDGE_LIBRARY)
    parser.add_argument("--kp", type=parse_gain, default=60.0,
                        help="位置环增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--kd", type=parse_gain, default=10.0,
                        help="阻尼增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--rate", type=float, default=30.0,
                        help="approach/final-hold refresh Hz; playback timing follows recorded "
                             "timestamps unless --control-hz is set")
    parser.add_argument(
        "--control-hz", type=float, default=0.0,
        help="command rate for the play loop; 0 = replay at the recorded frame rate. "
             "A higher rate (e.g. 100) linearly interpolates between recorded frames, "
             "halving the per-cycle step (softer motor commutation) without changing "
             "the trajectory or its pacing",
    )
    parser.add_argument(
        "--frequency", type=float, default=0.0, help="replay Hz; 0 uses the recorded frequency_hz"
    )
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=1.0,
        help="rad/s command slew cap for playback (default 1.0)",
    )
    parser.add_argument(
        "--auto-speed-cap",
        type=float,
        default=5.0,
        help="rad/s hard ceiling on the auto slew cap; IK transients above this "
        "are replayed slower instead of at full speed",
    )
    parser.add_argument(
        "--approach-speed",
        type=float,
        default=1.0,
        help="rad/s joint speed for approach moves (dangling arm -> start pose, "
        "end convergence, loop restarts); conservative by design",
    )
    parser.add_argument(
        "--max-frame-gap",
        type=float,
        default=2.0,
        help="s; longest recorded pause reproduced before being compressed",
    )
    parser.add_argument(
        "--max-command-lead", type=float, default=0.15,
        help="rad；命令相对实测的最大超前量（异常单拍位移 ≤8.6°，力矩三项"
             "合计 ≈ 电机峰值一半；正常回放不触发）",
    )
    parser.add_argument(
        "--gravity-ff", dest="gravity_ff", action="store_true", default=None,
        help="下发重力前馈（指令姿态处计算，经 v2 tau 通道）。默认跟随 episode "
             "元数据 teleop_gravity_ff——录制开了回放就自动开；旧 episode 无该"
             "字段时默认开",
    )
    parser.add_argument(
        "--no-gravity-ff", dest="gravity_ff", action="store_false",
        help="关闭重力前馈（回到 kp 硬扛悬垂的旧行为，声音会明显变大）",
    )
    parser.add_argument(
        "--tau-clip", type=float, default=8.0,
        help="重力前馈单关节力矩限幅 Nm（默认 8）",
    )
    parser.add_argument(
        "--telemetry-csv", type=Path,
        help="per-replay telemetry CSV (default: unique file beside the source episode)",
    )
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--foc-timeout", type=float, default=1.0)
    parser.add_argument(
        "--enable-dex1",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Replay the Dex1 gripper (state index 7) through DDS as a closed ratio",
    )
    parser.add_argument("--dex1-net", default=os.environ.get("DEX1_NETWORK", ""), help="DDS interface for the single Dex1")
    parser.add_argument("--dex1-side", choices=("left", "right"), default=None,
                        help="DDS side topic for the physical Dex1")
    parser.add_argument("--dex1-domain", type=int, default=0)
    parser.add_argument("--grip-torque-limit", type=float, default=-1.0)
    parser.add_argument("--approach-timeout", type=float, default=10.0)
    parser.add_argument("--approach-tol", type=float, default=0.05, help="rad, max joint error to start")
    parser.add_argument("--hold-time", type=float, default=2.0, help="s to hold the last pose")
    parser.add_argument("--loop", action="store_true", help="repeat the trajectory until Ctrl+C")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=-1)
    parser.add_argument("--show-observations", action=argparse.BooleanOptionalAction, default=True,
                        help="show recorded camera views by timestamp (default; use --no-show-observations to disable)")
    parser.add_argument('--preview-backend', choices=('browser', 'window'), default='browser',
                        help='browser: localhost preview for VS Code Simple Browser; window: OpenCV')
    parser.add_argument('--print-actions', action=argparse.BooleanOptionalAction, default=True,
                        help='Print each playback command after sending; --no-print-actions disables output')
    parser.add_argument("--preview-scale", type=float, default=0.75)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--no-confirm", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    episode = resolve_episode(Path(args.episode))
    recorded_side = 'right'
    with episode.open('rb') as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic == '/episode/meta':
                recorded_side = json.loads(message.data).get('config', {}).get('control_side', 'right')
                break
    if recorded_side not in ('right', 'left') or (args.side and args.side != recorded_side):
        raise SystemExit('Replay arm side must match the recorded control_side')
    args.side = recorded_side
    if args.dex1_side and args.dex1_side != args.side:
        raise SystemExit('Gripper side must match the recorded arm side')
    args.dex1_side = args.side
    from dual.config import PORTS, validate_arm_port
    args.port = args.port or os.environ.get('PORT_'+args.side.upper()) or str(PORTS[args.side])
    validate_arm_port(args.port, args.side)
    if args.motion_mode and not args.port:
        raise SystemExit("Set --port or PORT to the verified arm serial device")
    if args.motion_mode and args.enable_dex1 and not args.dex1_net:
        raise SystemExit("Set --dex1-net or DEX1_NETWORK to the DDS interface")
    if args.rate <= 0 or args.speed <= 0:
        raise SystemExit("--rate and --speed must be positive")
    if args.motion_mode and not args.bridge_lib.expanduser().is_file():
        raise SystemExit(
            f"bridge library not found: {args.bridge_lib}; "
            "run python scripts/build_r1arm_sdk_bridge.py first"
        )
    if args.motion_mode and not Path(args.port).exists():
        raise SystemExit(f"serial port not found: {args.port}")

    calibration = args.calib or args.sdk_root / f"config/joint_calib_zero_{'右' if args.side == 'right' else '左'}臂.txt"
    if args.motion_mode:
        try:
            calibration = validate_calibration_file(calibration)
        except (OSError, ValueError) as exc:
            raise SystemExit(str(exc))
        if not args.no_confirm:
            confirm_motion(args, calibration)

    run(args, Path(calibration).expanduser().resolve())


if __name__ == "__main__":
    main()
