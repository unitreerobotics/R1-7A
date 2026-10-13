#!/usr/bin/env python3
"""Replay an R1Arm MCAP episode in MuJoCo."""

from __future__ import annotations

import argparse
import base64
import json
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from mcap.reader import make_reader


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.mujoco_sim import MujocoArmSim
from robot.r1arm_config import (
    ARM_JOINTS,
    DEFAULT_DUAL_MJCF_PATH,
    DEX1_JOINTS,
    LEFT_ARM_JOINTS,
    LEFT_DEX1_JOINTS,
    R1ArmConfig,
)
from teleop.r1arm_dataset import R1ArmDatasetAdapter, validate_r1arm_frame


PREVIEW_WINDOW = "R1Arm MCAP observations"


def load_episode(path: Path) -> tuple[list[dict[str, Any]], dict, dict, dict]:
    if not path.is_file():
        raise FileNotFoundError(f"MCAP file not found: {path}")
    frames: list[dict[str, Any]] = []
    info: dict[str, Any] = {}
    text: dict[str, Any] = {}
    config: dict[str, Any] = {}
    with path.open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic not in ("/episode/meta", "/whole_body/frame"):
                continue
            payload = json.loads(message.data.decode("utf-8"))
            if channel.topic == "/episode/meta":
                info = payload.get("info", {})
                text = payload.get("text", {})
                config = payload.get("config", {})
            else:
                frames.append(payload)
    if not frames:
        raise ValueError("MCAP contains no /whole_body/frame messages")
    frames.sort(key=lambda item: int(item.get("idx", 0)))
    expected_dof = int(config.get("action_dim", 16 if config.get("dual_arm", False) else 8))
    camera_order = tuple(info.get("camera_order") or frames[0].get("colors", {}).keys())
    # An empty camera_order is a valid state/action-only episode.
    for frame in frames:
        validate_r1arm_frame(
            frame,
            expected_dof=expected_dof,
            expected_camera_keys=camera_order,
        )
    return frames, info, text, config


def decode_observation(record: dict[str, Any]) -> np.ndarray:
    encoded = record.get("data")
    if record.get("encoding") != "jpeg" or not isinstance(encoded, str):
        raise ValueError("camera observation is not a base64 JPEG")
    image = cv2.imdecode(
        np.frombuffer(base64.b64decode(encoded), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    if image is None:
        raise ValueError("failed to decode camera observation")
    return image


def observation_mosaic(
    frame: dict[str, Any], index: int, total: int, camera_order: tuple[str, ...]
) -> np.ndarray:
    images = []
    for key in camera_order:
        parts = key.split("_")
        label = "_".join(parts[2:]) if len(parts) > 2 else key
        image = decode_observation(frame["colors"][key])
        cv2.putText(
            image,
            f"{label} | {index + 1}/{total}",
            (10, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        images.append(image)
    columns = 1 if len(images) == 1 else 2
    while len(images) % columns:
        images.append(np.zeros_like(images[0]))
    rows = [
        np.hstack(images[offset : offset + columns])
        for offset in range(0, len(images), columns)
    ]
    return np.vstack(rows)


def frame_period(frames: list[dict[str, Any]], index: int, fallback_hz: float) -> float:
    if index + 1 < len(frames):
        current = int(frames[index].get("sample_time_ns", 0))
        following = int(frames[index + 1].get("sample_time_ns", 0))
        delta = (following - current) / 1e9
        if 1e-4 <= delta <= 1.0:
            return delta
    return 1.0 / max(float(fallback_hz), 1.0)


def replay(args: argparse.Namespace) -> None:
    episode = Path(args.episode).expanduser().resolve()
    frames, info, text, metadata = load_episode(episode)
    recorded_dual = bool(metadata.get("dual_arm", False))
    dual_arm = recorded_dual if args.dual_arm is None else bool(args.dual_arm)
    control_side = args.side or str(metadata.get("control_side", "right"))
    adapter = R1ArmDatasetAdapter(dual_arm=dual_arm, control_side=control_side)
    recorded_names = metadata.get("joint_names")
    if recorded_names is not None and list(recorded_names) != list(adapter.joint_names):
        raise ValueError(
            "MCAP joint order does not match this R1Arm simulation: "
            f"recorded={recorded_names}, expected={list(adapter.joint_names)}"
        )

    start = max(0, int(args.start_frame))
    end = len(frames) if args.end_frame < 0 else min(len(frames), args.end_frame + 1)
    if start >= end:
        raise SystemExit("Selected frame range is empty")

    config = R1ArmConfig(dual_arm=dual_arm, control_side=control_side)
    recorded_mjcf = str(metadata.get("mjcf_path", ""))
    if args.mjcf:
        config.mjcf_path = args.mjcf
    elif recorded_mjcf and Path(recorded_mjcf).is_file():
        config.mjcf_path = recorded_mjcf
    elif dual_arm:
        config.mjcf_path = str(DEFAULT_DUAL_MJCF_PATH)

    joint_names = ARM_JOINTS + DEX1_JOINTS
    if dual_arm:
        joint_names += LEFT_ARM_JOINTS + LEFT_DEX1_JOINTS
    initial_state = adapter.sim_q_from_frame(frames[start], mode="state")
    sim = MujocoArmSim(
        config.mjcf_path,
        q_init=initial_state,
        show_viewer=not args.no_viewer,
        joint_names=joint_names,
    )

    recorded_hz = float(info.get("frequency_hz", 30.0))
    camera_order = tuple(info.get("camera_order") or frames[0]["colors"].keys())
    if args.show_observations and not camera_order:
        print("[Replay] WARNING: episode has no camera observations; skipping display")
        args.show_observations = False
    hz = args.hz if args.hz > 0 else recorded_hz
    print(f"[MCAP] {episode}")
    print(f"[MCAP] frames={len(frames)} goal={text.get('goal', '')!r}")
    print(
        f"[Replay] MuJoCo mode={args.mode} dual_arm={dual_arm} "
        f"side={control_side} hz={hz:.2f} speed={args.speed:.2f}"
    )

    index = start
    try:
        while True:
            tick = time.monotonic()
            frame = frames[index]
            sim_q = adapter.sim_q_from_frame(frame, mode=args.mode)
            period = frame_period(frames, index, hz)
            if args.mode == "state":
                # Exact observation replay: place the plant at recorded q.
                sim.set_q(sim_q, also_ctrl=True)
                sim.sync()
            else:
                # Action replay: drive the same position actuators as teleoperation.
                sim.set_ctrl(sim_q)
                substeps = max(1, int(round(period / float(sim.model.opt.timestep))))
                sim.step(substeps)

            if args.show_observations:
                display = observation_mosaic(frame, index, len(frames), camera_order)
                if args.preview_scale != 1.0:
                    display = cv2.resize(
                        display,
                        None,
                        fx=args.preview_scale,
                        fy=args.preview_scale,
                        interpolation=cv2.INTER_LINEAR,
                    )
                cv2.imshow(PREVIEW_WINDOW, display)
                if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                    break

            index += 1
            if index >= end:
                if not args.loop:
                    break
                index = start
                sim.set_q(initial_state, also_ctrl=True)

            remaining = period / max(float(args.speed), 1e-6) - (time.monotonic() - tick)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if args.show_observations:
            cv2.destroyAllWindows()
        final_q = sim.get_q()
        sim.close()
        print(f"[Replay] finished at frame {index}; final_sim_q={np.round(final_q, 4)}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True)
    parser.add_argument("--mode", choices=("action", "state"), default="action")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--hz", type=float, default=0.0, help="0 uses MCAP frequency_hz")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=-1)
    parser.add_argument("--mjcf", default="")
    parser.add_argument("--side", choices=("left", "right"), default=None)
    arms = parser.add_mutually_exclusive_group()
    arms.add_argument("--use-dual-arm", dest="dual_arm", action="store_true")
    arms.add_argument("--single-arm", dest="dual_arm", action="store_false")
    parser.set_defaults(dual_arm=None)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--show-observations", action="store_true")
    parser.add_argument("--preview-scale", type=float, default=0.75)
    return parser


def main() -> None:
    replay(build_parser().parse_args())


if __name__ == "__main__":
    main()
