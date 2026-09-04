#!/usr/bin/env python3
"""Replay and inspect R1-A7 four-camera MCAP recordings.

Default mode is visual-only and never initializes DDS. Real-robot playback
requires both --motion-mode and an explicit confirmation word.

Usage:
    # Validate an episode without a window (also the quickest format check)
    python scripts/replay_mcap.py --episode data/.../episode_0000.mcap --no-video

    # Watch it back
    python scripts/replay_mcap.py --episode data/.../episode_0000.mcap

    # Drive the real robot from the recording (secure the robot first)
    python scripts/replay_mcap.py --episode ... --motion-mode --confirm REPLAY_R1
"""

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

SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.config import DEX1_REAL_CLOSE, DEX1_REAL_OPEN, DEX1_SIM_CLOSE, DEX1_SIM_OPEN
from teleop.mcap_writer import FourCameraMcapRecorder


CAMERA_ORDER = FourCameraMcapRecorder.CAMERA_ORDER
CAMERA_LABELS = ("head_left", "head_right", "wrist_left", "wrist_right")
PREVIEW_WINDOW = "R1-A7 MCAP replay: head L/R, wrist L/R"


def decode_image(record: dict[str, Any]) -> np.ndarray:
    if not isinstance(record, dict) or record.get("encoding") != "jpeg":
        raise ValueError("camera record is not a JPEG object")
    encoded = record.get("data")
    if not isinstance(encoded, str):
        raise ValueError("camera JPEG data is missing")
    frame = cv2.imdecode(np.frombuffer(base64.b64decode(encoded), dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("failed to decode camera JPEG")
    return frame


def load_episode(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    if not path.is_file():
        raise FileNotFoundError(f"MCAP file not found: {path}")

    frames: list[dict[str, Any]] = []
    info: dict[str, Any] = {}
    text: dict[str, Any] = {}
    with path.open("rb") as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic not in ("/episode/meta", "/whole_body/frame"):
                continue
            try:
                payload = json.loads(message.data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError(f"invalid JSON on topic {channel.topic}: {exc}") from exc
            if channel.topic == "/episode/meta":
                info = payload.get("info", {})
                text = payload.get("text", {})
            else:
                frames.append(payload)
    if not frames:
        raise ValueError("MCAP contains no /whole_body/frame messages")
    frames.sort(key=lambda frame: int(frame.get("idx", 0)))
    return frames, info, text


def validate_episode(frames: list[dict[str, Any]], info: dict[str, Any]) -> None:
    expected = list(CAMERA_ORDER)
    metadata_order = info.get("camera_order")
    if metadata_order is not None and metadata_order != expected:
        raise ValueError(f"unexpected camera_order in MCAP: {metadata_order}; expected {expected}")
    colors = frames[0].get("colors", {})
    missing = [key for key in expected if key not in colors]
    if missing:
        raise ValueError(f"first frame is missing required cameras: {missing}")
    for key in expected:
        decode_image(colors[key])


def select_control(frame: dict[str, Any], mode: str) -> tuple[np.ndarray, np.ndarray]:
    section = frame.get("actions" if mode == "action" else "states", {})
    q_key = "robot_q_desired" if mode == "action" else "robot_q_current"
    hand_key = "hand_cmd" if mode == "action" else "hand_state"
    q = np.asarray(section.get(q_key, []), dtype=float).reshape(-1)
    hand = np.asarray(section.get(hand_key, []), dtype=float).reshape(-1)
    if q.size < 18:
        raise ValueError(f"recorded {q_key} has {q.size} values; need body(4) + arms(14)")
    if hand.size < 4:
        raise ValueError(f"recorded {hand_key} has {hand.size} values; need Dex1 left/right values")
    return q, hand


def draw_label(frame: np.ndarray, text: str) -> np.ndarray:
    result = frame.copy()
    cv2.putText(result, text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(result, text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
    return result


def show_frame(frame: dict[str, Any], episode_name: str, index: int, total: int, paused: bool, scale: float) -> int:
    colors = frame["colors"]
    images = [decode_image(colors[key]) for key in CAMERA_ORDER]
    status = "PAUSED" if paused else "PLAY"
    labels = [draw_label(image, f"{label} | {status} | {index + 1}/{total}") for image, label in zip(images, CAMERA_LABELS)]
    top = np.hstack(labels[:2])
    bottom = np.hstack(labels[2:])
    display = np.vstack((top, bottom))
    cv2.putText(display, episode_name, (12, display.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)
    if scale != 1.0:
        display = cv2.resize(display, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    cv2.imshow(PREVIEW_WINDOW, display)
    return cv2.waitKey(1) & 0xFF


class R1A7RealReplay:
    """Direct R1 arm and Dex1 command publisher for an already-recorded trajectory."""

    def __init__(self, net: str, head_pose_file: str) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_

        from robot.config import load_head_pose
        from robot.r1_a7_real import REAL_JOINTS, R1A7LowCmdConfig, R1A7LowCmdTeleop

        self._motor_cmd_type = unitree_go_msg_dds__MotorCmd_
        self._motor_cmds_type = MotorCmds_
        self._publisher_type = ChannelPublisher
        self._real_joints = REAL_JOINTS
        joint_indices = {name: index for index, (name, _, _, _) in enumerate(REAL_JOINTS)}
        config = R1A7LowCmdConfig(interface=net, enable_motion=True, max_joint_delta=0.04, control_dt=0.02)
        self.arm = R1A7LowCmdTeleop(config, joint_indices)
        self.arm.init_dds()
        self.arm.wait_for_lowstate()

        head_pose = load_head_pose(head_pose_file)
        with self.arm.lock:
            self.arm.target_q[joint_indices["head_pitch_joint"]] = float(head_pose[0])
            self.arm.target_q[joint_indices["head_yaw_joint"]] = float(head_pose[1])
        self.arm.start()

        self.left_publisher = self._publisher_type("rt/dex1/left/cmd", MotorCmds_)
        self.right_publisher = self._publisher_type("rt/dex1/right/cmd", MotorCmds_)
        self.left_publisher.Init()
        self.right_publisher.Init()
        self.left_message = self._make_gripper_message()
        self.right_message = self._make_gripper_message()
        print("[Real replay] rt/lowcmd and rt/dex1/{left,right}/cmd publishers are active")

    def _make_gripper_message(self):
        message = self._motor_cmds_type()
        command = self._motor_cmd_type()
        command.dq = 0.0
        command.tau = 0.0
        command.kp = 5.0
        command.kd = 0.05
        message.cmds = [command]
        return message

    @staticmethod
    def _sim_to_real(value: float) -> float:
        ratio = (DEX1_SIM_OPEN - float(value)) / (DEX1_SIM_OPEN - DEX1_SIM_CLOSE)
        ratio = float(np.clip(ratio, 0.0, 1.0))
        return DEX1_REAL_OPEN + (DEX1_REAL_CLOSE - DEX1_REAL_OPEN) * ratio

    def command(self, q: np.ndarray, hand: np.ndarray) -> None:
        arms = q[4:18]
        with self.arm.lock:
            self.arm.target_q[:14] = arms
        self.left_message.cmds[0].q = self._sim_to_real(hand[0])
        self.right_message.cmds[0].q = self._sim_to_real(hand[2])
        self.left_publisher.Write(self.left_message)
        self.right_publisher.Write(self.right_message)

    def stop(self) -> None:
        self.arm.stop()


def replay(args: argparse.Namespace) -> None:
    episode = Path(args.episode).expanduser().resolve()
    frames, info, text = load_episode(episode)
    validate_episode(frames, info)
    goal = text.get("goal", "")
    print(f"[MCAP] {episode}")
    print(f"[MCAP] frames={len(frames)}, goal={goal!r}")
    print(f"[MCAP] camera_order={list(CAMERA_ORDER)}")

    real = None
    if args.no_robot and args.motion_mode:
        print("[Replay] --no-robot overrides --motion-mode; no robot commands will be sent.")
    if args.motion_mode and not args.no_robot:
        if args.confirm != "REPLAY_R1":
            raise SystemExit("Real replay is blocked. Re-run with --motion-mode --confirm REPLAY_R1 after securing the robot.")
        real = R1A7RealReplay(args.net, args.head_pose_file)

    index = max(0, int(args.start_frame))
    end = len(frames) if args.end_frame < 0 else min(len(frames), int(args.end_frame) + 1)
    if index >= end:
        raise SystemExit("Selected frame range is empty")
    paused = False
    frame_period = 1.0 / max(args.hz * args.speed, 1e-6)
    print("[Replay] q: quit, space: pause, left/right: previous/next frame")

    try:
        while True:
            tick_start = time.monotonic()
            frame = frames[index]
            q, hand = select_control(frame, args.mode)
            if real is not None and not paused:
                real.command(q, hand)

            key = -1
            if not args.no_video:
                key = show_frame(frame, episode.name, index, len(frames), paused, args.preview_scale)
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                paused = not paused
                continue
            if key in (81, 2424832):  # OpenCV/X11 left arrow variants
                index = max(int(args.start_frame), index - 1)
                paused = True
                continue
            if key in (83, 2555904):  # OpenCV/X11 right arrow variants
                index = min(end - 1, index + 1)
                paused = True
                continue
            if paused:
                time.sleep(0.01)
                continue

            index += 1
            if index >= end:
                if args.loop:
                    index = max(0, int(args.start_frame))
                else:
                    break
            remaining = frame_period - (time.monotonic() - tick_start)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        cv2.destroyAllWindows()
        if real is not None:
            real.stop()
            print("[Real replay] arm lowcmd publisher stopped")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--episode", required=True, help="Path to episode_XXXX.mcap")
    parser.add_argument("--mode", choices=["action", "state"], default="action")
    parser.add_argument("--hz", type=float, default=30.0, help="Recorded sample frequency")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=-1, help="Inclusive final frame; -1 means final MCAP frame")
    parser.add_argument("--no-video", action="store_true", help="Validate/play commands without an OpenCV window")
    parser.add_argument("--preview-scale", type=float, default=0.75)
    parser.add_argument("--motion-mode", action="store_true", help="Enable real R1 arm and Dex1 command publishing")
    parser.add_argument("--no-robot", "--no_robot", dest="no_robot", action="store_true",
                        help="Disable all real-robot DDS and command publishing")
    parser.add_argument("--confirm", default="", help="Must be exactly REPLAY_R1 when using --motion-mode")
    parser.add_argument("--net", default="enp49s0", help="Local DDS network interface for real replay")
    parser.add_argument("--head-pose-file", default="",
                        help="Fixed R1 head target used only during real replay")
    return parser


def main() -> None:
    replay(build_parser().parse_args())


if __name__ == "__main__":
    main()