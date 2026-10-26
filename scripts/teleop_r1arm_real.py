#!/usr/bin/env python3
"""PICO teleoperation for one standalone real R1Arm (7 DoF + optional Dex1)."""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.r1arm_config import R1ArmConfig
from robot.r1arm_sdk import (
    DEFAULT_BRIDGE_LIBRARY,
    DEFAULT_SDK_ROOT,
    as_gain_array,
    format_gain,
    parse_gain,
    validate_calibration_file,
)
from teleop.r1arm_real_controller import (
    R1ArmRealTeleopController,
)


def confirm_motion(args: argparse.Namespace) -> None:
    print("\nREAL R1Arm MOTION IS ABOUT TO BE ENABLED")
    print(f"  serial:       {args.port} @ {args.baud}")
    print(f"  calibration:  {args.calib}")
    print(f"  gains:        kp={format_gain(args.kp)}, kd={format_gain(args.kd)}, "
          f"gravity_ff={'on' if args.gravity_ff else 'off'}")
    print(f"  max speed:    {args.max_joint_speed} rad/s")
    print(f"  q_purpose follower: kp={args.trajectory_position_gain:g} s^-2, "
          f"kd={args.trajectory_velocity_gain:g} s^-1; "
          f"v≤{(args.trajectory_max_speed or args.max_joint_speed):g} rad/s")
    kp_arr = as_gain_array(args.kp)
    print(f"  max lead:     {args.max_command_lead} rad; nominal kp×lead="
          f"{float(np.max(kp_arr*args.max_command_lead)):.1f} Nm "
          "(not a total torque bound; slew and lead limits may conflict)")
    print(f"  controller:   {args.side} PICO controller")
    print("Keep the emergency stop reachable and support the arm during startup probing.")
    answer = input("Type MOVE to continue: ").strip()
    if answer != "MOVE":
        raise SystemExit("Cancelled; no real motor command was sent")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-mode", action="store_true", help="Actually open and command the arm")
    parser.add_argument(
        "--port",
        default=os.environ.get("PORT", ""),
        help="R1Arm RS485 serial device",
    )
    parser.add_argument("--baud", type=int, default=6_000_000)
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=DEFAULT_SDK_ROOT,
        help="Used to derive the default calibration path",
    )
    parser.add_argument("--calib", type=Path, help="joint calibration file")
    parser.add_argument("--bridge-lib", type=Path, default=DEFAULT_BRIDGE_LIBRARY)
    parser.add_argument("--urdf", type=Path, help="Simulation/IK URDF (same model as the working sim)")
    parser.add_argument("--side", choices=["left", "right"], default="right")
    parser.add_argument("--scale-factor", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=90.0,
                        help="控制环 Hz。默认 90；XR 独立接收，图像采样频率单独配置")
    parser.add_argument("--trajectory-position-gain", type=float, default=64.0,
                        help="Baseline q_purpose position-error response (s^-2)")
    parser.add_argument("--trajectory-velocity-gain", type=float, default=16.0,
                        help="Baseline q_purpose velocity damping (s^-1)")
    parser.add_argument(
        "--ik-dt", type=float, default=0.0,
        help="Placo velocity-limit timestep s; 0 = 1/--rate. With current Placo "
             "joint-velocity limits disabled, changing this does not scale raw pose-IK steps",
    )
    parser.add_argument(
        "--gravity-ff", dest="gravity_ff", action="store_true", default=True,
        help="下发重力前馈力矩（指令姿态处计算，clip 后经 v2 tau 通道）；"
             "默认开（P0 三重实测：J4 悬垂误差 −70%%）。--no-gravity-ff 关闭",
    )
    parser.add_argument(
        "--no-gravity-ff", dest="gravity_ff", action="store_false",
        help="关闭重力前馈（回到 kp 硬扛悬垂的旧行为）",
    )
    parser.add_argument(
        "--tau-clip", type=float, default=8.0,
        help="重力前馈单关节力矩限幅 Nm（默认 8）",
    )
    parser.add_argument("--kp", type=parse_gain, default=60.0,
                        help="位置环增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--kd", type=parse_gain, default=10.0,
                        help="阻尼增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--max-joint-speed", type=float, default=1.0,
                        help="SDK command speed limit (rad/s; default 1.0)")
    parser.add_argument(
        "--trajectory-max-speed", type=float, default=None,
        help="q_purpose shaper speed limit (rad/s; defaults to --max-joint-speed)",
    )
    parser.add_argument(
        "--max-command-lead", type=float, default=0.15,
        help="命令相对实测的目标最大超前量 rad（默认 0.15）；与位置步长冲突时优先步长，不能作为总力矩上界",
    )
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--foc-timeout", type=float, default=1.0)
    parser.add_argument("--enable-dex1", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dex1-net", default=os.environ.get("DEX1_NETWORK", ""), help="DDS interface for the one real Dex1")
    parser.add_argument("--dex1-side", choices=("left", "right"), default=os.environ.get("DEX1_SIDE", "right"),
                        help="DDS side topic for the physical Dex1; independent of the arm side")
    parser.add_argument("--dex1-domain", type=int, default=0)
    parser.add_argument("--grip-torque-limit", type=float, default=-1.0)
    parser.add_argument(
        "--enable-a-reset", action=argparse.BooleanOptionalAction, default=True,
        help="Allow A to move toward ARM_INIT",
    )
    parser.add_argument(
        "--home-on-start",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="After startup FOC hold, soft-blend to ARM_INIT before enabling tracking",
    )
    parser.add_argument(
        "--reset-blend-frames",
        type=int,
        default=60,
        help="A/home reset 时长（控制帧数；blend 按 wall-clock frames/50Hz 计秒，"
             "默认 60 ≈ 1.2s；想要旧 2s 体验传 100）",
    )
    parser.add_argument("--mock-xr", action="store_true", help="Dry-run only; rejected with --motion-mode")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--gripper-log", type=Path, help="Write per-cycle trigger/DDS diagnostics to a new CSV")
    parser.add_argument("--no-confirm", action="store_true")
    return parser


def main(argv=None, *, controller_cls=None, defaults=None) -> None:
    parser = build_parser()
    if defaults:
        parser.set_defaults(**defaults)
    args = parser.parse_args(argv)
    if args.motion_mode and not args.port:
        parser.error("Set --port or PORT to the verified arm serial device")
    if args.motion_mode and args.enable_dex1 and not args.dex1_net:
        parser.error("Set --dex1-net or DEX1_NETWORK to the DDS interface")

    if args.motion_mode and args.mock_xr:
        parser.error("--mock-xr cannot be combined with --motion-mode")
    if args.trajectory_position_gain <= 0.0 or args.trajectory_velocity_gain <= 0.0:
        parser.error("trajectory follower gains must be positive")
    if args.max_joint_speed <= 0.0:
        parser.error("--max-joint-speed must be positive")
    if args.trajectory_max_speed is not None and args.trajectory_max_speed <= 0.0:
        parser.error("--trajectory-max-speed must be positive")
    if args.motion_mode and not args.bridge_lib.expanduser().is_file():
        parser.error(
            f"bridge library not found: {args.bridge_lib}; "
            "run python scripts/build_r1arm_sdk_bridge.py first"
        )
    if args.motion_mode and not Path(args.port).exists():
        parser.error(
            f"serial port not found: {args.port}; check /dev/serial/by-id and /dev/ttyUSB*"
        )

    calibration = args.calib or args.sdk_root / "config/joint_calib_zero_右臂.txt"
    if args.motion_mode:
        try:
            calibration = validate_calibration_file(calibration)
        except (OSError, ValueError) as exc:
            parser.error(str(exc))
        args.calib = calibration
        if not args.no_confirm:
            confirm_motion(args)

    config = R1ArmConfig(
        control_side=args.side,
        scale_factor=args.scale_factor,
        reset_blend_frames=args.reset_blend_frames,
        trajectory_position_gain=args.trajectory_position_gain,
        trajectory_velocity_gain=args.trajectory_velocity_gain,
        trajectory_max_speed_rad_s=(
            args.trajectory_max_speed
            if args.trajectory_max_speed is not None else args.max_joint_speed
        ),
    )
    if args.urdf:
        config.urdf_path = str(args.urdf.expanduser().resolve())

    controller_cls = controller_cls or (
        (R1ArmRealTeleopController)
    )
    controller = controller_cls(
        config=config,
        port=args.port,
        baudrate=args.baud,
        calibration_path=calibration,
        bridge_library=args.bridge_lib,
        motion_mode=args.motion_mode,
        enable_dex1=args.enable_dex1,
        dex1_interface=args.dex1_net,
        dex1_side=args.dex1_side,
        dex1_domain=args.dex1_domain,
        grip_torque_limit=args.grip_torque_limit,
        kp=args.kp,
        kd=args.kd,
        rate=args.rate,
        max_joint_speed=args.max_joint_speed,
        max_command_lead=args.max_command_lead,
        feedback_timeout=args.feedback_timeout,
        foc_timeout=args.foc_timeout,
        home_on_start=args.home_on_start,
        mock_xr=args.mock_xr,
        debug=args.debug,
        scale_factor=args.scale_factor,
        ik_dt=args.ik_dt,
        gravity_ff=args.gravity_ff,
        tau_clip=args.tau_clip,
    )
    controller.enable_a_button_reset = bool(args.enable_a_reset)
    if args.gripper_log is None:
        controller.run(rate=args.rate)
    else:
        args.gripper_log.parent.mkdir(parents=True, exist_ok=True)
        try:
            with args.gripper_log.open("x", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=[
                    "time_ns", "home_reset", "gripper_wait_release", "trigger_value", "trigger_valid",
                    "dex1_motor_command_q", "dex1_motor_q", "dex1_tau_est", "dex1_force_latched",
                    "dex1_publish_ok", "dex1_publish_fail_count", "dex1_command_count",
                    "dex1_feedback_valid", "dex1_feedback_age_s", "dex1_feedback_time_ns"])
                writer.writeheader()
                controller.gripper_diagnostic_writer = writer
                print(f"Gripper diagnostics: {args.gripper_log.resolve()}")
                controller.run(rate=args.rate)
        finally:
            controller.gripper_diagnostic_writer = None
            if controller.running:
                controller._on_stop()


if __name__ == "__main__":
    main()
