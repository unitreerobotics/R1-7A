#!/usr/bin/env python3
"""PICO teleoperation for one standalone real R1Arm (7 DoF + optional Dex1)."""

from __future__ import annotations

import argparse
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
    R1ArmQPConstrainedIKTeleopController,
    R1ArmRealTeleopController,
    R1ArmUnshapedIKTeleopController,
)


def confirm_motion(args: argparse.Namespace) -> None:
    print("\nREAL R1Arm MOTION IS ABOUT TO BE ENABLED")
    print(f"  serial:       {args.port} @ {args.baud}")
    print(f"  calibration:  {args.calib}")
    print(f"  gains:        kp={format_gain(args.kp)}, kd={format_gain(args.kd)}, "
          f"gravity_ff={'on' if args.gravity_ff else 'off'}")
    print(f"  max speed:    {args.max_joint_speed} rad/s")
    if not args.unshaped_ik and not args.qp_constrained_ik:
        if args.velocity_only_shaper:
            print(f"  q_purpose shaper: velocity-only; "
                  f"v≤{(args.trajectory_max_speed or args.max_joint_speed):g} rad/s "
                  "(acceleration/jerk disabled during grip tracking)")
        else:
            print(f"  q_purpose follower: kp={args.trajectory_position_gain:g} s^-2, "
                  f"kd={args.trajectory_velocity_gain:g} s^-1; "
                  f"v≤{(args.trajectory_max_speed or args.max_joint_speed):g} rad/s")
    kp_arr = as_gain_array(args.kp)
    kd_arr = as_gain_array(args.kd)
    torque_cap = float(np.max(kp_arr * args.max_command_lead
                              + kd_arr * args.max_joint_speed
                              + (args.tau_clip if args.gravity_ff else 0.0)))
    print(f"  max lead:     {args.max_command_lead} rad → 力矩上界 {torque_cap:.1f} Nm"
          f"（三项合计；电机峰值 34.4 Nm 的 {torque_cap / 34.4 * 100:.0f}%）")
    print(f"  controller:   {args.side} PICO controller")
    if args.unshaped_ik:
        print("  IK shaping:   off (Placo/DLS per-cycle speed limits; no accel/jerk shaping)")
    if args.qp_constrained_ik:
        jerk_note = (
            f" + jerk ≤{args.qp_jerk_limit:g} rad/s³"
            if args.qp_jerk_limit is not None else " (jerk disabled)"
        )
        print(f"  IK shaping:   Placo QP velocity + acceleration bounds{jerk_note}")
    print("Keep the emergency stop reachable and support the arm during startup probing.")
    answer = input("Type MOVE to continue: ").strip()
    if answer != "MOVE":
        raise SystemExit("Cancelled; no real motor command was sent")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-mode", action="store_true", help="Actually open and command the arm")
    parser.add_argument(
        "--port",
        default=os.environ.get("PORT", "/dev/ttyUSB2"),
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
                        help="控制环 Hz。默认 90；XR 输入约 30Hz，图像采样频率单独配置")
    parser.add_argument("--xr-filter-min-cutoff", type=float, default=8.0,
                        help="XR One-Euro pose filter minimum cutoff Hz")
    parser.add_argument("--xr-filter-beta", type=float, default=1.0,
                        help="XR One-Euro speed response; higher follows fast hand motion more")
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
        help="命令相对实测的最大超前量 rad（默认 0.15）。它限制的是力矩公式"
             "首项 kp×误差 的上界：异常单拍位移 ≤8.6°，三项合计力矩"
             "（kp×lead + kd×|dq| + tau_ff）≈ 峰值一半；正常跟踪不触发",
    )
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--foc-timeout", type=float, default=1.0)
    parser.add_argument("--enable-dex1", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dex1-net", default="enp4s0", help="DDS interface for the one real Dex1")
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
    parser.add_argument(
        "--unshaped-ik", action="store_true",
        help="Experimental direct IK tracking; retain SDK and transition safety limits",
    )
    parser.add_argument(
        "--velocity-only-shaper", action="store_true",
        help="Limit active IK tracking to a per-cycle joint-speed bound only; no acceleration/jerk shaping",
    )
    parser.add_argument(
        "--qp-constrained-ik", action="store_true",
        help="Experimental active IK with velocity/acceleration bounds inside Placo QP",
    )
    parser.add_argument("--qp-jerk-limit", type=float, default=None,
                        help="Optional active IK jerk bound in Placo QP (rad/s^3; e.g. 80)")
    parser.add_argument("--no-confirm", action="store_true")
    args = parser.parse_args()

    if args.motion_mode and args.mock_xr:
        parser.error("--mock-xr cannot be combined with --motion-mode")
    if sum((args.unshaped_ik, args.velocity_only_shaper, args.qp_constrained_ik)) > 1:
        parser.error("--unshaped-ik, --velocity-only-shaper, and --qp-constrained-ik are mutually exclusive")
    if args.qp_jerk_limit is not None and args.qp_jerk_limit <= 0.0:
        parser.error("--qp-jerk-limit must be positive")
    if args.trajectory_position_gain <= 0.0 or args.trajectory_velocity_gain <= 0.0:
        parser.error("trajectory follower gains must be positive")
    if args.max_joint_speed <= 0.0:
        parser.error("--max-joint-speed must be positive")
    if args.trajectory_max_speed is not None and args.trajectory_max_speed <= 0.0:
        parser.error("--trajectory-max-speed must be positive")
    if args.qp_jerk_limit is not None and not args.qp_constrained_ik:
        parser.error("--qp-jerk-limit requires --qp-constrained-ik")
    if args.motion_mode and not args.bridge_lib.expanduser().is_file():
        parser.error(
            f"bridge library not found: {args.bridge_lib}; "
            "run python scripts/build_r1arm_sdk_bridge.py first"
        )
    if args.motion_mode and not Path(args.port).exists():
        parser.error(
            f"serial port not found: {args.port}; check /dev/serial/by-id and /dev/ttyUSB*"
        )

    calibration = args.calib or args.sdk_root / "config/joint_calib_candidate.txt"
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
        xr_filter_min_cutoff=args.xr_filter_min_cutoff,
        xr_filter_beta=args.xr_filter_beta,
        trajectory_position_gain=args.trajectory_position_gain,
        trajectory_velocity_gain=args.trajectory_velocity_gain,
        trajectory_max_speed_rad_s=(
            args.trajectory_max_speed
            if args.trajectory_max_speed is not None else args.max_joint_speed
        ),
    )
    if args.urdf:
        config.urdf_path = str(args.urdf.expanduser().resolve())

    controller_cls = (
        R1ArmQPConstrainedIKTeleopController if args.qp_constrained_ik
        else R1ArmUnshapedIKTeleopController if args.unshaped_ik
        else R1ArmRealTeleopController
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
        dex1_domain=args.dex1_domain,
        grip_torque_limit=args.grip_torque_limit,
        **({"qp_jerk_limit": args.qp_jerk_limit} if args.qp_constrained_ik else {}),
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
    controller.velocity_only_shaper = bool(args.velocity_only_shaper)
    controller.run(rate=args.rate)


if __name__ == "__main__":
    main()
