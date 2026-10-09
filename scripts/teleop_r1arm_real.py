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
from teleop.r1arm_real_controller import R1ArmRealTeleopController


def confirm_motion(args: argparse.Namespace) -> None:
    print("\nREAL R1Arm MOTION IS ABOUT TO BE ENABLED")
    print(f"  serial:       {args.port} @ {args.baud}")
    print(f"  calibration:  {args.calib}")
    print(f"  gains:        kp={format_gain(args.kp)}, kd={format_gain(args.kd)}, "
          f"gravity_ff={'on' if args.gravity_ff else 'off'}")
    print(f"  max speed:    {args.max_joint_speed} rad/s")
    kp_arr = as_gain_array(args.kp)
    kd_arr = as_gain_array(args.kd)
    torque_cap = float(np.max(kp_arr * args.max_command_lead
                              + kd_arr * args.max_joint_speed
                              + (args.tau_clip if args.gravity_ff else 0.0)))
    print(f"  max lead:     {args.max_command_lead} rad → 力矩上界 {torque_cap:.1f} Nm"
          f"（三项合计；电机峰值 34.4 Nm 的 {torque_cap / 34.4 * 100:.0f}%）")
    print(f"  controller:   {args.side} PICO controller")
    print("Keep the emergency stop reachable and support the arm during startup probing.")
    answer = input("Type MOVE to continue: ").strip()
    if answer != "MOVE":
        raise SystemExit("Cancelled; no real motor command was sent")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-mode", action="store_true", help="Actually open and command the arm")
    parser.add_argument(
        "--port",
        default=os.environ.get(
            "PORT", "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTANPO49-if02-port0"
        ),
        help="R1Arm RS485 serial device",
    )
    parser.add_argument("--baud", type=int, default=6_000_000)
    parser.add_argument(
        "--sdk-root",
        type=Path,
        default=DEFAULT_SDK_ROOT,
        help="Used to derive the default calibration path",
    )
    parser.add_argument("--calib", type=Path, help="joint_calib.txt path")
    parser.add_argument("--bridge-lib", type=Path, default=DEFAULT_BRIDGE_LIBRARY)
    parser.add_argument("--urdf", type=Path, help="Simulation/IK URDF (same model as the working sim)")
    parser.add_argument("--side", choices=["left", "right"], default="right")
    parser.add_argument("--scale-factor", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=30.0,
                        help="控制环 Hz。默认 30（2026-10-09 定居：XR帧时钟 1:1 + "
                             "IK + 执行 + 录制全链 30Hz，action/state 同拍统一）")
    parser.add_argument(
        "--ik-dt", type=float, default=0.0,
        help="IK 积分步长 s；0 = 跟随 1/--rate（对齐）。100 Hz 下如果 dt 对齐后"
             "出现振荡/噪声（IK 增益恢复全幅），试 0.02：保留指令密度收益、"
             "压回 IK 每步增量",
    )
    parser.add_argument(
        "--gravity-ff", dest="gravity_ff", action="store_true", default=True,
        help="下发重力前馈力矩（指令姿态处计算，clip 后经 v2 tau 通道）；"
             "默认开（P0 三重实测：J4 悬垂误差 −70%）。--no-gravity-ff 关闭",
    )
    parser.add_argument(
        "--no-gravity-ff", dest="gravity_ff", action="store_false",
        help="关闭重力前馈（回到 kp 硬扛悬垂的旧行为）",
    )
    parser.add_argument(
        "--tau-clip", type=float, default=8.0,
        help="重力前馈单关节力矩限幅 Nm（默认 8）",
    )
    parser.add_argument("--kp", type=parse_gain, default=80.0,
                        help="位置环增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--kd", type=parse_gain, default=2,
                        help="阻尼增益；单个数=全关节广播，或 7 个逗号分隔值=逐关节")
    parser.add_argument("--max-joint-speed", type=float, default=1.0,
                        help="rad/s（默认 1.5：电机能力 48%，与 QP/整形器三层同源）")
    parser.add_argument(
        "--max-command-lead", type=float, default=0.15,
        help="命令相对实测的最大超前量 rad（默认 0.15）。它限制的是力矩公式"
             "首项 kp×误差 的上界：异常单拍位移 ≤8.6°，三项合计力矩"
             "（kp×lead + kd×|dq| + tau_ff）≈ 峰值一半；正常跟踪不触发",
    )
    parser.add_argument("--feedback-timeout", type=float, default=3.0)
    parser.add_argument("--foc-timeout", type=float, default=1.0)
    parser.add_argument("--enable-dex1", action="store_true")
    parser.add_argument("--dex1-net", default="", help="DDS interface for the one real Dex1")
    parser.add_argument("--dex1-domain", type=int, default=0)
    parser.add_argument("--grip-torque-limit", type=float, default=-1.0)
    parser.add_argument("--enable-a-reset", action="store_true", help="Allow A to move toward ARM_INIT")
    parser.add_argument(
        "--home-on-start",
        action="store_true",
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
    parser.add_argument("--no-confirm", action="store_true")
    args = parser.parse_args()

    if args.motion_mode and args.mock_xr:
        parser.error("--mock-xr cannot be combined with --motion-mode")
    if args.motion_mode and not args.bridge_lib.expanduser().is_file():
        parser.error(
            f"bridge library not found: {args.bridge_lib}; "
            "run python scripts/build_r1arm_sdk_bridge.py first"
        )
    if args.motion_mode and not Path(args.port).exists():
        parser.error(
            f"serial port not found: {args.port}; check /dev/serial/by-id and /dev/ttyUSB*"
        )

    calibration = args.calib or args.sdk_root / "config/joint_calib.txt"
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
    )
    if args.urdf:
        config.urdf_path = str(args.urdf.expanduser().resolve())

    controller = R1ArmRealTeleopController(
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
    controller.run(rate=args.rate)


if __name__ == "__main__":
    main()
