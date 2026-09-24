#!/usr/bin/env python3
"""PICO teleoperation for one standalone real R1Arm (7 DoF + optional Dex1)."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.r1arm_config import R1ArmConfig
from robot.r1arm_sdk import DEFAULT_BRIDGE_LIBRARY, DEFAULT_SDK_ROOT, validate_calibration_file
from teleop.r1arm_real_controller import R1ArmRealTeleopController


def confirm_motion(args: argparse.Namespace) -> None:
    print("\nREAL R1Arm MOTION IS ABOUT TO BE ENABLED")
    print(f"  serial:       {args.port} @ {args.baud}")
    print(f"  calibration:  {args.calib}")
    print(f"  gains:        kp={args.kp}, kd={args.kd}")
    print(f"  max speed:    {args.max_joint_speed} rad/s")
    print(f"  max lead:     {args.max_command_lead} rad")
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
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--kp", type=float, default=80.0)
    parser.add_argument("--kd", type=float, default=2)
    parser.add_argument("--max-joint-speed", type=float, default=1, help="rad/s")
    parser.add_argument("--max-command-lead", type=float, default=1, help="rad")
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
        default=100,
        help="A/home reset duration in control frames (default: 100 = about 2 s at 50 Hz)",
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
    )
    controller.enable_a_button_reset = bool(args.enable_a_reset)
    controller.run(rate=args.rate)


if __name__ == "__main__":
    main()
