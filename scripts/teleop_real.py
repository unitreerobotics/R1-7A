#!/usr/bin/env python3
"""Real-robot teleoperation for R1-A7 (entrypoint).

Runs teleoperation against the real robot via DDS (``rt/lowcmd`` + Dex1). The
control logic lives in ``teleop.real_controller.RealTeleopController``.

Usage:
    # Dry run (no hardware, no DDS publish) — safe to try anywhere
    python scripts/teleop_real.py --mock-xr

    # Real robot (confirms before moving)
    python scripts/teleop_real.py --motion-mode --enable-dex1 --net eth0

    # Real robot, skip the confirmation prompt
    python scripts/teleop_real.py --motion-mode --enable-dex1 --net eth0 --no-confirm
"""

import argparse
import sys
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.config import R1A7Config
from teleop.real_controller import RealTeleopController


def main():
    parser = argparse.ArgumentParser(description="R1-A7 Real-Robot Teleoperation")
    parser.add_argument("--urdf", type=str, help="URDF file path")
    parser.add_argument("--scale-factor", type=float, default=1.0, help="IK scale factor")
    parser.add_argument("--visualize", action="store_true", help="Enable meshcat visualization")
    parser.add_argument("--no-visualize", action="store_true", help="Disable meshcat visualization")
    parser.add_argument("--mock-xr", action="store_true", help="Use mock XR input (no PICO needed)")
    parser.add_argument("--debug", action="store_true", help="Enable debug output")
    parser.add_argument("--rate", type=float, default=50.0, help="Control loop rate (Hz)")

    parser.add_argument("--net", "--interface", dest="net", type=str, default="eth0",
                        help="DDS network interface")
    parser.add_argument("--motion-mode", action="store_true",
                        help="Publish to the real robot via DDS (off = dry run)")
    parser.add_argument("--enable-dex1", action="store_true",
                        help="Enable the real Dex1 gripper (implies --enable-real-hand)")
    parser.add_argument("--enable-real-hand", action="store_true",
                        help="Enable the real hand controller")
    parser.add_argument("--elbow-lower-override", type=float, default=-1.30,
                        help="Relaxed elbow lower joint limit (rad)")
    parser.add_argument("--head-pose-file", type=str, default="",
                        help="JSON head pose file (pitch/yaw rad)")
    parser.add_argument("--no-confirm", action="store_true",
                        help="Skip the 'move' confirmation prompt before publishing")

    args = parser.parse_args()

    # Pre-flight safety gate, before any DDS channel is opened.
    if args.motion_mode and not args.no_confirm:
        from robot.r1_a7_real import R1A7LowCmdConfig, confirm_real_motion

        confirm_real_motion(
            R1A7LowCmdConfig(interface=args.net, enable_motion=True, no_confirm=False)
        )

    # Build config
    config = R1A7Config(
        scale_factor=args.scale_factor,
        elbow_lower_override=args.elbow_lower_override,
    )
    if args.urdf:
        config.urdf_path = args.urdf
    if args.head_pose_file:
        config.head_pose_file = args.head_pose_file

    # Create and run controller
    controller = RealTeleopController(
        config=config,
        interface=args.net,
        motion_mode=args.motion_mode,
        enable_real_hand=(args.enable_real_hand or args.enable_dex1),
        mock_xr=args.mock_xr,
        visualize=args.visualize and not args.no_visualize,
        debug=args.debug,
        scale_factor=args.scale_factor,
    )

    controller.run(rate=args.rate)


if __name__ == "__main__":
    main()
