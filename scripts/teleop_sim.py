#!/usr/bin/env python3
"""Simulation teleoperation for R1-A7 robot (entrypoint).

Runs teleoperation in simulation mode without connecting to the real robot: a
meshcat visualization plus controller-pose tracking. The control logic lives in
``teleop.sim_controller.SimTeleopController``.

Usage:
    # With meshcat visualization (default)
    python scripts/teleop_sim.py

    # Without visualization
    python scripts/teleop_sim.py --no-visualize

    # With mock XR input (no PICO needed)
    python scripts/teleop_sim.py --mock-xr

    # Full options
    python scripts/teleop_sim.py --visualize --mock-xr --debug
"""

import argparse
import sys
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from robot.config import R1A7Config
from teleop.sim_controller import SimTeleopController


def main():
    parser = argparse.ArgumentParser(description="R1-A7 Simulation Teleoperation")
    parser.add_argument("--urdf", type=str, help="URDF file path")
    parser.add_argument("--scale-factor", type=float, default=1.0, help="IK scale factor")
    parser.add_argument("--visualize", action="store_true", default=True, help="Enable meshcat visualization")
    parser.add_argument("--no-visualize", action="store_true", help="Disable meshcat visualization")
    parser.add_argument("--mock-xr", action="store_true", help="Use mock XR input (no PICO needed)")
    parser.add_argument("--debug", action="store_true", help="Enable debug output")
    parser.add_argument("--rate", type=float, default=50.0, help="Control loop rate (Hz)")

    args = parser.parse_args()

    # Build config
    config = R1A7Config(
        scale_factor=args.scale_factor,
    )
    if args.urdf:
        config.urdf_path = args.urdf

    # Create and run controller
    controller = SimTeleopController(
        config=config,
        visualize=not args.no_visualize,
        mock_xr=args.mock_xr,
        debug=args.debug,
        scale_factor=args.scale_factor,
    )

    controller.run(rate=args.rate)


if __name__ == "__main__":
    main()
