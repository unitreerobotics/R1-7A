#!/usr/bin/env python3
"""Experimental real R1Arm teleop using direct, velocity-limited IK output."""

import sys

from teleop_r1arm_real import main


if __name__ == "__main__":
    if "--unshaped-ik" not in sys.argv:
        sys.argv.append("--unshaped-ik")
    main()
