#!/usr/bin/env python3
"""Experimental real R1Arm teleop with velocity/acceleration-constrained IK."""

import sys

from teleop_r1arm_real import main


if __name__ == "__main__":
    if "--qp-constrained-ik" not in sys.argv:
        sys.argv.append("--qp-constrained-ik")
    main()
