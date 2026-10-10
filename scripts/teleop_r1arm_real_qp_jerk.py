#!/usr/bin/env python3
"""Experimental real R1Arm teleop with velocity/acceleration/jerk-constrained IK."""

import sys

from teleop_r1arm_real import main


if __name__ == "__main__":
    if "--qp-constrained-ik" not in sys.argv:
        sys.argv.append("--qp-constrained-ik")
    if "--qp-jerk-limit" not in sys.argv and not any(
        arg.startswith("--qp-jerk-limit=") for arg in sys.argv
    ):
        sys.argv.append("--qp-jerk-limit")
        sys.argv.append("80")
    main()
