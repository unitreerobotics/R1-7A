#!/usr/bin/env python3
"""Record real R1Arm teleoperation using velocity/acceleration-constrained IK."""

import sys

from record_r1arm_real import main


if __name__ == "__main__":
    if "--qp-constrained-ik" not in sys.argv:
        sys.argv.append("--qp-constrained-ik")
    main()
