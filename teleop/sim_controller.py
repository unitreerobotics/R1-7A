"""Simulation teleoperation controller for R1-A7.

Drives a Pinocchio model with meshcat visualization instead of the real robot.
The tracking + IK logic lives in ``teleop.r1a7_controller_base``; this subclass
only supplies the output side (meshcat visualization).  The base-class
``_output_command`` is already a no-op, so no override is needed here.
"""

from __future__ import annotations

from teleop.r1a7_controller_base import R1A7TeleopControllerBase


class SimTeleopController(R1A7TeleopControllerBase):
    """Simulation teleoperation controller using controller pose tracking."""
