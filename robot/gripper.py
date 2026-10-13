"""Dex1 position mapping used by the R1Arm MuJoCo controller."""

from __future__ import annotations

import numpy as np


def update_gripper_command(
    current: float,
    trigger_value: float,
    open_q: float,
    close_q: float,
    speed: float,
    dt: float,
) -> float:
    """Map trigger position to a rate-limited simulated gripper position."""
    trigger_value = float(np.clip(trigger_value, 0.0, 1.0))
    desired = open_q + (close_q - open_q) * trigger_value
    max_delta = max(0.0, float(speed) * max(0.0, float(dt)))
    if max_delta <= 0.0:
        return desired
    return float(current + np.clip(desired - current, -max_delta, max_delta))


class Dex1SimGripper:
    """Single simulated Dex1 trigger-to-position mapper."""

    def __init__(
        self,
        open_value: float = 0.0245,
        close_value: float = -0.02,
        speed: float = 0.08,
    ) -> None:
        self.open_q = float(open_value)
        self.close_q = float(close_value)
        self.speed = float(speed)
        self.current = self.open_q

    def update(self, trigger_value: float, dt: float) -> float:
        self.current = update_gripper_command(
            self.current, trigger_value, self.open_q, self.close_q, self.speed, dt
        )
        return self.current

    def reset(self) -> float:
        self.current = self.open_q
        return self.current
