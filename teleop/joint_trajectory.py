"""Shared validation of elapsed command time."""
import numpy as np


def command_interval(elapsed):
    """Do not invent elapsed time or integrate across a stalled control loop."""
    dt = float(elapsed)
    if not np.isfinite(dt) or not 0 < dt <= .1:
        raise RuntimeError(f'Unsafe command interval: {dt!r} s (expected 0 < dt <= 0.1)')
    return dt
