"""Physical endpoints are per gripper, independent of motor ID and DDS topic."""
import json
from pathlib import Path
import math


def load_dex1_endpoints(side, path=None):
    if side not in ('right', 'left'):
        raise ValueError('Gripper side must be left or right')
    with Path(path or Path(__file__).with_name('dex1_endpoints.json')).open() as stream:
        data = json.load(stream)[side]
    closed, opened = float(data['closed']), float(data['open'])
    if not all(map(math.isfinite, (closed, opened))) or opened <= closed:
        raise ValueError(f'{side}: invalid gripper endpoints')
    return closed, opened
