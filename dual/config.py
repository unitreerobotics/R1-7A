"""One model convention; per-side calibration and physical bus transforms."""
from pathlib import Path
import hashlib
import os

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ARM_SIDES = {'right': ('right',), 'left': ('left',), 'dual': ('right', 'left')}
SDK_ROOT = Path(os.environ.get('R1ARM_SDK_ROOT', ROOT.parent / 'R1_arm_SDK_x86/r1_arm')).expanduser().resolve()
SERIAL_PREFIX = os.environ.get('R1ARM_SERIAL_PREFIX', '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBPG771-')
PORTS = {side: Path(SERIAL_PREFIX + suffix + '-port0') for side, suffix in
         (('right', 'if01'), ('left', 'if03'), ('grip_right', 'if00'), ('grip_left', 'if02'))}
CALIBRATIONS = {side: SDK_ROOT / f'config/joint_calib_zero_{label}臂.txt'
                for side, label in (('right', '右'), ('left', '左'))}
HOME = np.array([0., 1.2, 0., -1.2, 0., 0., 0.])
# Physical calibrated bus -> the existing internal SDK convention.
LEFT_BUS_SIGN = np.array([1., -1., 1., -1., 1., -1., 1.])


def fingerprint(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_pair(right_port, left_port, right_calib, left_calib):
    if Path(right_port).resolve() == Path(left_port).resolve():
        raise ValueError('Left and right arm ports resolve to the same device')
    if Path(right_calib).resolve() == Path(left_calib).resolve():
        raise ValueError('Left and right arms require separate calibration files')
    for side, port in (('right', right_port), ('left', left_port)):
        validate_arm_port(port, side)


def validate_arm_port(port, side=None):
    path = Path(port).expanduser()
    if any(path.resolve() == PORTS[key].resolve() for key in ('grip_right', 'grip_left')):
        raise ValueError(f'Arm port {port} is a known GRIPPER device')
    name = str(path)
    if name.startswith(SERIAL_PREFIX):
        expected = 'if01' if side == 'right' else 'if03' if side == 'left' else None
        if expected and expected not in name:
            raise ValueError(f'{side} arm requires {expected}, got {port}')


def parse_home(text):
    from teleop.r1arm_real_controller import MODEL_LOWER, MODEL_UPPER
    q = np.fromstring(text, sep=',')
    if q.size != 7 or not np.isfinite(q).all() or np.any(q < MODEL_LOWER) or np.any(q > MODEL_UPPER):
        raise ValueError('home must contain 7 finite model joint angles within URDF limits')
    return q
