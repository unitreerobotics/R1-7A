"""Capture a NEW per-arm physical-zero calibration; never infer new offsets offline."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dual.config import PORTS, CALIBRATIONS, ROOT, validate_arm_port, fingerprint
from robot.r1arm_sdk import R1ArmSdkBus, validate_calibration_file


def offsets_from_samples(samples, direction, scale):
    values = np.asarray(samples, dtype=float)
    if values.ndim != 2 or values.shape[1] != 7 or values.shape[0] < 20 or not np.isfinite(values).all():
        raise ValueError('Need at least 20 complete finite seven-joint samples')
    if np.max(np.ptp(values * direction * scale, axis=0)) > .02:
        raise ValueError('Arm moved during capture; no calibration written')
    return -np.mean(values, axis=0) * direction * scale


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('right', 'left'), required=True)
    parser.add_argument('--port', type=Path)
    parser.add_argument('--calib', type=Path, help='Existing physical calibration; directions/scales are retained')
    parser.add_argument('--bridge-lib', type=Path, default=ROOT/'build/dual_sdk_bridge/libr1arm_sdk_bridge.so')
    parser.add_argument('--capture', action='store_true', help='Send zero-gain probes, capture supported physical zero')
    parser.add_argument('--install', action='store_true', help='Back up and replace the named physical calibration after successful capture')
    args = parser.parse_args(argv)
    port, path = args.port or PORTS[args.side], args.calib or CALIBRATIONS[args.side]
    validate_arm_port(port, args.side)
    path = validate_calibration_file(path)
    source_digest = fingerprint(path)
    table = np.loadtxt(path)
    if table.shape != (7, 4) or not np.array_equal(table[:,0], np.arange(7)):
        raise ValueError('Capture requires ordered joint dir scale offset rows 0..6')
    direction, scale, old_offset = table[:,1], table[:,2], table[:,3]
    print(f'{args.side}: port={port}; calibration={path}; physical reference=all seven joints zero')
    if not args.capture:
        print('DRY RUN: no bus opened. Use --capture after supporting and positioning this arm at physical zero.')
        return
    if not port.exists():
        raise FileNotFoundError(f'Arm port is missing: {port}')
    if subprocess.run(['fuser',str(port)],capture_output=True).returncode == 0:
        raise RuntimeError('Arm port is in use; stop teleop/replay before manual calibration')
    if input(f'Support the arm at PHYSICAL ZERO. Type ZERO {args.side}: ').strip() != f'ZERO {args.side}':
        raise SystemExit('Cancelled')
    bus = R1ArmSdkBus(args.bridge_lib)
    samples = []
    previous = None
    try:
        bus.open(str(port), 6000000, path)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and len(samples) < 50:
            bus.probe_once()
            state = bus.read_state()
            if np.any(state.error_code):
                raise RuntimeError('Motor fault during calibration; no file written')
            if np.all(state.ready):
                if not np.isfinite(state.q).all() or not np.isfinite(state.dq).all() or np.max(np.abs(state.dq)) > .05:
                    raise RuntimeError('Arm is moving or feedback is non-finite; no calibration written')
                if state.feedback_count is None:
                    raise RuntimeError('Freshness-capable SDK bridge required for calibration')
                if previous is None or np.all(state.feedback_count > previous):
                    samples.append((state.q-old_offset)/(direction*scale))
                    previous = state.feedback_count.copy()
            time.sleep(.02)
    finally:
        try:
            bus.stop()
        finally:
            bus.close()
    if fingerprint(path) != source_digest:
        raise RuntimeError("Source calibration changed during capture; installation refused")
    offset = offsets_from_samples(samples, direction, scale)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    candidate = path.with_name(path.stem+f'.candidate_{stamp}.txt')
    with candidate.open('x') as stream:
        stream.write('# joint_calib: calibrated = dir * scale * motor + offset\n# Physical zero; model direction adapters are separate\n')
        for i in range(7):
            stream.write(f'{i} {direction[i]:.8f} {scale[i]:.8f} {offset[i]:.8f}\n')
    validate_calibration_file(candidate)
    candidate.with_suffix('.json').write_text(json.dumps({
        'side': args.side, 'port': str(port), 'reference_q': [0]*7,
        'source_calibration': str(path), 'source_sha256': source_digest,
        'samples': len(samples), 'raw_motor_mean': np.mean(samples, axis=0).tolist(),
        'raw_motor_span': np.ptp(samples, axis=0).tolist(),
    }, indent=2))
    print(f'New candidate: {candidate}')
    if args.install:
        backup = path.with_name(path.name+f'.backup_{stamp}')
        with backup.open('xb') as stream:
            stream.write(path.read_bytes())
        fd, temporary = tempfile.mkstemp(dir=path.parent, prefix='.calib-')
        try:
            with os.fdopen(fd, 'wb') as stream:
                stream.write(candidate.read_bytes())
                stream.flush(); os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary): os.unlink(temporary)
        print(f'Installed {path}; old calibration preserved as {backup}')


if __name__ == '__main__':
    main()
