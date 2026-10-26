#!/usr/bin/env python3
"""Manually capture four stable Dex1 endpoints using DDS subscription only."""
import argparse
import datetime
import json
from pathlib import Path
import sys
import time
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dual.dex1_monitor import GripperMonitor


def stable_endpoint(samples):
    data = np.asarray(samples, dtype=float)
    if data.ndim != 2 or data.shape[0] < 20 or data.shape[1] != 2 or not np.isfinite(data).all():
        raise ValueError('Need >=20 finite fresh (q,dq) samples')
    if np.ptp(data[:,0]) > .03 or np.max(np.abs(data[:,1])) > .05:
        raise ValueError('Endpoint is moving; capture rejected')
    return float(np.median(data[:,0]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--network', default='lo')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--side', choices=('right', 'left', 'both'), default='both')
    args = parser.parse_args()
    directory = args.output_dir or Path('data/diagnostics')/('dex1_endpoints_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    directory.mkdir(parents=True, exist_ok=False)
    monitor = GripperMonitor(args.network)
    result = {'source': str(directory.resolve()), 'right': {}, 'left': {}}
    try:
        sides = ('right', 'left') if args.side == 'both' else (args.side,)
        for side in sides:
            for endpoint in ('closed', 'open'):
                input(f'Exit ALL command publishers. Gently place {side} gripper at {endpoint}; press Enter (never force against brake): ')
                samples, previous = [], -1
                start = time.monotonic()
                while time.monotonic()-start < 1.2:
                    snapshot = monitor.snapshot()
                    if any(kind == 'cmd' and data['count'] > 0 for (_, kind), data in snapshot.items()):
                        raise RuntimeError('A command publisher is active; manual sampling refused')
                    data = snapshot.get((side, 'state'))
                    if data and data['age'] < .2 and data['count'] != previous:
                        samples.append((data['q'], data['dq']))
                        previous = data['count']
                    time.sleep(.002)
                value = stable_endpoint(samples)
                result[side][endpoint] = value
                (directory/f'{side}_{endpoint}.json').write_text(json.dumps({'q':value,'samples':samples}, indent=2))
                if endpoint == 'open' and result[side]['open'] <= result[side]['closed']:
                    raise ValueError('Open endpoint must be greater than closed endpoint')
                (directory/'partial.json').write_text(json.dumps(result, indent=2))
        (directory/'endpoints.json').write_text(json.dumps(result, indent=2))
        print(f'Review before installation: {directory}/endpoints.json')
    finally:
        monitor.close()


if __name__ == '__main__': main()
