#!/usr/bin/env python3
"""Read-only camera verification using the production recording defaults."""
import argparse
import os
import sys
import time
from pathlib import Path

# The installed OpenCV wheel provides xcb, but no Wayland Qt plugin.
if os.environ.get('QT_QPA_PLATFORM') == 'wayland':
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.local_camera_client import DEFAULT_CAMERAS, DUAL_DEFAULT_CAMERAS, LocalCameraSubscriber, parse_camera_specs


def main(mode='right'):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--no-window', action='store_true')
    args = parser.parse_args()
    defaults = DUAL_DEFAULT_CAMERAS if mode == 'dual' else DEFAULT_CAMERAS
    camera = LocalCameraSubscriber(parse_camera_specs(list(defaults)))
    print(f'[{mode}] production defaults: expected 640x480 @ 30 Hz per camera', flush=True)
    print('\n'.join(defaults), flush=True)
    camera.start()
    try:
        camera.resolve_active(10)
        if camera.inactive or len(camera.active_keys) != len(defaults):
            raise RuntimeError(f'All requested cameras required: {camera.inactive}')
        initial = {w.spec.key: w.snapshot()[1] for w in camera.workers}
        previous = dict(initial)
        start = last = time.monotonic()
        shapes = {key:set() for key in initial}
        bad = False
        while time.monotonic()-start < args.seconds:
            now = time.monotonic()
            report = now-last >= 2
            tiles = []
            for worker in camera.workers:
                frame, count, _, mono, _ = worker.snapshot()
                key = worker.spec.key
                if frame is None:
                    raise RuntimeError(f'{key}: missing frame')
                h,w = frame.shape[:2]
                shapes[key].add((w,h))
                age = (time.monotonic_ns()-mono)/1e9
                valid = (w,h)==(640,480) and age < .2 and worker.status != 'frozen'
                bad |= not valid
                if report:
                    hz = (count-previous[key])/(now-last)
                    print(f'{key}: actual={w}x{h}, capture={hz:.2f} Hz, age={age*1000:.1f} ms, status={worker.status}',flush=True)
                    previous[key] = count
                if not args.no_window:
                    tile = cv2.resize(frame,(640,480))
                    cv2.putText(tile,f'{key} {w}x{h}',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0) if valid else (0,0,255),2)
                    tiles.append(tile)
            if report:
                last=now
            if tiles:
                cv2.imshow(f'{mode} recording cameras: q to exit',np.hstack(tiles))
                if cv2.waitKey(1)&255 in (27,ord('q')):
                    break
            else:
                time.sleep(.01)
        elapsed=time.monotonic()-start
        for worker in camera.workers:
            key=worker.spec.key
            hz=(worker.snapshot()[1]-initial[key])/elapsed
            ok=shapes[key]=={(640,480)} and 28<=hz<=32 and not bad
            print(f'{key}: {"PASS" if ok else "CHECK"}; shapes={sorted(shapes[key])}; average={hz:.2f} Hz over {elapsed:.1f}s',flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()
        if not args.no_window:
            cv2.destroyAllWindows()


if __name__=='__main__':
    main()
