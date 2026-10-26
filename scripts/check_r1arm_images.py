#!/usr/bin/env python3
"""Decode every recorded image and report timing; optional image-only playback."""
import argparse
import base64
import hashlib
import json
import os
import time
from pathlib import Path

if os.environ.get('QT_QPA_PLATFORM') == 'wayland':
    os.environ['QT_QPA_PLATFORM'] = 'xcb'

import cv2
import numpy as np
from mcap.reader import make_reader


def inspect(path, preview=False):
    stats = {}
    count = 0
    expected = None
    previous_time = None
    start = time.monotonic()
    first_time = None
    with Path(path).open('rb') as stream:
        for _, channel, message in make_reader(stream).iter_messages():
            if channel.topic == '/episode/meta':
                expected = json.loads(message.data)['info'].get('camera_order')
            if channel.topic != '/whole_body/frame':
                continue
            row = json.loads(message.data)
            colors = row['colors']
            if not colors or (expected is not None and set(colors) != set(expected)):
                raise ValueError(f'frame {count}: camera keys missing or inconsistent')
            tiles = []
            stamp = row['sample_time_ns']
            if first_time is None:
                first_time = stamp
            for key, record in colors.items():
                if record.get('encoding') != 'jpeg':
                    raise ValueError(f'{key}: unsupported encoding')
                raw = base64.b64decode(record['data'], validate=True)
                image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(f'frame {count}: {key} JPEG decode failed')
                s = stats.setdefault(key, dict(frames=0, shapes=set(), ids=[], ages_ms=[], hashes=set()))
                s['frames'] += 1
                s['shapes'].add(tuple(image.shape))
                s['hashes'].add(hashlib.sha256(raw).hexdigest())
                if key in row.get('camera_frame_ids', {}):
                    s['ids'].append(row['camera_frame_ids'][key])
                if key in row.get('camera_frame_times_ns', {}):
                    s['ages_ms'].append((stamp-row['camera_frame_times_ns'][key])/1e6)
                if preview:
                    tile = cv2.resize(image, (640,480))
                    cv2.putText(tile,key,(10,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
                    tiles.append(tile)
            count += 1
            previous_time = stamp
            if preview:
                cv2.imshow('Recorded images only: q to exit',np.hstack(tiles))
                delay = max(1,int(((stamp-first_time)*1e-9-(time.monotonic()-start))*1000))
                if cv2.waitKey(delay) & 255 in (ord('q'),27):
                    break
    cv2.destroyAllWindows() if preview else None
    if not count:
        raise ValueError('No image observations found')
    for s in stats.values():
        ids = s.pop('ids')
        ages = s.pop('ages_ms')
        s['shapes'] = sorted(s['shapes'])
        s['unique_images'] = len(s.pop('hashes'))
        s['repeated_frame_ids'] = sum(a == b for a,b in zip(ids,ids[1:]))
        s['frame_id_rollbacks'] = sum(b < a for a,b in zip(ids,ids[1:]))
        s['image_age_ms_p50_p95_max'] = np.percentile(ages,[50,95,100]).tolist() if ages else None
    return dict(observations=count,observation_hz=(count-1)/((previous_time-first_time)*1e-9) if count>1 else None,cameras=stats)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path)
    parser.add_argument('--preview',action='store_true')
    args = parser.parse_args()
    print(json.dumps(inspect(args.episode,args.preview),indent=2))
