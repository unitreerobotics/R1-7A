#!/usr/bin/env python3
"""View the local R1Arm cameras live to adjust their mounting.

Opens the exact same backends and camera specs as ``record_r1arm_real.py``
(``--camera`` parses identically), shows the frames in one labeled window and
prints per-camera status, so what you see while positioning the cameras is
what the recorder will store. Quit with q or Esc.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# The OpenCV wheel in r1_teleop ships the xcb Qt plugin, not the Wayland one.
if os.environ.get("QT_QPA_PLATFORM") == "wayland":
    os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from teleop.local_camera_client import (
    DEFAULT_CAMERAS,
    LocalCameraSubscriber,
    parse_camera_specs,
)

WINDOW = "R1Arm cameras: head / wrist"


def _labeled(frame: np.ndarray, label: str, color=(0, 255, 0)) -> np.ndarray:
    image = frame.copy()
    cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                color, 2, cv2.LINE_AA)
    return image


def _tile(view: str, status: str, age: float, size=(480, 640)) -> np.ndarray:
    age_text = f"{age:.1f}s ago" if np.isfinite(age) else "no frame yet"
    tile = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    for i, text in enumerate(("NO SIGNAL", view, f"status: {status}", age_text)):
        color = (0, 0, 255) if i == 0 else (200, 200, 200)
        cv2.putText(tile, text, (40, 120 + i * 48), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, color, 2, cv2.LINE_AA)
    return tile


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--camera",
        action="append",
        metavar="SPEC",
        default=None,
        help="Same syntax as record_r1arm_real.py; default: both local cameras",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Auto-close after this many seconds (0 = until q/Esc)",
    )
    parser.add_argument("--scale", type=float, default=0.75, help="Preview scale")
    args = parser.parse_args()
    if not args.camera:
        args.camera = list(DEFAULT_CAMERAS)
    specs = parse_camera_specs(list(args.camera))

    camera = LocalCameraSubscriber(specs)
    camera.start()
    try:
        print("[View] waiting up to 8s for camera frames...", flush=True)
        deadline = time.monotonic() + 8.0
        next_status = time.monotonic()
        while time.monotonic() < deadline and not camera.wait_first_frame(timeout=0.2):
            if time.monotonic() >= next_status:
                print(f"[View] connecting: {camera.describe()}", flush=True)
                next_status = time.monotonic() + 1.0
        # Keep the same stability check used by recording before showing cameras.
        camera.resolve_active(timeout=0.0)
        for key, detail in camera.inactive:
            print(f"[View] UNAVAILABLE {key}: {detail}")
        if not camera.active_keys:
            raise SystemExit(
                "[View] no camera delivered a frame; run scripts/verify_r1arm_right_cameras.py"
            )
        print(f"[View] showing: {list(camera.active_keys)} (q or Esc to quit)")

        active = camera.active_keys
        deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
        while True:
            frames, frame_id, _, _ = camera.get()
            health = camera.health()
            images = []
            for view in active:
                status, age = health.get(view, ("unknown", float("inf")))
                if frames is None:
                    images.append(_tile(view, status, age))
                    continue
                # camera.get() returns frames in active-keys order
                frame = frames[active.index(view)]
                if status == "frozen":
                    images.append(_labeled(frame, f"{view} | FROZEN", color=(0, 0, 255)))
                else:
                    stamp = f"{frame_id}" if np.isfinite(age) else "-"
                    images.append(
                        _labeled(frame, f"{view} | {status} | packet {stamp}")
                    )
            common_h = min(im.shape[0] for im in images)
            resized = []
            for im in images:
                if im.shape[0] == common_h:
                    resized.append(im)
                else:
                    s = common_h / im.shape[0]
                    resized.append(cv2.resize(im, None, fx=s, fy=s,
                                              interpolation=cv2.INTER_AREA))
            preview = np.hstack(resized)
            if args.scale != 1.0:
                preview = cv2.resize(preview, None, fx=args.scale, fy=args.scale,
                                     interpolation=cv2.INTER_AREA)
            cv2.imshow(WINDOW, preview)
            if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()
        cv2.destroyAllWindows()
        print("[View] closed")


if __name__ == "__main__":
    main()
