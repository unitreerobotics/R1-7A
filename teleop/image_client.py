"""Four-camera image subscriber for the R1-A7 PC2 image service.

Migrated from ``record/r1_a7_record.py`` in the original ``xr_teleoperate_r1a7``
repository. Receives the PC2 Dex1 wire format over ZMQ and splits it into the
four camera views used by the recorder.

Wire format (must match the server running on the robot's PC2):
  ``struct("II")`` header -> (color_jpeg_size, ir_size)
  followed by ``color_jpeg_size`` bytes of JPEG holding all four camera views
  concatenated horizontally, then an optional IR payload we ignore.

The split is fixed as: head_left, head_right, wrist_left, wrist_right.
"""

from __future__ import annotations

import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np
import zmq


CAMERA_NAMES = ("head_left", "head_right", "wrist_left", "wrist_right")


class FourCameraSubscriber:
    """Receive the PC2 Dex1 wire format and split its four-camera JPEG."""

    def __init__(self, address: str, port: int) -> None:
        self.connect = f"tcp://{address}:{int(port)}"
        self.lock = threading.Lock()
        self.frames: Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None
        self.frame_id = 0
        self.received_at_ns = 0
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None

    def start(self) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        # Keep only the newest frames: recording wants live images, not a backlog.
        self.socket.setsockopt(zmq.RCVHWM, 2)
        self.socket.setsockopt(zmq.RCVTIMEO, 500)
        self.socket.connect(self.connect)
        self.thread = threading.Thread(target=self._loop, name="r1-a7-four-camera-sub", daemon=True)
        self.thread.start()
        print(f"[Image] receiving PC2 Dex1 stream from {self.connect}")

    @staticmethod
    def _decode(message: bytes) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        header_size = struct.calcsize("II")
        if len(message) < header_size:
            raise ValueError("message is shorter than the Dex1 image header")
        color_size, ir_size = struct.unpack("II", message[:header_size])
        color_end = header_size + color_size
        if color_size <= 0 or color_end > len(message):
            raise ValueError("invalid JPEG size in Dex1 image header")
        if ir_size and color_end + ir_size > len(message):
            raise ValueError("invalid IR size in Dex1 image header")

        combined = cv2.imdecode(np.frombuffer(message[header_size:color_end], dtype=np.uint8), cv2.IMREAD_COLOR)
        if combined is None:
            raise ValueError("failed to decode PC2 JPEG")
        if combined.shape[1] % 4:
            raise ValueError(f"combined image width {combined.shape[1]} cannot be divided into four cameras")
        head_left, head_right, wrist_left, wrist_right = np.hsplit(combined, 4)
        return head_left, head_right, wrist_left, wrist_right

    def _loop(self) -> None:
        assert self.socket is not None
        last_error = ""
        while not self.stop_event.is_set():
            try:
                message = self.socket.recv()
                frames = self._decode(message)
            except zmq.Again:
                continue
            except zmq.ZMQError:
                if self.stop_event.is_set():
                    break
                continue
            except Exception as exc:
                # A malformed stream would otherwise flood the console at frame
                # rate; print each distinct error only once.
                error = str(exc)
                if error != last_error:
                    print(f"[Image] skipped malformed frame: {error}")
                    last_error = error
                continue
            with self.lock:
                self.frames = frames
                self.frame_id += 1
                self.received_at_ns = time.time_ns()

    def wait_first_frame(self, timeout: float) -> bool:
        """Block until the first frame arrives or ``timeout`` seconds elapse."""
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while time.monotonic() < deadline:
            with self.lock:
                if self.frames is not None:
                    return True
            time.sleep(0.05)
        with self.lock:
            return self.frames is not None

    def get(self) -> tuple[Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]], int, int]:
        """Return ``(frames, frame_id, received_at_ns)``; frames is None if idle."""
        with self.lock:
            if self.frames is None:
                return None, self.frame_id, self.received_at_ns
            return tuple(frame.copy() for frame in self.frames), self.frame_id, self.received_at_ns

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.socket is not None:
            self.socket.close(0)
            self.socket = None
        if self.context is not None:
            self.context.term()
            self.context = None