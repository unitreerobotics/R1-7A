"""Four-camera MCAP episode writer for R1-A7 teleoperation recordings.

Migrated from ``teleop/utils/mcap_writer.py`` in the original
``xr_teleoperate_r1a7`` repository. The on-disk layout is deliberately kept
byte-for-byte compatible with the existing datasets — episodes recorded here
must stay readable by the original tooling, and vice versa. Do not "tidy up"
the topic names, key names, or the JPEG/base64 encoding.

Layout of one ``episode_XXXX.mcap``:
  ``/episode/meta``        one JSON message: {info, text, config}
  ``/whole_body/frame``    one JSON message per sample:
                           {idx, sample_time_ns, image_source_frame_id,
                            colors, states, actions}

``colors`` holds the four cameras in the fixed ``CAMERA_ORDER`` below, each as
a base64-encoded JPEG.
"""

from __future__ import annotations

import base64
import datetime
import json
import os
import time

import cv2


class FourCameraMcapRecorder:
    """Write R1-A7 teleoperation episodes as JSON messages in an MCAP file."""

    CAMERA_ORDER = (
        "color_0_head_left",
        "color_1_head_right",
        "color_2_wrist_left",
        "color_3_wrist_right",
    )

    def __init__(self, task_dir: str, task_goal: str, frequency: float) -> None:
        from mcap.writer import Writer

        self._writer_type = Writer
        self.task_dir = os.path.abspath(task_dir)
        self.task_goal = str(task_goal)
        self.frequency = float(frequency)
        os.makedirs(self.task_dir, exist_ok=True)
        self.episode_id = self._find_last_episode_id()
        self.file_handle = None
        self.writer = None
        self.frame_channel_id = None
        self.item_id = -1
        self.current_path = None

    def _find_last_episode_id(self) -> int:
        """Continue numbering after whatever is already in the task dir."""
        ids = []
        for name in os.listdir(self.task_dir):
            if name.startswith("episode_") and name.endswith(".mcap"):
                number = name[len("episode_") : -len(".mcap")]
                if number.isdigit():
                    ids.append(int(number))
        return max(ids) if ids else -1

    @staticmethod
    def _schema() -> dict:
        return {
            "type": "object",
            "properties": {
                "idx": {"type": "integer"},
                "sample_time_ns": {"type": "integer"},
                "image_source_frame_id": {"type": "integer"},
                "colors": {"type": "object"},
                "states": {"type": "object"},
                "actions": {"type": "object"},
            },
            "required": ["idx", "sample_time_ns", "colors", "states", "actions"],
        }

    def start_episode(self, metadata: dict) -> str:
        """Open a new episode file and write its metadata message."""
        if self.writer is not None:
            raise RuntimeError("An MCAP episode is already open")

        self.episode_id += 1
        self.item_id = -1
        self.current_path = os.path.join(self.task_dir, f"episode_{self.episode_id:04d}.mcap")
        self.file_handle = open(self.current_path, "wb")
        self.writer = self._writer_type(self.file_handle)
        self.writer.start()

        meta_schema_id = self.writer.register_schema(
            name="r1_a7.episode_meta",
            encoding="jsonschema",
            data=json.dumps({"type": "object"}, ensure_ascii=False).encode("utf-8"),
        )
        meta_channel_id = self.writer.register_channel(
            topic="/episode/meta", message_encoding="json", schema_id=meta_schema_id
        )
        frame_schema_id = self.writer.register_schema(
            name="r1_a7.four_camera_frame",
            encoding="jsonschema",
            data=json.dumps(self._schema(), ensure_ascii=False).encode("utf-8"),
        )
        self.frame_channel_id = self.writer.register_channel(
            topic="/whole_body/frame", message_encoding="json", schema_id=frame_schema_id
        )

        timestamp_ns = time.time_ns()
        meta = {
            "info": {
                "format": "r1_a7_four_camera_mcap_json_v1",
                "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
                "frequency_hz": self.frequency,
                "camera_order": list(self.CAMERA_ORDER),
                "camera_positions": ["head_left", "head_right", "wrist_left", "wrist_right"],
            },
            "text": {"goal": self.task_goal},
            "config": metadata,
        }
        self.writer.add_message(
            channel_id=meta_channel_id,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(meta, ensure_ascii=False).encode("utf-8"),
        )
        return self.current_path

    @staticmethod
    def _encode_color(frame):
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            raise RuntimeError("Failed to JPEG encode camera frame")
        return {
            "encoding": "jpeg",
            "height": int(frame.shape[0]),
            "width": int(frame.shape[1]),
            "channels": int(frame.shape[2]) if frame.ndim == 3 else 1,
            "data": base64.b64encode(encoded.tobytes()).decode("ascii"),
        }

    def add_frame(
        self,
        frames: tuple,
        states: dict,
        actions: dict,
        image_source_frame_id: int,
    ) -> None:
        """Append one sample. ``frames`` must follow ``CAMERA_ORDER``."""
        if self.writer is None or self.frame_channel_id is None:
            return
        if len(frames) != 4:
            raise ValueError(f"Expected four camera frames, got {len(frames)}")

        self.item_id += 1
        timestamp_ns = time.time_ns()
        colors = {
            name: self._encode_color(frame)
            for name, frame in zip(self.CAMERA_ORDER, frames)
        }
        message = {
            "idx": self.item_id,
            "sample_time_ns": timestamp_ns,
            "image_source_frame_id": int(image_source_frame_id),
            "colors": colors,
            "states": states,
            "actions": actions,
        }
        self.writer.add_message(
            channel_id=self.frame_channel_id,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(message, ensure_ascii=False).encode("utf-8"),
        )

    def stop_episode(self) -> None:
        """Finalize and close the open episode, if any."""
        if self.writer is not None:
            self.writer.finish()
            self.writer = None
        if self.file_handle is not None:
            self.file_handle.close()
            self.file_handle = None
        self.frame_channel_id = None

    def delete_last_episode(self):
        """Delete the latest closed episode and return its ``(id, path)``.

        Returns ``None`` if an episode is still open or nothing is on disk.
        """
        if self.writer is not None or self.episode_id < 0:
            return None
        episode_id = self.episode_id
        episode_path = os.path.join(self.task_dir, f"episode_{episode_id:04d}.mcap")
        if not os.path.isfile(episode_path):
            return None
        os.remove(episode_path)
        self.episode_id -= 1
        self.current_path = None
        return episode_id, episode_path