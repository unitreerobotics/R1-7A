"""MCAP episode writer used by the R1Arm recorders.

Migrated from ``teleop/utils/mcap_writer.py`` in the original
``xr_teleoperate_r1a7`` repository. Stable topic names, keys, and JPEG/base64
encoding are retained so existing MCAP episodes remain readable.

Layout of one ``episode_XXXX.mcap``:
  ``/episode/meta``        one JSON message: {info, text, config}
  ``/whole_body/frame``    one JSON message per sample:
                           {idx, sample_time_ns, image_source_frame_id,
                            colors, states, actions}
  ``/r1arm/control_sample`` optional, one JSON message per control cycle

``colors`` holds base64-encoded JPEG observations. Simulation callers may
select a subset through ``camera_order``.
"""

from __future__ import annotations

import base64
import datetime
import json
import os
import time

import cv2


class FourCameraMcapRecorder:
    """Write teleoperation episodes as JSON messages in an MCAP file."""

    CAMERA_ORDER = (
        "color_0_head_left",
        "color_1_head_right",
        "color_2_wrist_left",
        "color_3_wrist_right",
    )

    def __init__(
        self,
        task_dir: str,
        task_goal: str,
        frequency: float,
        format_name: str = "r1_a7_four_camera_mcap_json_v1",
        camera_order: tuple[str, ...] | list[str] | None = None,
        control_frequency: float | None = None,
    ) -> None:
        from mcap.writer import Writer

        self._writer_type = Writer
        self.task_dir = os.path.abspath(task_dir)
        self.task_goal = str(task_goal)
        self.frequency = float(frequency)
        self.format_name = str(format_name)
        self.control_frequency = None if control_frequency is None else float(control_frequency)
        # An explicit empty tuple is allowed for state/action-only recordings.
        self.camera_order = tuple(self.CAMERA_ORDER if camera_order is None else camera_order)
        if len(set(self.camera_order)) != len(self.camera_order):
            raise ValueError(f"camera_order contains duplicates: {self.camera_order}")
        os.makedirs(self.task_dir, exist_ok=True)
        self.episode_id = self._find_last_episode_id()
        self.file_handle = None
        self.writer = None
        self.frame_channel_id = None
        self.control_channel_id = None
        self.event_channel_id = None
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
                "control_seq": {"type": ["integer", "null"]},
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
        if self.control_frequency is not None:
            control_schema_id = self.writer.register_schema(
                name="r1arm.control_sample",
                encoding="jsonschema",
                data=json.dumps({"type": "object"}, ensure_ascii=False).encode("utf-8"),
            )
            self.control_channel_id = self.writer.register_channel(
                topic="/r1arm/control_sample", message_encoding="json", schema_id=control_schema_id
            )
            event_schema_id = self.writer.register_schema(
                name="r1arm.record_event",
                encoding="jsonschema",
                data=json.dumps({"type": "object"}, ensure_ascii=False).encode("utf-8"),
            )
            self.event_channel_id = self.writer.register_channel(
                topic="/r1arm/record_event", message_encoding="json", schema_id=event_schema_id
            )

        timestamp_ns = time.time_ns()
        meta = {
            "info": {
                "format": self.format_name,
                "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
                "frequency_hz": self.frequency,
                **({
                    "control_frequency_hz": self.control_frequency,
                    "control_sample_topic": "/r1arm/control_sample",
                    "control_sample_semantics": "one complete controller snapshot per control_seq; record_segment separates pause/resume intervals",
                } if self.control_frequency is not None else {}),
                "camera_order": list(self.camera_order),
                "clock_contract": {
                    "sample_time_ns": "host wall clock ns",
                    "monotonic_time_ns": "host monotonic ns; only comparable within this process boot",
                    "camera_received_time_ns": "host wall clock at frame receive/decode; not exposure time",
                    "camera_device_timestamp_ms": "device clock when backend exposes one; not mapped to host clock",
                },
                "camera_positions": [
                    name.removeprefix("color_").split("_", 1)[-1]
                    for name in self.camera_order
                ],
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
        sample_time_ns: int | None = None,
        camera_frame_times_ns: dict[str, int] | None = None,
        control_seq: int | None = None,
        camera_frame_ids: dict[str, int] | None = None,
        camera_frame_times_monotonic_ns: dict[str, int] | None = None,
        camera_device_timestamps_ms: dict[str, float | None] | None = None,
        record_segment: int = 0,
        episode_id: int | None = None,
    ) -> None:
        """Append one sample. ``frames`` must follow ``self.camera_order``.

        ``camera_frame_times_ns``（P3 对齐）: 每路相机该帧的到达时刻
        （``time.time_ns()`` 基准）。与 ``sample_time_ns``（本样本写入
        时刻）的差 = 每路图像相对 action 的实际偏移，可测而非假设。
        """
        if self.writer is None or self.frame_channel_id is None:
            return
        if len(frames) != len(self.camera_order):
            raise ValueError(
                f"Expected {len(self.camera_order)} camera frames, got {len(frames)}"
            )

        self.item_id += 1
        timestamp_ns = time.time_ns() if sample_time_ns is None else int(sample_time_ns)
        colors = {
            name: self._encode_color(frame)
            for name, frame in zip(self.camera_order, frames)
        }
        message = {
            "idx": self.item_id,
            "sample_time_ns": timestamp_ns,
            "image_source_frame_id": int(image_source_frame_id),
            "control_seq": None if control_seq is None else int(control_seq),
            "record_segment": int(record_segment),
            "episode_id": None if episode_id is None else int(episode_id),
            "camera_frame_ids": ({str(k): int(v) for k, v in camera_frame_ids.items()}
                                  if camera_frame_ids else {}),
            "camera_frame_times_ns": (
                {str(k): int(v) for k, v in camera_frame_times_ns.items()}
                if camera_frame_times_ns else {}
            ),
            "camera_frame_times_monotonic_ns": (
                {str(k): int(v) for k, v in camera_frame_times_monotonic_ns.items()}
                if camera_frame_times_monotonic_ns else {}
            ),
            "camera_device_timestamps_ms": (
                {str(k): (None if v is None else float(v))
                 for k, v in camera_device_timestamps_ms.items()}
                if camera_device_timestamps_ms else {}
            ),
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

    def add_event(self, event: dict) -> None:
        """Persist recorder lifecycle events on their own MCAP topic."""
        if self.writer is None or self.event_channel_id is None:
            return
        message = dict(event)
        timestamp_ns = int(message.get("time_ns", time.time_ns()))
        message.setdefault("time_ns", timestamp_ns)
        self.writer.add_message(
            channel_id=self.event_channel_id,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(message, ensure_ascii=False).encode("utf-8"),
        )

    def add_control_sample(
        self, sample: dict, record_segment: int = 0, episode_id: int | None = None,
        record_generation: int | None = None,
    ) -> None:
        """Append one complete high-rate controller snapshot, without an image."""
        if self.writer is None or self.control_channel_id is None:
            return
        message = dict(sample)
        message["record_segment"] = int(record_segment)
        if episode_id is not None:
            message["episode_id"] = int(episode_id)
        if record_generation is not None:
            message["record_generation"] = int(record_generation)
        timestamp_ns = int(message["control_time_ns"])
        self.writer.add_message(
            channel_id=self.control_channel_id,
            log_time=timestamp_ns,
            publish_time=timestamp_ns,
            data=json.dumps(
                message,
                ensure_ascii=False,
                default=lambda value: value.tolist() if hasattr(value, "tolist") else value.item(),
            ).encode("utf-8"),
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
        self.control_channel_id = None
        self.event_channel_id = None

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
