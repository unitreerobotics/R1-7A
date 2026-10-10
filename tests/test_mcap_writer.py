from __future__ import annotations

import json

from mcap.reader import make_reader

from teleop.mcap_writer import FourCameraMcapRecorder


def test_control_sample_topic_is_additive_and_round_trips(tmp_path) -> None:
    recorder = FourCameraMcapRecorder(
        task_dir=str(tmp_path),
        task_goal="test",
        frequency=30,
        camera_order=(),
        control_frequency=100,
    )
    path = recorder.start_episode({"test": True})
    recorder.add_control_sample(
        {"control_seq": 17, "control_time_ns": 123456, "q_purpose": [0.1, 0.2]},
        record_segment=2,
    )
    recorder.stop_episode()

    with open(path, "rb") as stream:
        messages = list(make_reader(stream).iter_messages())
    by_topic = {channel.topic: (schema, message) for schema, channel, message in messages}
    assert set(by_topic) == {"/episode/meta", "/r1arm/control_sample"}
    meta = json.loads(by_topic["/episode/meta"][1].data)
    assert meta["info"]["frequency_hz"] == 30.0
    assert meta["info"]["control_frequency_hz"] == 100.0
    sample = json.loads(by_topic["/r1arm/control_sample"][1].data)
    assert sample == {
        "control_seq": 17,
        "control_time_ns": 123456,
        "q_purpose": [0.1, 0.2],
        "record_segment": 2,
    }
