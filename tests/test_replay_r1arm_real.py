import json
from types import SimpleNamespace

import numpy as np

import scripts.replay_r1arm_real as replay


def test_action_replay_prefers_shaped_q_purpose(monkeypatch, tmp_path):
    path = tmp_path / "episode_0000.mcap"
    path.write_bytes(b"test")
    metadata = {"config": {"state_dim": 8}}
    frame = {
        "idx": 3,
        "actions": {
            "q_purpose": [0.1] * 8,
            "action": [0.2] * 8,
        },
    }

    class FakeReader:
        def iter_messages(self):
            for topic, value in (("/episode/meta", metadata), ("/whole_body/frame", frame)):
                yield None, SimpleNamespace(topic=topic), SimpleNamespace(
                    data=json.dumps(value).encode("utf-8")
                )

    monkeypatch.setattr(replay, "make_reader", lambda _stream: FakeReader())

    actions, frames, _, _ = replay.load_episode_states(path, "action")

    np.testing.assert_allclose(actions[0], np.full(8, 0.1))
    assert frames[0]["idx"] == 3
