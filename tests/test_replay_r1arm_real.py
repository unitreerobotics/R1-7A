import json
from types import SimpleNamespace

import numpy as np

import scripts.replay_r1arm_real as replay
import teleop.r1arm_mujoco_controller as mujoco_controller


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


def test_replay_shaper_speed_override_is_a_separate_cli_option():
    args = replay.build_parser().parse_args([
        "--episode", "episode.mcap", "--reshape-raw-ik",
        "--trajectory-max-speed", "1.2",
    ])
    assert args.trajectory_max_speed == 1.2


def test_reshape_raw_ik_starts_at_recorded_home_and_uses_production_shaper(monkeypatch):
    seen = {}

    class FakeController:
        def __init__(self, config, **_kwargs):
            seen["config"] = config
            self.arm_q_indices = list(range(7))
            self.q = np.zeros(7)

        def _create_initial_q(self):
            return np.zeros(7)

        def _reset_trajectory_state(self, _side, q):
            self.command = q[:7].copy()
            self.velocity = np.zeros(7)

        def _trajectory_step(self, _side, q, desired, dt):
            self.velocity = (desired[:7] - self.command) * min(dt * 10.0, 1.0)
            self.command += self.velocity
            return self.command.copy()

        def _trajectory_state(self, _side, _q):
            return self.command.copy(), self.velocity.copy(), np.zeros(7)

        def _on_stop(self):
            seen["closed"] = True

    monkeypatch.setattr(mujoco_controller, "R1ArmMujocoTeleopController", FakeController)
    frames = [
        {"idx": 0, "actions": {"q_ik_raw": [0.0] * 7}},
        {"idx": 1, "actions": {"q_ik_raw": [0.2] * 7}},
    ]
    q0 = np.full(7, 0.1)
    shaped, velocity = replay.reshape_raw_ik_trajectory(
        frames, q0, np.array([0.0, 1.0 / 30.0]),
        speed_limit=1.0, acceleration_limit=6.0, jerk_limit=80.0,
        position_gain=20.0, velocity_gain=9.0, rate_hz=30.0,
    )

    np.testing.assert_allclose(shaped[0], q0)
    assert np.all(shaped[1] > shaped[0])
    assert np.all(velocity[1] > 0.0)
    assert seen["config"].trajectory_position_gain == 20.0
    assert seen["config"].trajectory_velocity_gain == 9.0
    assert seen["closed"]
