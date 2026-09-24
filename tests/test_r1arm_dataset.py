from __future__ import annotations

import numpy as np

from teleop.r1arm_dataset import R1ArmDatasetAdapter


def test_single_arm_uses_compact_eight_dof_space() -> None:
    adapter = R1ArmDatasetAdapter(dual_arm=False, control_side="right")
    measured = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02, 0.024])
    commanded = np.array([1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, -0.01, -0.01])
    states, actions = adapter.frame_data(measured, commanded)

    assert adapter.vector_size == 8
    assert "body" not in states
    assert "body" not in actions
    np.testing.assert_allclose(states["joint_state"], [*measured[:7], 0.022])
    np.testing.assert_allclose(states["robot_q_current"], states["joint_state"])
    np.testing.assert_allclose(actions["action"], [*commanded[:7], -0.01])
    np.testing.assert_allclose(actions["robot_q_desired"], actions["action"])
    assert len(states["hand_state"]) == 1
    assert len(actions["hand_cmd"]) == 1


def test_dual_arm_compact_order_matches_sim_order_and_replays() -> None:
    adapter = R1ArmDatasetAdapter(dual_arm=True, control_side="right")
    # MuJoCo order: right arm(7), right fingers(2), left arm(7), left fingers(2).
    right = np.hstack([np.arange(7, dtype=float), [0.02, 0.02]])
    left = np.hstack([np.arange(7, dtype=float) + 10.0, [-0.01, -0.01]])
    measured = np.hstack([right, left])
    commanded = measured + 0.001
    states, actions = adapter.frame_data(measured, commanded)
    frame = {"states": states, "actions": actions}

    assert adapter.vector_size == 16
    np.testing.assert_allclose(states["joint_state"], [*right[:7], 0.02, *left[:7], -0.01])
    np.testing.assert_allclose(adapter.sim_q_from_frame(frame, "state"), measured)
    np.testing.assert_allclose(adapter.sim_q_from_frame(frame, "action"), commanded)
    assert len(states["arms"]) == 14
    assert len(states["hand_state"]) == 2


def test_single_gripper_expands_to_two_mujoco_fingers() -> None:
    adapter = R1ArmDatasetAdapter(dual_arm=False, control_side="left")
    compact = np.linspace(-0.2, 0.5, 8)
    frame = {
        "states": {"joint_state": compact.tolist()},
        "actions": {"action": compact.tolist()},
    }
    expanded = adapter.sim_q_from_frame(frame, "state")

    assert expanded.shape == (9,)
    np.testing.assert_allclose(expanded[:7], compact[:7])
    np.testing.assert_allclose(expanded[7:], [compact[7], compact[7]])
