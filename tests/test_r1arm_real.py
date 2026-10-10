from __future__ import annotations

from pathlib import Path
import threading
import time

import numpy as np
from types import SimpleNamespace

from robot.r1arm_sdk import model_to_sdk_q, sdk_to_model_q, validate_calibration_file
import teleop.r1arm_controller_base as controller_base
import teleop.r1arm_real_controller as real_controller
from robot.eef.dex1_single import Dex1Single
from teleop.r1arm_controller_base import R1ArmTeleopControllerBase
from teleop.r1arm_real_controller import (
    R1ArmQPConstrainedIKTeleopController,
    R1ArmUnshapedIKTeleopController,
    SDK_LOWER,
    SDK_UPPER,
    limit_sdk_position_command,
)
from teleop.r1arm_placo_solver import PlacoArmIKSolver, joint_delta_bounds
from robot.r1arm_config import ARM_JOINTS, DEX1_JOINTS, DEFAULT_URDF_PATH


def test_model_sdk_joint_mapping_only_flips_j4() -> None:
    model = np.array([0.1, 0.2, 0.3, -0.4, 0.5, -0.2, 0.7])
    sdk = model_to_sdk_q(model)
    np.testing.assert_allclose(sdk, [0.1, 0.2, 0.3, 0.4, 0.5, -0.2, 0.7])
    np.testing.assert_allclose(sdk_to_model_q(sdk), model)


def test_real_motion_does_not_publish_incomplete_ik_snapshot() -> None:
    controller = R1ArmTeleopControllerBase.__new__(R1ArmTeleopControllerBase)
    controller.motion_mode = True
    controller._control_seq = 10
    complete_sample = {"control_seq": 9, "q_measured": np.zeros(7)}
    controller._control_sample = complete_sample
    controller._control_sample_lock = threading.Lock()

    controller._publish_ik_diagnostics({}, xr_valid=True)

    assert controller._control_seq == 10
    sample = controller.control_sample()
    assert sample["control_seq"] == 9
    np.testing.assert_array_equal(sample["q_measured"], np.zeros(7))


def test_control_capture_preserves_each_cycle_and_can_flush_on_stop() -> None:
    controller = R1ArmTeleopControllerBase.__new__(R1ArmTeleopControllerBase)
    controller._control_sample = None
    controller._control_sample_lock = threading.Lock()
    controller._capture_control_samples = False
    controller._captured_control_samples = controller_base.deque()
    controller.start_control_sample_capture()
    for seq in range(3):
        controller.publish_control_sample({"control_seq": seq, "q": np.array([seq])})

    samples, latest = controller.drain_control_sample_capture(stop=True)

    assert [sample["control_seq"] for sample in samples] == [0, 1, 2]
    assert latest["control_seq"] == 2
    assert controller.drain_control_sample_capture()[0] == []


def test_sdk_limits_match_positive_j4_convention() -> None:
    assert np.isclose(SDK_LOWER[3], 0.0)
    assert np.isclose(SDK_UPPER[3], 3.1415)


def test_command_limiter_applies_limits_slew_and_measured_lead() -> None:
    previous = np.zeros(7)
    measured = np.zeros(7)
    target = np.array([9.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    command = limit_sdk_position_command(
        target,
        previous,
        measured,
        max_delta=0.02,
        max_lead=0.01,
    )
    np.testing.assert_allclose(command, np.full(7, 0.01))


def test_command_limiter_never_exceeds_slew_when_lead_window_is_infeasible() -> None:
    previous = np.zeros(7)
    measured = np.full(7, 0.2)
    target = np.full(7, 0.5)
    command = limit_sdk_position_command(
        target, previous, measured, max_delta=0.02, max_lead=0.15
    )
    assert np.max(np.abs(command - previous)) <= 0.02 + 1e-9
    assert np.all(command >= SDK_LOWER) and np.all(command <= SDK_UPPER)


def test_trajectory_generator_brakes_without_snapping_velocity(monkeypatch) -> None:
    monkeypatch.setattr(controller_base, "clamp_configuration", lambda _model, q: q)

    class DummyController(R1ArmTeleopControllerBase):
        pass

    controller = DummyController.__new__(DummyController)
    controller.config = SimpleNamespace(
        trajectory_max_speed_rad_s=1.0,
        ik_max_acc_per_s2=6.0,
        ik_max_jerk_per_s3=80.0,
        trajectory_position_gain=16.0,
        trajectory_velocity_gain=8.0,
    )
    controller.rate_hz = 30.0
    controller._step_dt = 1.0 / 30.0
    controller.arm_q_indices = list(range(7))
    controller.model = object()
    controller._trajectory_q_cmd = {"right": np.zeros(7)}
    controller._trajectory_v_cmd = {"right": np.array([1.0, 0, 0, 0, 0, 0, 0])}
    controller._trajectory_a_cmd = {"right": np.zeros(7)}

    q_next = controller._trajectory_step("right", np.zeros(7), np.zeros(7))
    assert 0.0 < controller._trajectory_v_cmd["right"][0] < 1.0
    assert controller._trajectory_a_cmd["right"][0] == -80.0 / 30.0
    assert q_next[0] > 0.0


def test_velocity_only_shaper_limits_speed_without_acceleration_bound(monkeypatch) -> None:
    monkeypatch.setattr(controller_base, "clamp_configuration", lambda _model, q: q)
    controller = R1ArmTeleopControllerBase.__new__(R1ArmTeleopControllerBase)
    controller.config = SimpleNamespace(trajectory_max_speed_rad_s=1.0)
    controller.rate_hz = 10.0
    controller._step_dt = 0.1
    controller.arm_q_indices = list(range(7))
    controller.model = object()
    controller._trajectory_q_cmd = {"right": np.zeros(7)}
    controller._trajectory_v_cmd = {"right": np.zeros(7)}
    controller._trajectory_a_cmd = {"right": np.zeros(7)}

    first = controller._velocity_only_step("right", np.zeros(7), np.ones(7))
    second = controller._velocity_only_step("right", first, -np.ones(7))
    assert np.allclose(first, 0.1)
    assert np.allclose(second, 0.0)
    assert np.max(np.abs(controller._trajectory_v_cmd["right"])) <= 1.0 + 1e-9
    assert controller._trajectory_a_cmd["right"][0] == -20.0


def test_idle_tracking_initializes_trajectory_state_before_stopping(monkeypatch) -> None:
    controller = R1ArmTeleopControllerBase.__new__(R1ArmTeleopControllerBase)
    controller.track = {"right": controller_base._SideTrackState()}
    controller.control_side = "right"
    controller.dual_arm = False
    controller.q = np.zeros(7)
    controller.arm_q_indices = list(range(7))
    controller._last_ee_intent = {}
    controller._last_ik_raw = {}
    controller._last_ik_solver = {}
    controller._trajectory_q_cmd = {}
    controller._trajectory_v_cmd = {}
    controller._trajectory_a_cmd = {}
    controller._solver_seeds = {"right": controller.q.copy()}
    controller.get_grip_state = lambda _side: 0.0
    controller._trajectory_step = lambda _side, q, _target: q.copy()
    controller._set_q_for_side = lambda _side, q: setattr(controller, "q", q)

    controller._track_side("right", {})

    np.testing.assert_array_equal(controller._trajectory_v_cmd["right"], np.zeros(7))
    assert controller._last_ik_solver["right"] == "hold"


def test_trajectory_speed_limit_is_in_rad_per_second(monkeypatch) -> None:
    monkeypatch.setattr(controller_base, "clamp_configuration", lambda _model, q: q)

    class DummyController(R1ArmTeleopControllerBase):
        pass

    controller = DummyController.__new__(DummyController)
    controller.config = SimpleNamespace(
        trajectory_max_speed_rad_s=0.4,
        ik_max_acc_per_s2=6.0,
        ik_max_jerk_per_s3=80.0,
        trajectory_position_gain=20.0,
        trajectory_velocity_gain=9.0,
    )
    controller.rate_hz = 30.0
    controller._step_dt = 1.0 / 30.0
    controller.arm_q_indices = list(range(7))
    controller.model = object()
    controller._trajectory_q_cmd = {"right": np.zeros(7)}
    controller._trajectory_v_cmd = {"right": np.zeros(7)}
    controller._trajectory_a_cmd = {"right": np.zeros(7)}

    for _ in range(120):
        controller._trajectory_step("right", np.zeros(7), np.full(7, 10.0))

    np.testing.assert_allclose(controller._trajectory_v_cmd["right"], np.full(7, 0.4))


def test_trajectory_generator_settles_at_fixed_home_target(monkeypatch) -> None:
    monkeypatch.setattr(controller_base, "clamp_configuration", lambda _model, q: q)

    class DummyController(R1ArmTeleopControllerBase):
        pass

    controller = DummyController.__new__(DummyController)
    controller.config = SimpleNamespace(
        trajectory_max_speed_rad_s=1.0,
        ik_max_acc_per_s2=6.0,
        ik_max_jerk_per_s3=80.0,
        trajectory_position_gain=16.0,
        trajectory_velocity_gain=8.0,
    )
    controller.rate_hz = 30.0
    controller._step_dt = 1.0 / 30.0
    controller.arm_q_indices = list(range(7))
    controller.model = object()
    q = np.zeros(7)
    q[0] = 0.1
    controller._trajectory_q_cmd = {"right": q.copy()}
    controller._trajectory_v_cmd = {"right": np.zeros(7)}
    controller._trajectory_a_cmd = {"right": np.zeros(7)}
    controller.q = q
    controller.q_left = None
    controller.dual_arm = False
    controller._q_blend_start = q.copy()
    controller._q_blend_goal = np.zeros(7)
    controller._q_left_blend_start = None
    controller._q_left_blend_goal = None
    controller._reset_blending = True
    controller._reset_blend_deadline = 0.0
    controller._reset_last_diag_time = 0.0
    measured_q = np.full(7, 0.05)
    controller._reset_feedback_state = lambda _side: (measured_q.copy(), np.zeros(7))
    controller._solver_seeds = {"right": q.copy()}
    controller._last_ik_raw = {"right": np.zeros(7)}
    controller._last_ik_solver = {}
    controller.grippers = {"right": SimpleNamespace(current=0.0, reset=lambda: None)}
    controller.dex1_q_indices = []
    controller.hand_ctrl = None
    controller.debug = False

    for _ in range(120):
        if not controller._step_reset_blend():
            break

    assert controller._reset_blending  # command reached home, measured arm did not
    measured_q[:] = 0.0
    measured_q[3] = 0.04  # home tolerance matches replay's default approach tolerance
    for _ in range(120):
        if not controller._step_reset_blend():
            break

    assert not controller._reset_blending
    np.testing.assert_allclose(controller._trajectory_q_cmd["right"], np.zeros(7), atol=1e-3)
    np.testing.assert_allclose(controller._trajectory_v_cmd["right"], np.zeros(7), atol=0.02)
    np.testing.assert_allclose(controller._trajectory_a_cmd["right"], np.zeros(7), atol=0.1)


def test_trajectory_speed_bound_wins_when_jerk_interval_is_infeasible(monkeypatch) -> None:
    monkeypatch.setattr(controller_base, "clamp_configuration", lambda _model, q: q)

    class DummyController(R1ArmTeleopControllerBase):
        pass

    controller = DummyController.__new__(DummyController)
    controller.config = SimpleNamespace(
        trajectory_max_speed_rad_s=1.0,
        ik_max_acc_per_s2=6.0,
        ik_max_jerk_per_s3=80.0,
        trajectory_position_gain=20.0,
        trajectory_velocity_gain=9.0,
    )
    controller.rate_hz = 30.0
    controller._step_dt = 1.0 / 30.0
    controller.arm_q_indices = list(range(7))
    controller.model = object()
    controller._trajectory_q_cmd = {"right": np.zeros(7)}
    controller._trajectory_v_cmd = {"right": np.array([-1.0, 0, 0, 0, 0, 0, 0])}
    controller._trajectory_a_cmd = {"right": np.array([-3.94, 0, 0, 0, 0, 0, 0])}

    controller._trajectory_step("right", np.zeros(7), np.full(7, -10.0))

    assert np.max(np.abs(controller._trajectory_v_cmd["right"])) <= 1.0 + 1e-9


def test_trajectory_follower_gains_change_position_and_velocity_response(monkeypatch) -> None:
    monkeypatch.setattr(controller_base, "clamp_configuration", lambda _model, q: q)

    class DummyController(R1ArmTeleopControllerBase):
        pass

    def step(position_gain: float, velocity_gain: float) -> float:
        controller = DummyController.__new__(DummyController)
        controller.config = SimpleNamespace(
            trajectory_max_speed_rad_s=1.0,
            ik_max_acc_per_s2=6.0,
            ik_max_jerk_per_s3=80.0,
            trajectory_position_gain=position_gain,
            trajectory_velocity_gain=velocity_gain,
        )
        controller.rate_hz = 30.0
        controller._step_dt = 1.0 / 30.0
        controller.arm_q_indices = list(range(7))
        controller.model = object()
        controller._trajectory_q_cmd = {"right": np.zeros(7)}
        controller._trajectory_v_cmd = {"right": np.array([0.1, 0, 0, 0, 0, 0, 0])}
        controller._trajectory_a_cmd = {"right": np.zeros(7)}
        controller._trajectory_step("right", np.zeros(7), np.array([0.05, 0, 0, 0, 0, 0, 0]))
        return float(controller._trajectory_a_cmd["right"][0])

    baseline = step(16.0, 8.0)
    assert np.isclose(baseline, 0.0)
    assert step(24.0, 8.0) > baseline
    assert step(16.0, 12.0) < baseline


def test_unshaped_ik_controller_passes_placo_solution_through(monkeypatch) -> None:
    class FakeSolver:
        def solve(self, _seed, _position, _orientation):
            result = np.zeros(7)
            result[0] = 0.05
            return result

    monkeypatch.setattr(real_controller, "clamp_configuration", lambda _model, q: q)
    controller = R1ArmUnshapedIKTeleopController.__new__(R1ArmUnshapedIKTeleopController)
    controller.dual_arm = False
    controller.q = np.zeros(7)
    controller.arm_q_indices = list(range(7))
    controller._solver_seeds = {"right": np.zeros(7)}
    controller._trajectory_q_cmd = {"right": np.zeros(7)}
    controller._trajectory_v_cmd = {"right": np.zeros(7)}
    controller._trajectory_a_cmd = {"right": np.zeros(7)}
    controller._last_ik_raw = {}
    controller._last_ik_solver = {}
    controller.config = SimpleNamespace(ik_max_acc_per_s2=6.0)
    controller.rate_hz = 30.0
    controller._step_dt = 1.0 / 30.0
    controller.use_placo = True
    controller.placo_solver = FakeSolver()
    controller.placo_solver_left = None
    controller.model = object()
    controller._trajectory_step = lambda *_args: (_ for _ in ()).throw(
        AssertionError("active IK must bypass the trajectory shaper")
    )

    controller._solve("right", np.zeros(3), None)

    assert controller.q[0] == 0.05
    assert controller._trajectory_q_cmd["right"][0] == 0.05
    assert controller._last_ik_solver["right"] == "placo"


def test_unshaped_dls_limits_one_solver_step_to_cycle_speed(monkeypatch) -> None:
    captured = {}

    def fake_solve(*args, **kwargs):
        captured.update(kwargs)
        return args[2].copy(), {}

    monkeypatch.setattr(controller_base, "solve_upper_arm_ik", fake_solve)

    class DummyController(R1ArmUnshapedIKTeleopController):
        pass

    controller = DummyController.__new__(DummyController)
    controller.model = object()
    controller.data = object()
    controller.arm_q_indices = list(range(7))
    controller.ee_frame = "ee"
    controller.debug = False
    controller._dls_max_step = 1.0 / 30.0
    controller._solve_dls(np.zeros(7), np.zeros(3), None)

    assert captured["max_step"] == 1.0 / 30.0
    assert captured["iterations"] == 1


def test_qp_velocity_acceleration_delta_bounds() -> None:
    lower, upper = joint_delta_bounds(
        np.zeros(2), np.ones(2), acceleration_limit=6.0, dt=1.0 / 30.0
    )
    np.testing.assert_allclose(lower, [-0.0066666667] * 2)
    np.testing.assert_allclose(upper, [0.0066666667] * 2)

    lower, upper = joint_delta_bounds(
        np.array([0.8]), np.array([1.0]), acceleration_limit=6.0, dt=1.0 / 30.0
    )
    np.testing.assert_allclose(lower, [0.02])
    np.testing.assert_allclose(upper, [1.0 / 30.0])

    lower, upper = joint_delta_bounds(
        np.zeros(1), np.ones(1), acceleration_limit=6.0, dt=1.0 / 30.0,
        previous_acceleration=np.zeros(1), jerk_limit=80.0,
    )
    jerk_delta = 80.0 / 30.0**3
    np.testing.assert_allclose(lower, [-jerk_delta])
    np.testing.assert_allclose(upper, [jerk_delta])

    # Existing positive acceleration can only increase within jerk * dt.
    lower, upper = joint_delta_bounds(
        np.zeros(1), np.ones(1), acceleration_limit=6.0, dt=1.0 / 30.0,
        previous_acceleration=np.array([1.0]), jerk_limit=80.0,
    )
    assert lower[0] > -jerk_delta
    assert upper[0] > jerk_delta


def test_placo_qp_enforces_acceleration_and_jerk_bounds_at_nonzero_pose() -> None:
    q = np.zeros(9)
    q[1], q[3] = 1.2, -1.2
    joint_indices = {name: i for i, name in enumerate(ARM_JOINTS + DEX1_JOINTS)}
    solver = PlacoArmIKSolver(
        DEFAULT_URDF_PATH, q, "dex1_base_link", joint_indices,
        arm_joints=ARM_JOINTS, hold_joints=DEX1_JOINTS,
        default_velocity_limit=1.0, default_acceleration_limit=6.0,
        default_jerk_limit=80.0,
    )
    target = solver.placo_robot.get_T_world_frame("dex1_base_link").copy()
    target[1, 3] += 0.05
    dt = 1.0 / 30.0

    result = solver.solve(
        q, target[:3, 3], None, dt=dt,
        previous_velocity=np.zeros(7), previous_acceleration=np.zeros(7),
    )

    assert result is not None
    lower, upper = solver.delta_bounds(dt, np.zeros(7), np.zeros(7))
    assert np.all(result[:7] - q[:7] >= lower - 1e-8)
    assert np.all(result[:7] - q[:7] <= upper + 1e-8)
    assert np.isclose(solver.solver.dt, dt)


def test_qp_dls_fallback_uses_same_acceleration_box(monkeypatch) -> None:
    monkeypatch.setattr(real_controller, "clamp_configuration", lambda _model, q: q)

    class FailedPlaco:
        def solve(self, *_args, **_kwargs):
            return None

        def delta_bounds(self, dt, velocity, acceleration=None):
            return joint_delta_bounds(
                velocity, np.ones(7), 6.0, dt,
                previous_acceleration=acceleration, jerk_limit=80.0,
            )

    controller = R1ArmQPConstrainedIKTeleopController.__new__(
        R1ArmQPConstrainedIKTeleopController
    )
    controller.dual_arm = False
    controller.q = np.zeros(9)
    controller.q_left = None
    controller.arm_q_indices = list(range(7))
    controller._solver_seeds = {"right": np.zeros(9)}
    controller._trajectory_q_cmd = {"right": np.zeros(7)}
    controller._trajectory_v_cmd = {"right": np.zeros(7)}
    controller._trajectory_a_cmd = {"right": np.zeros(7)}
    controller._last_ik_raw = {}
    controller._last_ik_solver = {}
    controller._qp_action_state = {}
    controller._qp_velocity_limits = np.ones(7)
    controller._qp_acceleration_limit = 6.0
    controller._qp_jerk_limit = 80.0
    controller.config = SimpleNamespace(ik_max_acc_per_s2=6.0)
    controller.rate_hz = 30.0
    controller._step_dt = 1.0 / 30.0
    controller.use_placo = True
    controller.placo_solver = FailedPlaco()
    controller.placo_solver_left = None
    controller.model = object()
    controller._solve_dls = lambda *_args: np.r_[0.2, np.zeros(8)]

    controller._solve("right", np.zeros(3), None)

    assert controller._last_ik_solver["right"] == "dls_vaj_projected"
    assert np.isclose(controller.q[0], 80.0 / 30.0**3)
    assert np.isclose(controller._trajectory_v_cmd["right"][0], 80.0 / 30.0**2)


def test_invalid_trigger_sample_reuses_last_valid_input() -> None:
    class BrokenXR:
        def get_key_value_by_name(self, _name: str) -> float:
            raise RuntimeError("XR sample unavailable")

    class DummyController(R1ArmTeleopControllerBase):
        pass

    controller = DummyController.__new__(DummyController)
    controller.mock_xr = False
    controller.xr_client = BrokenXR()
    controller._trigger_samples = {"right": (0.7, True, 1.0)}
    value, valid = controller.get_trigger_sample("right")
    assert value == 0.7
    assert not valid


def test_dex1_feedback_diagnostics_report_stale_cache() -> None:
    hand = Dex1Single.__new__(Dex1Single)
    hand._lock = threading.Lock()
    hand._q = 2.5
    hand._tau = 0.1
    hand._state_received_monotonic = time.monotonic() - 1.0
    hand._state_received_time_ns = 123
    hand._command_sent_monotonic = time.monotonic()
    hand._command_sent_time_ns = 456
    hand._command_count = 7
    hand._command_publish_ok = True
    hand._command_publish_fail_count = 0
    hand.last_command = 1.0

    data = hand.get_record_data()

    assert data["feedback_valid"] is False
    assert data["feedback_age_s"] >= 1.0
    assert data["feedback_time_ns"] == 123
    assert data["command_count"] == 7
    assert data["command_time_ns"] == 456


def test_dex1_records_failed_dds_publish() -> None:
    hand = Dex1Single.__new__(Dex1Single)
    hand._lock = threading.Lock()
    hand._q = 5.0
    hand._tau = 0.0
    hand._tau_zero = 0.0
    hand._tau_filtered = 0.0
    hand._grip_torque_limit = -1.0
    hand._alpha = 0.85
    hand._latched_q = None
    hand._state_received = True
    hand._state_received_monotonic = time.monotonic()
    hand._state_received_time_ns = 123
    hand._command_count = 0
    hand._command_publish_fail_count = 0
    hand._command_publish_ok = None
    hand.last_command = 5.0
    hand.message = SimpleNamespace(cmds=[SimpleNamespace(q=5.0)])
    hand.publisher = SimpleNamespace(Write=lambda _message: False)

    hand.set_closed_ratio(1.0)
    data = hand.get_record_data()

    assert data["command_publish_ok"] is False
    assert data["command_publish_fail_count"] == 1
    assert data["command_count"] == 1


def test_calibration_validation_accepts_all_seven_joints(tmp_path: Path) -> None:
    calibration = tmp_path / "joint_calib.txt"
    calibration.write_text(
        "# joint dir scale offset\n"
        + "".join(f"{joint} 1 1 0\n" for joint in range(7)),
        encoding="utf-8",
    )
    assert validate_calibration_file(calibration) == calibration.resolve()


def test_calibration_validation_rejects_missing_joint(tmp_path: Path) -> None:
    calibration = tmp_path / "joint_calib.txt"
    calibration.write_text("0 1 1 0\n", encoding="utf-8")
    try:
        validate_calibration_file(calibration)
    except ValueError as exc:
        assert "joints 0..6" in str(exc)
    else:
        raise AssertionError("Missing calibration joints were accepted")
