from __future__ import annotations

from pathlib import Path

import numpy as np

from robot.r1arm_sdk import model_to_sdk_q, sdk_to_model_q, validate_calibration_file
from teleop.r1arm_real_controller import SDK_LOWER, SDK_UPPER, limit_sdk_position_command


def test_model_sdk_joint_mapping_only_flips_j4() -> None:
    model = np.array([0.1, 0.2, 0.3, -0.4, 0.5, -0.2, 0.7])
    sdk = model_to_sdk_q(model)
    np.testing.assert_allclose(sdk, [0.1, 0.2, 0.3, 0.4, 0.5, -0.2, 0.7])
    np.testing.assert_allclose(sdk_to_model_q(sdk), model)


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
