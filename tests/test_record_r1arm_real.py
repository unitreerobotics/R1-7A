from scripts.record_r1arm_real import (
    _record_sample_is_complete,
    _stale_camera_keys,
    _stale_camera_summary,
)


def test_record_skips_initial_ik_only_sample() -> None:
    assert not _record_sample_is_complete({"control_seq": 1, "q_purpose": [0.0] * 7})


def test_record_accepts_complete_control_sample() -> None:
    fields = (
        "control_seq", "control_time_ns", "q_purpose", "q_measured", "q_ik_raw",
        "q_motor_target", "dq_purpose", "ddq_purpose", "dq_target",
        "dq_measured", "tau_est",
    )
    sample = {key: 1 for key in fields}
    assert _record_sample_is_complete(sample)


def test_stale_camera_diagnostic_reports_stale_selected_streams() -> None:
    class Camera:
        active_keys = ("wrist",)

        def frame_ages(self):
            return {"wrist": 1.2}

        def health(self):
            return {
                "wrist": ("open", 1.2),
                "head": ("unavailable", float("inf")),
            }

    camera = Camera()
    assert _stale_camera_keys(camera, 1.0) == ["wrist"]
    assert "wrist" in _stale_camera_summary(camera, 1.0)
    assert "head" not in _stale_camera_summary(camera, 1.0)
