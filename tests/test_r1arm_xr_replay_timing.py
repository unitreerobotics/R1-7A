from scripts.replay_r1arm_xr_shaper_sweep import resample_control_frames


def _frame(time_ns: int, idx: int) -> dict:
    return {
        "control_time_ns": time_ns,
        "sample_time_ns": time_ns,
        "idx": idx,
    }


def test_resample_control_frames_zero_order_holds_latest_source_sample():
    frames = [_frame(0, 0), _frame(20_000_000, 1), _frame(45_000_000, 2)]

    replay = resample_control_frames(frames, 50.0)

    assert [frame["idx"] for frame in replay] == [0, 1, 1, 2]
    assert [round(frame["_replay_time_s"], 3) for frame in replay] == [0.0, 0.02, 0.04, 0.045]
    assert [round(frame["_replay_dt_s"], 3) for frame in replay] == [0.02, 0.02, 0.02, 0.005]
