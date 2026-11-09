import math

from experiments.analyze_baseline_repeats import summarize_monitor


def recording(mode="balance", angle=math.radians(2)):
    return {"result": {"final_motor_target_mm_s": 0, "final_motor_velocity_mm_s": 0},
            "samples": [{"t": 100 + i*.1, "state_timestamp_s": 100 + i*.1 - .02,
                         "theta": angle, "mode": mode, "safety": "RUNNING", "limited": 0}
                        for i in range(61)]}


def test_upright_flyby_is_not_scored_as_pid_stability():
    result = summarize_monitor(recording(mode="swing_up"))
    assert not result["captured"]
    assert result["continuous_stable_balance_s"] == 0
    assert result["balance_rms_deg"] is None
    assert result["pid_assessment"] == "not_evaluable_no_capture"


def test_actual_balance_uses_periodic_upright_angle():
    result = summarize_monitor(recording(angle=2*math.pi + math.radians(2)))
    assert result["captured"]
    assert result["continuous_stable_balance_s"] == 6
    assert abs(result["balance_rms_deg"] - 2) < 1e-9
    assert result["stopped"]


def test_invalid_state_and_observation_gap_break_balance_continuity():
    data = recording()
    data["samples"][30]["theta"] = None
    result = summarize_monitor(data)
    assert result["continuous_stable_balance_s"] < 3
    data = recording()
    del data["samples"][25:31]
    result = summarize_monitor(data)
    assert result["continuous_stable_balance_s"] < 3


def test_stale_sample_does_not_confirm_balance():
    data = recording()
    for sample in data["samples"]:
        sample["state_timestamp_s"] -= .5
    result = summarize_monitor(data)
    assert result["verified_balance_time_s"] == 0
    assert result["continuous_stable_balance_s"] == 0
