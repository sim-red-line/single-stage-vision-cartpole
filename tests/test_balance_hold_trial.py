import copy

import pytest

from experiments.run_upright_direction_trial import swing_trial_hard_limit, verified_balance_observation


def ready():
    return {"state": {"theta_rad": 0.0, "timestamp_s": 100.0, "valid": True},
            "swing_up": {"mode": "balance"}, "safety_state": "RUNNING",
            "tracking": {"status": "ok"}, "system_health": {"safe_for_control": True},
            "motor": {"connected": True, "homed": True, "watchdog_ok": True,
                      "estop": False, "fault": "", "left_limit": False, "right_limit": False}}


def test_fifteen_second_hold_has_finite_total_and_parent_cleanup_budget():
    assert swing_trial_hard_limit(30, 5) == 35.5
    assert swing_trial_hard_limit(30, 15) == 45.5
    assert swing_trial_hard_limit(30, 15) + 9.5 == 55


@pytest.mark.parametrize("module_name", ["run_upright_direction_trial", "run_scheduled_swing_up"])
@pytest.mark.parametrize("value", ["nan", "inf", "0", "15.01", "invalid"])
def test_invalid_hold_never_contacts_hardware(monkeypatch, module_name, value):
    import importlib
    module = importlib.import_module("experiments." + module_name)
    args = ["trial", "--swing-up"] if module_name == "run_upright_direction_trial" else ["trial", "--start-at-unix", "1120"]
    monkeypatch.setattr("sys.argv", args + ["--balance-hold-seconds", value])
    def forbidden(*args, **kwargs):
        raise AssertionError("invalid hold must not contact hardware")
    monkeypatch.setattr(module, "request", forbidden)
    with pytest.raises(SystemExit) as stopped:
        module.main()
    assert stopped.value.code == 2


def test_stale_or_future_camera_frame_does_not_count_toward_hold():
    assert verified_balance_observation(ready(), 100.03)
    assert not verified_balance_observation(ready(), 100.121)
    assert not verified_balance_observation(ready(), 99.99)


@pytest.mark.parametrize("section,key,value", [
    ("state", "valid", False), ("state", "theta_rad", .10),
    ("swing_up", "mode", "swing_up"), ("tracking", "status", "lost"),
    ("motor", "watchdog_ok", False), ("motor", "estop", True),
    ("motor", "homed", False), ("system_health", "safe_for_control", False),
])
def test_hold_requires_raw_vision_balance_mode_and_live_hardware(section, key, value):
    status = copy.deepcopy(ready())
    status[section][key] = value
    assert not verified_balance_observation(status, 100.03)
