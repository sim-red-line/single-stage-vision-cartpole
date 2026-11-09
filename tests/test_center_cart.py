import math
import time

import pytest

from experiments.center_cart import check_ready
from raspberry_pi.config import load_config


@pytest.fixture
def ready():
    config = load_config()
    config["controller"]["cart_center_mm"] = 256.369
    config["controller"]["swing_up"]["max_cart_excursion_mm"] = 80
    status = {
        "safety_state": "IDLE", "experiment": {"active": None},
        "system_health": {"safe_for_control": True}, "tracking": {"status": "ok"},
        "motor": {"connected": True, "homed": True, "watchdog_ok": True,
                  "fault": "", "estop": False, "left_limit": False, "right_limit": False,
                  "position_mm": 340.5, "velocity_mm_s": 0.0, "target_velocity_mm_s": 0.0,
                  "raw": {"firmware": "cartpole-safe-8"}},
        "state": {"valid": True, "timestamp_s": time.monotonic(), "theta_rad": math.pi,
                  "omega_rad_s": 0.0, "cart_position_mm": 339.0},
    }
    return status, config


def test_center_recovery_uses_existing_rail_limits_without_widening_swing_excursion(ready):
    status, config = ready
    check_ready(status, config, stationary=True)
    assert config["controller"]["swing_up"]["max_cart_excursion_mm"] == 80


@pytest.mark.parametrize("section,key,value", [
    ("motor", "estop", True), ("motor", "left_limit", True),
    ("motor", "homed", False), ("motor", "velocity_mm_s", 1.0),
    ("motor", "position_mm", 400.0), ("motor", "target_velocity_mm_s", float("nan")),
    ("state", "theta_rad", 0.0), ("state", "omega_rad_s", float("nan")),
    ("state", "timestamp_s", float("nan")), ("state", "timestamp_s", 0.0),
    ("system_health", "safe_for_control", False), ("tracking", "status", "low_confidence"),
])
def test_center_preflight_rejects_unsafe_or_unbounded_recovery(ready, section, key, value):
    status, config = ready
    status[section][key] = value
    with pytest.raises(RuntimeError):
        check_ready(status, config, stationary=True)


def test_center_motion_rejects_speed_above_verified_firmware_mode(ready):
    status, config = ready
    status["motor"]["velocity_mm_s"] = 21.0
    with pytest.raises(RuntimeError):
        check_ready(status, config, stationary=False)


def test_two_step_recovery_keeps_each_move_bounded_and_preserves_coordinates(ready):
    status, config = ready
    status["motor"]["position_mm"] = 382.05
    status["state"]["cart_position_mm"] = 381.0
    with pytest.raises(RuntimeError, match="100 mm"):
        check_ready(status, config, stationary=True)
    check_ready(status, config, stationary=True, target_mm=300.0)
    status["motor"]["position_mm"] = 300.0
    status["state"]["cart_position_mm"] = 299.5
    check_ready(status, config, stationary=True)
    assert config["controller"]["cart_center_mm"] == 256.369


@pytest.mark.parametrize("target", [350.0, 200.0, float("nan"), float("inf"), 5.0])
def test_waypoint_cannot_move_away_from_center_or_beyond_safe_path(ready, target):
    status, config = ready
    with pytest.raises(RuntimeError):
        check_ready(status, config, stationary=True, target_mm=target)


def test_explicit_trial_start_crosses_center_only_inside_the_small_start_region(ready):
    status, config = ready
    status["motor"]["position_mm"] = 295.613
    status["state"]["cart_position_mm"] = 296.0
    with pytest.raises(RuntimeError, match="toward the configured center"):
        check_ready(status, config, stationary=True, target_mm=244.35)
    check_ready(status, config, stationary=True, target_mm=244.35, trial_start=True)
    assert config["controller"]["cart_center_mm"] == 256.369


@pytest.mark.parametrize("offset", [-20.01, 20.01, float("nan"), float("inf")])
def test_trial_start_is_bounded_even_during_motion(ready, offset):
    status, config = ready
    status["motor"]["position_mm"] = 260.0
    status["state"]["cart_position_mm"] = 260.0
    for stationary in (True, False):
        with pytest.raises(RuntimeError):
            check_ready(status, config, stationary=stationary,
                        target_mm=256.369 + offset, trial_start=True)


def test_trial_start_retains_move_speed_and_watchdog_limits(ready):
    status, config = ready
    status["motor"]["position_mm"] = 350.0
    with pytest.raises(RuntimeError, match="100 mm"):
        check_ready(status, config, stationary=True, target_mm=244.35, trial_start=True)
    status["motor"]["position_mm"] = 260.0
    status["motor"]["watchdog_ok"] = False
    with pytest.raises(RuntimeError):
        check_ready(status, config, stationary=True, target_mm=244.35, trial_start=True)
    status["motor"]["watchdog_ok"] = True
    status["motor"]["velocity_mm_s"] = 21.0
    with pytest.raises(RuntimeError, match="speed"):
        check_ready(status, config, stationary=False, target_mm=244.35, trial_start=True)
