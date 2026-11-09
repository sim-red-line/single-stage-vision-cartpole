import math

import pytest

from experiments.run_upright_direction_trial import preflight


def ready_status(speed=160.0, excursion=120.0):
    return {
        "safety_state": "IDLE",
        "state": {"valid": True, "theta_rad": math.pi, "omega_rad_s": 0.0},
        "motor": {"connected": True, "homed": True, "watchdog_ok": True,
                  "estop": False, "fault": "", "left_limit": False, "right_limit": False,
                  "velocity_mm_s": 0.0, "target_velocity_mm_s": 0.0},
        "swing_up": {"enabled": True, "max_duration_s": 30.0,
                     "max_velocity_mm_s": speed, "max_cart_excursion_mm": excursion,
                     "max_acceleration_mm_s2": 1500.0},
        "calibration": {"live_verified": True},
        "system_health": {"safe_for_control": True},
        "gains": {"kp_angle": 26000.0, "kd_angle": 3300.0},
        "controller_config": {"output_mode": "acceleration"},
    }


def test_default_profile_accepts_existing_commissioned_envelope():
    preflight(ready_status(), energy_swing_up=True)


def test_reserved_balance_speed_still_requires_the_expanded_profile():
    status = ready_status(160.0, 120.0)
    status["swing_up"]["balance_max_velocity_mm_s"] = 240.0
    with pytest.raises(RuntimeError, match="balance velocity"):
        preflight(status, energy_swing_up=True)
    preflight(status, energy_swing_up=True, commissioning_profile="capture240")


@pytest.mark.parametrize("speed", [0.0, float("nan"), 240.01])
def test_reserved_balance_speed_cannot_escape_the_explicit_profile(speed):
    status = ready_status(160.0, 160.0)
    status["swing_up"]["balance_max_velocity_mm_s"] = speed
    with pytest.raises(RuntimeError, match="balance velocity"):
        preflight(status, energy_swing_up=True, commissioning_profile="capture240")


def test_expanded_settings_require_an_explicit_profile():
    with pytest.raises(RuntimeError, match="commissioned trial envelope"):
        preflight(ready_status(240.0, 160.0), energy_swing_up=True)
    preflight(ready_status(240.0, 160.0), energy_swing_up=True,
              commissioning_profile="capture240")


@pytest.mark.parametrize("key,value", [
    ("max_velocity_mm_s", 240.01), ("max_cart_excursion_mm", 160.01),
    ("max_acceleration_mm_s2", 1500.01), ("max_velocity_mm_s", float("nan")),
])
def test_explicit_profile_still_rejects_settings_beyond_its_limits(key, value):
    status = ready_status(240.0, 160.0)
    status["swing_up"][key] = value
    with pytest.raises(RuntimeError):
        preflight(status, energy_swing_up=True, commissioning_profile="capture240")


@pytest.mark.parametrize("section,key,value", [
    ("motor", "watchdog_ok", False), ("motor", "homed", False),
    ("motor", "estop", True), ("motor", "left_limit", True),
    ("motor", "velocity_mm_s", float("nan")),
    ("motor", "target_velocity_mm_s", float("inf")),
    ("gains", "kd_angle", float("nan")),
    ("state", "valid", False), ("state", "omega_rad_s", 0.3),
    ("system_health", "safe_for_control", False),
])
def test_profile_does_not_relax_hardware_or_tracking_interlocks(section, key, value):
    status = ready_status(240.0, 160.0)
    status[section][key] = value
    with pytest.raises(RuntimeError):
        preflight(status, energy_swing_up=True, commissioning_profile="capture240")
