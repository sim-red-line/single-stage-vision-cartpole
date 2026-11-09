from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from raspberry_pi.config import load_config
from raspberry_pi.motor_interface import MotorStatus
from raspberry_pi.runtime import CartPoleRuntime
from raspberry_pi.safety import SafetyState
from raspberry_pi.web_server import create_app


@pytest.fixture
def runtime(tmp_path):
    (tmp_path / "dashboard").mkdir()
    config = load_config()
    config["controller"].update(output_mode="acceleration", output_sign=-1,
                                max_velocity_mm_s=80.0, max_acceleration_mm_s2=1000.0)
    config["controller"]["swing_up"].update(enabled=True, max_velocity_mm_s=80.0,
                                           max_cart_excursion_mm=80.0)
    instance = CartPoleRuntime(config, root=tmp_path)
    instance.motor = SimpleNamespace(
        status=MotorStatus(connected=True, homed=True, watchdog_ok=True),
        stop=lambda: None, close=lambda: None,
    )
    instance.safety.update_prerequisites(connected=True, homed=True, calibrated=True,
                                         tracking_ok=True, power_ok=True)
    yield instance
    instance.close()


def test_configuring_energy_does_not_arm_or_change_safety_envelope(runtime):
    original_gains = runtime.gains.as_dict()
    result = runtime.configure_swing_up({"energy_gain": 8.0,
                                        "energy_target_margin": 0.02,
                                        "energy_rate_lag_s": 0.03,
                                        "kick_acceleration_mm_s2": 200.0,
                                        "kick_duration_s": 0.3,
                                        "max_duration_s": 10.0})
    assert runtime.safety.state is SafetyState.IDLE
    assert runtime.gains.as_dict() == original_gains
    assert result["mode"] == "idle" and result["energy_gain"] == 8.0
    assert result["energy_target_margin"] == 0.02
    assert result["energy_rate_lag_s"] == 0.03
    assert result["max_velocity_mm_s"] == 80.0
    assert result["max_acceleration_mm_s2"] == 250.0
    assert result["max_cart_excursion_mm"] == 80.0
    result = runtime.configure_swing_up({"max_duration_s": 30.0})
    assert result["max_duration_s"] == 30.0
    assert runtime.safety.state is SafetyState.IDLE


@pytest.mark.parametrize("parameters", [
    {"max_velocity_mm_s": 600.0}, {"max_cart_excursion_mm": 200.0},
    {"kick_acceleration_mm_s2": 251.0}, {"max_duration_s": 31.0},
    {"energy_gain": float("nan")}, {"energy_gain": True}, {},
    {"energy_target_margin": -0.01}, {"energy_target_margin": 0.051},
    {"energy_rate_lag_s": -0.01}, {"energy_rate_lag_s": 0.061},
])
def test_parameters_cannot_raise_limits_or_accept_invalid_values(runtime, parameters):
    with pytest.raises(ValueError):
        runtime.configure_swing_up(parameters)


@pytest.mark.parametrize("safety", [SafetyState.ARMED, SafetyState.RUNNING, SafetyState.ESTOP])
def test_parameters_cannot_change_while_armed_running_or_estopped(runtime, safety):
    runtime.safety.state = safety
    with pytest.raises(RuntimeError):
        runtime.configure_swing_up({"energy_gain": 8.0})


def test_parameters_cannot_change_while_cart_is_moving(runtime):
    runtime.motor.status.velocity_mm_s = 1.0
    with pytest.raises(RuntimeError):
        runtime.configure_swing_up({"energy_gain": 8.0})


def test_api_accepts_only_bounded_energy_fields_without_starting_motion(runtime):
    app = create_app(runtime, manage_runtime=False)
    with TestClient(app) as client:
        response = client.put("/api/control/swing-up/parameters", json={"energy_gain": 8})
        assert response.status_code == 200
        assert response.json()["swing_up"]["energy_gain"] == 8.0
        assert runtime.safety.state is SafetyState.IDLE
        response = client.put("/api/control/swing-up/parameters", json={"energy_target_margin": 0.02})
        assert response.status_code == 200
        assert response.json()["swing_up"]["energy_target_margin"] == 0.02
        response = client.put("/api/control/swing-up/parameters", json={"max_velocity_mm_s": 600})
        assert response.status_code == 422
