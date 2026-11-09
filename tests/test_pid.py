import math

import pytest

from raspberry_pi.models import CartPoleState, PIDGains, TrackingStatus
from raspberry_pi.pid_controller import CascadedPIDController, FilteredPID


def test_filtered_pid_anti_windup_keeps_integral_bounded():
    pid = FilteredPID(10.0, 5.0, 0.0, -1.0, 1.0, anti_windup_gain=0.5)
    for _ in range(200):
        result = pid.update(10.0, 0.0, 0.01)
        assert result.output == pytest.approx(1.0)
    assert abs(pid.integral) < 25.0
    # Reversing the error must permit a prompt move out of positive saturation.
    for _ in range(20):
        result = pid.update(-10.0, 0.0, 0.01)
    assert result.output < 0.0


def test_filtered_pid_uses_estimated_measurement_rate_without_sample_hold_spike():
    pid = FilteredPID(0.0, 0.0, 100.0, -1000.0, 1000.0, derivative_cutoff_hz=0.0)
    pid.update(0.0, 1.0, 0.01, measurement_rate=2.0)
    # The measurement is held on the next 100 Hz tick, but the estimator's
    # physical rate remains 2 units/s.  D must remain -200 rather than drop to 0.
    result = pid.update(0.0, 1.0, 0.01, measurement_rate=2.0)
    assert result.d == pytest.approx(-200.0)


def test_cascade_uses_timestamp_and_acceleration_limit():
    config = {
        "max_angle_setpoint_rad": 0.1,
        "max_velocity_mm_s": 600.0,
        "max_acceleration_mm_s2": 2000.0,
        "derivative_cutoff_hz": 18.0,
        "anti_windup_gain": 0.35,
        "cart_center_mm": 250.0,
        "engage_angle_rad": 0.2,
        "output_sign": 1,
    }
    gains = PIDGains(kp_angle=500.0, kd_angle=20.0, kp_position=0.001)
    controller = CascadedPIDController(config, gains)
    sample = CartPoleState(
        timestamp_s=1.0,
        theta_rad=0.1,
        omega_rad_s=0.0,
        cart_position_mm=250.0,
        cart_velocity_mm_s=0.0,
        valid=True,
        tracking_status=TrackingStatus.OK,
    )
    first = controller.update(sample, 1.0)
    assert not first.active
    second = controller.update(sample, 1.01)
    assert second.active
    assert abs(second.limited_velocity_mm_s) <= 20.0 + 1e-9
    assert abs(second.requested_velocity_mm_s) > abs(second.limited_velocity_mm_s)


def test_invalid_measurement_resets_controller():
    config = {
        "max_angle_setpoint_rad": 0.1,
        "max_velocity_mm_s": 600.0,
        "max_acceleration_mm_s2": 2000.0,
        "cart_center_mm": 250.0,
        "engage_angle_rad": 0.2,
        "output_sign": 1,
    }
    controller = CascadedPIDController(config, PIDGains(kp_angle=100.0))
    invalid = CartPoleState(
        1.0, None, None, None, None, False, TrackingStatus.LOST, reason="marker lost"
    )
    output = controller.update(invalid, 1.0)
    assert output.limited_velocity_mm_s == 0.0
    assert not output.active
    assert output.reason == "marker lost"

