import math
import threading
import time

import pytest

from experiments.simulate_swing_up import reference_configuration, simulate
from raspberry_pi.config import load_config
from raspberry_pi.models import CartPoleState, PIDGains, TrackingFrame, TrackingStatus
from raspberry_pi.motor_interface import MotorStatus
from raspberry_pi.pid_controller import CascadedPIDController, HybridSwingUpController
from raspberry_pi.runtime import CartPoleRuntime
from raspberry_pi.safety import SafetyState


GAINS = PIDGains(kp_angle=22000.0, kd_angle=3300.0)


def configuration(**swing_changes):
    config = load_config()["controller"]
    config.update(output_mode="acceleration", output_sign=-1, cart_center_mm=0.0,
                  engage_angle_rad=0.12, max_velocity_mm_s=80.0,
                  max_acceleration_mm_s2=1000.0)
    config["swing_up"].update({"enabled": True, "cart_position_gain_s2": 0.0,
                              "cart_velocity_gain_s": 0.0, **swing_changes})
    return config


def state(now=1.0, theta=math.pi, omega=0.0, x=0.0, velocity=0.0):
    return CartPoleState(now, theta, omega, x, velocity, True, TrackingStatus.OK)


def test_disabled_controller_preserves_balance_only_behavior():
    config = configuration()
    config["swing_up"]["enabled"] = False
    direct = CascadedPIDController(config, GAINS)
    hybrid = HybridSwingUpController(config, GAINS)
    for now in (1.0, 1.01, 1.02, 1.03):
        sample = state(now, theta=0.01)
        assert hybrid.update(sample, now) == direct.update(sample, now)
    assert hybrid.snapshot()["mode"] == "balance_only"


def test_hanging_start_kick_accumulates_with_sample_held_velocity():
    controller = HybridSwingUpController(configuration(kick_duration_s=0.20), GAINS)
    sample = state()
    assert controller.update(sample, 1.0).limited_velocity_mm_s == 0.0
    # The camera still reports zero cart velocity between frames. Ten 100 Hz
    # ticks must accumulate acceleration, rather than repeatedly request 0.6.
    for index in range(1, 11):
        output = controller.update(sample, 1.0 + index * 0.01)
    assert controller.mode == "swing_up"
    assert output.limited_velocity_mm_s == pytest.approx(6.0)
    assert "kick" in output.reason


@pytest.mark.parametrize("theta", [0.4, 1.2, 2.7, -2.7])
@pytest.mark.parametrize("omega", [-5.0, -1.0, 1.0, 5.0])
@pytest.mark.parametrize("margin", [0.0, 0.02])
def test_energy_command_decreases_squared_energy_error(theta, omega, margin):
    controller = HybridSwingUpController(
        configuration(kick_duration_s=0.0, energy_target_margin=margin), GAINS
    )
    controller.update(state(theta=theta, omega=omega), 1.0)
    output = controller.update(state(1.01, theta=theta, omega=omega), 1.01)
    acceleration_m_s2 = output.limited_velocity_mm_s / 0.01 / 1000.0
    energy_rate = -acceleration_m_s2 * omega * math.cos(theta) / 9.80665
    assert controller.energy_error * energy_rate < 0.0


def test_kick_only_occurs_near_hanging_rest_not_during_upright_flyby():
    controller = HybridSwingUpController(configuration(capture_omega_rad_s=0.5), GAINS)
    output = controller.update(state(theta=0.01, omega=0.8), 1.0)
    assert controller.mode == "swing_up"
    assert "kick" not in output.reason


def test_upright_centering_window_preserves_energy_authority_near_top():
    options = dict(energy_gain=64.0, energy_target_margin=0.05,
                   cart_position_gain_s2=3.2, cart_velocity_gain_s=3.0,
                   kick_duration_s=0.0)
    old = HybridSwingUpController(configuration(**options), GAINS)
    phased = HybridSwingUpController(configuration(
        **options, upright_centering_window_rad=0.7
    ), GAINS)
    for controller in (old, phased):
        controller.update(state(theta=0.2, omega=-1.0, velocity=200.0), 1.0)
    raw = old.update(state(1.01, theta=0.2, omega=-1.0, velocity=200.0), 1.01)
    changed = phased.update(state(1.01, theta=0.2, omega=-1.0, velocity=200.0), 1.01)
    assert changed.limited_velocity_mm_s > raw.limited_velocity_mm_s
    assert changed.limited_velocity_mm_s > 0.0
    assert abs(changed.limited_velocity_mm_s) <= phased.max_acceleration * 0.01 + 1e-9
    assert phased.mode == "swing_up"


def test_upright_centering_window_retains_far_angle_command_and_excursion_guard():
    config = configuration(cart_position_gain_s2=3.2, cart_velocity_gain_s=3.0)
    raw = HybridSwingUpController(config, GAINS)
    phased = HybridSwingUpController(configuration(
        cart_position_gain_s2=3.2, cart_velocity_gain_s=3.0,
        upright_centering_window_rad=0.7
    ), GAINS)
    for index in range(10):
        now = 1.0 + index * 0.01
        sample = state(now, theta=1.0, omega=-1.0, x=15.0, velocity=20.0)
        assert phased.update(sample, now) == raw.update(sample, now)
    output = phased.update(state(1.10, theta=0.2, x=1000.0), 1.10)
    assert not output.active and "excursion" in phased.fault_reason


def test_energy_alignment_removes_rate_filter_phase_error_across_angle_wrap():
    lag, length, amplitude = 0.03, 0.187, 0.30
    natural_rate = math.sqrt(9.80665 / length)
    raw = HybridSwingUpController(configuration(pendulum_length_m=length), GAINS)
    aligned = HybridSwingUpController(
        configuration(pendulum_length_m=length, energy_rate_lag_s=lag), GAINS
    )
    errors = [[], []]
    for frame in range(150):
        t = 1.0 + frame / 30.0
        theta = math.atan2(math.sin(math.pi + amplitude * math.cos(natural_rate * t)),
                           math.cos(math.pi + amplitude * math.cos(natural_rate * t)))
        delayed_rate = -amplitude * natural_rate * math.sin(natural_rate * (t - lag))
        sample = state(t, theta=theta, omega=delayed_rate)
        for controller, values in zip((raw, aligned), errors):
            controller.update(sample, t)
            if frame > 5:
                values.append(controller.energy_error)
    span = lambda values: max(values) - min(values)
    assert span(errors[1]) < span(errors[0]) * 0.15


def test_short_prediction_matches_small_angle_free_dynamics():
    length, horizon = 0.187, 0.04
    controller = HybridSwingUpController(
        configuration(pendulum_length_m=length, prediction_camera_latency_s=horizon), GAINS
    )
    controller.update(state(theta=0.01, omega=0.0), 1.0)
    rate = math.sqrt(9.80665 / length)
    assert controller.predicted_theta == pytest.approx(0.01 * math.cosh(rate * horizon), rel=0.01)
    assert controller.predicted_omega == pytest.approx(0.01 * rate * math.sinh(rate * horizon), rel=0.01)


def test_prediction_includes_previously_admitted_cart_acceleration():
    controller = HybridSwingUpController(
        configuration(pendulum_length_m=0.187, prediction_camera_latency_s=0.04), GAINS
    )
    sample = state()
    first = controller.update(sample, 1.0)
    controller.accept_applied_velocity(first.limited_velocity_mm_s)
    second = controller.update(sample, 1.01)
    controller.accept_applied_velocity(second.limited_velocity_mm_s)
    controller.update(sample, 1.02)
    assert controller.predicted_omega > 0.0
    delta = math.atan2(math.sin(controller.predicted_theta - math.pi),
                       math.cos(controller.predicted_theta - math.pi))
    assert delta > 0.0


def test_prediction_horizon_is_bounded_even_before_camera_stale_limit():
    controller = HybridSwingUpController(
        configuration(prediction_camera_latency_s=0.04, energy_rate_lag_s=0.03), GAINS
    )
    output = controller.update(state(), 1.11)
    assert not output.active and output.limited_velocity_mm_s == 0.0
    assert "prediction horizon" in controller.fault_reason


def test_balance_prediction_leaves_energy_shaping_and_capture_eligibility_unchanged():
    raw = HybridSwingUpController(configuration(energy_rate_lag_s=0.03), GAINS)
    forecast = HybridSwingUpController(configuration(
        energy_rate_lag_s=0.03, balance_prediction_camera_latency_s=0.026
    ), GAINS)
    for index in range(30):
        now = 1.0 + index * 0.01
        sample = state(now, theta=0.20, omega=-1.0)
        original = raw.update(sample, now)
        predicted = forecast.update(sample, now)
        assert predicted == original
        assert forecast.mode == "swing_up"  # Raw angle is outside capture.
        raw.accept_applied_velocity(original.limited_velocity_mm_s)
        forecast.accept_applied_velocity(predicted.limited_velocity_mm_s)
    assert forecast.predicted_theta is None


def test_separate_balance_rate_lag_keeps_unaligned_energy_drive_identical():
    raw = HybridSwingUpController(configuration(energy_rate_lag_s=0.0), GAINS)
    forecast = HybridSwingUpController(configuration(
        energy_rate_lag_s=0.0, balance_rate_lag_s=0.03,
        balance_prediction_camera_latency_s=0.02,
    ), GAINS)
    for index in range(90):
        now = 1.0 + index / 100.0
        sample = state(now, theta=1.0 + 0.2 * math.sin(index / 7),
                       omega=0.5 * math.cos(index / 7))
        original = raw.update(sample, now)
        predicted = forecast.update(sample, now)
        assert predicted == original
        assert forecast.energy_error == raw.energy_error
        assert forecast.mode == raw.mode == "swing_up"
        raw.accept_applied_velocity(original.limited_velocity_mm_s)
        forecast.accept_applied_velocity(predicted.limited_velocity_mm_s)


def test_balance_rate_alignment_matches_legacy_forecast_without_sharing_energy_lag():
    legacy = HybridSwingUpController(configuration(
        energy_rate_lag_s=0.03, balance_prediction_camera_latency_s=0.02,
    ), GAINS)
    separate = HybridSwingUpController(configuration(
        energy_rate_lag_s=0.0, balance_rate_lag_s=0.03,
        balance_prediction_camera_latency_s=0.02,
    ), GAINS)
    for index in range(8):
        now = 1.0 + index / 30.0
        for controller in (legacy, separate):
            controller._energy_angle(now, 0.1 + 0.4 * (now - 1.0))
    sample = state(now, theta=0.1 + 0.4 * (now - 1.0), omega=0.4,
                   x=10.0, velocity=20.0)
    old = legacy._balance_state(sample, now + 0.025)
    new = separate._balance_state(sample, now + 0.025)
    assert new == old
    assert separate.energy_rate_lag == 0.0
    assert new.timestamp_s == sample.timestamp_s
    assert new.cart_position_mm == sample.cart_position_mm


def test_missing_balance_rate_alignment_inherits_energy_setting():
    controller = HybridSwingUpController(configuration(energy_rate_lag_s=0.03), GAINS)
    assert controller.balance_rate_lag == 0.03


def test_balance_forecast_preserves_measured_rail_position_velocity_and_age():
    controller = HybridSwingUpController(configuration(
        pendulum_length_m=0.187, balance_prediction_camera_latency_s=0.04
    ), GAINS)
    sample = state(theta=0.01, x=15.0, velocity=2.0)
    controller.update(sample, 1.0)
    forecast = controller._balance_state(sample, 1.0)
    rate = math.sqrt(9.80665 / 0.187)
    assert forecast.theta_rad == pytest.approx(0.01 * math.cosh(rate * 0.04), rel=0.01)
    assert forecast.omega_rad_s == pytest.approx(0.01 * rate * math.sinh(rate * 0.04), rel=0.01)
    assert forecast.timestamp_s == sample.timestamp_s
    assert forecast.cart_position_mm == sample.cart_position_mm
    assert forecast.cart_velocity_mm_s == sample.cart_velocity_mm_s
    assert sample.theta_rad == 0.01


def test_balance_actuator_forecast_keeps_braking_after_admitted_stop():
    controller = HybridSwingUpController(configuration(
        pendulum_length_m=0.187, balance_prediction_camera_latency_s=0.02
    ), GAINS)
    controller.update(state(theta=0.0), 1.0)
    controller.accept_applied_velocity(80.0)
    controller.update(state(1.10, theta=0.0), 1.10)
    assert controller._actuator_velocity == pytest.approx(80.0)
    controller.accept_applied_velocity(0.0)
    controller.update(state(1.11, theta=0.0), 1.11)
    assert controller._actuator_velocity == pytest.approx(70.0)
    controller.accept_applied_velocity(0.0)
    controller.update(state(1.12, theta=0.0), 1.12)
    assert controller._actuator_velocity == pytest.approx(60.0)
    # A 20 ms, -1000 mm/s² braking impulse creates +0.107 rad/s angular rate.
    # Differentiating targets would incorrectly make the second 10 ms coast.
    assert controller.predicted_omega == pytest.approx(1000.0 * 0.02 / 187.0, rel=0.02)


def test_balance_forecast_horizon_faults_without_resetting_measurement_timestamp():
    controller = HybridSwingUpController(configuration(
        balance_prediction_camera_latency_s=0.04, energy_rate_lag_s=0.03
    ), GAINS)
    output = controller.update(state(theta=0.01), 1.11)
    assert controller.mode == "fault" and not output.active
    assert output.limited_velocity_mm_s == 0.0
    assert "balance prediction horizon" in controller.fault_reason


def test_balance_forecast_does_not_override_raw_excursion_or_release():
    controller = HybridSwingUpController(configuration(
        balance_prediction_camera_latency_s=0.026
    ), GAINS)
    controller.update(state(theta=0.01), 1.0)
    assert controller.mode == "balance"
    controller.update(state(1.01, theta=0.30), 1.01)
    assert controller.mode == "swing_up"
    output = controller.update(state(1.02, theta=0.0, x=1000.0), 1.02)
    assert not output.active and "excursion" in controller.fault_reason


def test_full_and_balance_only_prediction_cannot_double_compensate_delay():
    with pytest.raises(ValueError, match="either full swing-up or balance-only"):
        HybridSwingUpController(configuration(
            prediction_camera_latency_s=0.02, balance_prediction_camera_latency_s=0.02
        ), GAINS)


def test_swing_speed_and_acceleration_never_exceed_host_limits():
    config = configuration(kick_acceleration_mm_s2=10000, kick_duration_s=10.0,
                           max_acceleration_mm_s2=20000, max_velocity_mm_s=10000)
    config.update(max_velocity_mm_s=20.0, max_acceleration_mm_s2=40.0)
    controller = HybridSwingUpController(config, GAINS)
    previous = 0.0
    for index in range(101):
        now = 1.0 + 0.01 * index
        output = controller.update(state(now), now)
        velocity = output.limited_velocity_mm_s
        assert abs(velocity) <= 20.0
        assert abs(velocity - previous) <= 40.0 * 0.01 + 1e-9
        previous = velocity


def test_capture_checks_both_angle_and_rate_and_preserves_velocity():
    controller = HybridSwingUpController(configuration(capture_omega_rad_s=0.5), GAINS)
    controller.update(state(), 1.0)
    before = controller.update(state(1.10), 1.10)
    assert before.limited_velocity_mm_s > 0
    capture = controller.update(state(1.11, theta=0.01, omega=0.1,
                                      velocity=before.limited_velocity_mm_s), 1.11)
    assert controller.mode == "balance"
    assert capture.active
    assert capture.limited_velocity_mm_s == before.limited_velocity_mm_s
    next_output = controller.update(state(1.12, theta=0.01, omega=0.1), 1.12)
    assert abs(next_output.limited_velocity_mm_s - capture.limited_velocity_mm_s) <= 10.0 + 1e-9


def test_capture_window_cannot_exceed_balance_engage_window():
    controller = HybridSwingUpController(configuration(capture_angle_rad=0.16), GAINS)
    assert controller.capture_angle == pytest.approx(0.12)
    controller.update(state(theta=0.14), 1.0)
    assert controller.mode == "swing_up"


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("theta,omega,expected", [
    (0.21, -1.3, "balance"),  # New angle band, rising with correction authority.
    (0.23, -1.3, "swing_up"),  # Still outside the widened angle band.
    (0.21, -1.51, "swing_up"),  # Rate gate remains 1.5 rad/s.
    (0.21, 1.3, "swing_up"),  # Divergence exceeds acceleration headroom.
])
def test_wider_capture_preserves_rate_and_authority_gates(sign, theta, omega, expected):
    config = configuration(
        pendulum_length_m=0.187, max_velocity_mm_s=240.0,
        max_acceleration_mm_s2=1500.0, capture_angle_rad=0.22,
        capture_omega_rad_s=1.5, capture_velocity_headroom_fraction=0.2,
    )
    config.update(max_velocity_mm_s=240.0, max_acceleration_mm_s2=2000.0,
                  engage_angle_rad=0.22, cart_center_mm=256.369)
    controller = HybridSwingUpController(config, GAINS)
    output = controller.update(state(theta=sign * theta, omega=sign * omega,
                                     x=256.369, velocity=0), 1.0)
    assert controller.mode == expected
    assert output.active


def test_capture_requires_velocity_authority_in_the_needed_direction():
    config = configuration(pendulum_length_m=0.187,
                           capture_velocity_headroom_fraction=0.2)
    blocked = HybridSwingUpController(config, GAINS)
    blocked.accept_applied_velocity(80.0)
    blocked.update(state(theta=0.04, omega=0.2, velocity=80.0), 1.0)
    assert blocked.mode == "swing_up"
    feasible = HybridSwingUpController(config, GAINS)
    feasible.accept_applied_velocity(-60.0)
    output = feasible.update(state(theta=0.04, omega=0.2, velocity=-60.0), 1.0)
    assert feasible.mode == "balance"
    assert output.limited_velocity_mm_s == -60.0


def test_capture_rejects_divergence_beyond_acceleration_authority():
    config = configuration(pendulum_length_m=0.187,
                           max_velocity_mm_s=500,
                           capture_velocity_headroom_fraction=0.2)
    config["max_velocity_mm_s"] = 500
    config["swing_up"]["max_velocity_mm_s"] = 500
    controller = HybridSwingUpController(config, GAINS)
    controller.update(state(theta=0.06, omega=1.0), 1.0)
    assert controller.mode == "swing_up"


def test_recorded_slow_upright_pass_fits_existing_balance_engage_window():
    config = configuration(pendulum_length_m=0.187, max_velocity_mm_s=160.0,
                           max_acceleration_mm_s2=1500.0, max_cart_excursion_mm=120.0,
                           capture_angle_rad=0.12, capture_omega_rad_s=0.8,
                           capture_velocity_headroom_fraction=0.2)
    config.update(max_velocity_mm_s=160.0, max_acceleration_mm_s2=2000.0,
                  cart_center_mm=256.369)
    controller = HybridSwingUpController(config, GAINS)
    controller.accept_applied_velocity(-59.39)
    output = controller.update(state(theta=-0.1129, omega=0.534, x=293.738,
                                     velocity=-65.0), 1.0)
    assert controller.mode == "balance"
    assert output.active and output.limited_velocity_mm_s == -59.39


def test_recorded_fast_pass_has_capture_authority_at_240_mm_s():
    config = configuration(
        pendulum_length_m=0.187, max_velocity_mm_s=240.0,
        max_acceleration_mm_s2=1500.0, capture_angle_rad=0.14,
        capture_omega_rad_s=2.0, capture_velocity_headroom_fraction=0.2,
    )
    config.update(max_velocity_mm_s=240.0, max_acceleration_mm_s2=2000.0,
                  engage_angle_rad=0.22, cart_center_mm=256.369)
    controller = HybridSwingUpController(config, GAINS)
    controller.accept_applied_velocity(-217.9)
    output = controller.update(state(theta=math.radians(-7.13), omega=1.741,
                                     x=259.9, velocity=-209.0), 1.0)
    assert controller.mode == "balance" and output.active
    assert output.limited_velocity_mm_s == -217.9
    # The same angular state with cart velocity in the other direction lacks
    # authority; the broader rate window must not bypass coupled headroom.
    blocked = HybridSwingUpController(config, GAINS)
    blocked.accept_applied_velocity(240.0)
    blocked.update(state(theta=math.radians(-7.13), omega=1.741,
                         x=259.9, velocity=240.0), 1.0)
    assert blocked.mode == "swing_up"


def test_capture_wraps_full_rotations_to_upright():
    controller = HybridSwingUpController(configuration(), GAINS)
    controller.update(state(theta=2 * math.pi + 0.01), 1.0)
    assert controller.mode == "balance"
    output = controller.update(state(1.01, theta=2 * math.pi + 0.01), 1.01)
    assert output.angle_p == pytest.approx(-220.0)


def test_recovery_preserves_velocity_and_bounds_new_swing_acceleration():
    controller = HybridSwingUpController(configuration(), GAINS)
    controller.update(state(theta=0.01), 1.0)
    controller.accept_applied_velocity(20.0)
    output = controller.update(state(1.01, theta=0.30, omega=0.4, velocity=20.0), 1.01)
    assert controller.mode == "swing_up"
    assert abs(output.limited_velocity_mm_s - 20.0) <= 2.5 + 1e-9


def test_balance_speed_reserve_and_recovery_never_jump_velocity():
    config = configuration(
        max_velocity_mm_s=160.0, balance_max_velocity_mm_s=240.0,
        max_acceleration_mm_s2=1500.0, pendulum_length_m=0.187,
        capture_velocity_headroom_fraction=0.2,
    )
    config.update(max_velocity_mm_s=240.0, max_acceleration_mm_s2=2000.0)
    controller = HybridSwingUpController(config, GAINS)
    controller.accept_applied_velocity(160.0)
    output = controller.update(state(theta=-0.04, omega=0.2, velocity=160.0), 1.0)
    assert controller.mode == "balance" and output.limited_velocity_mm_s == 160.0
    output = controller.update(state(1.01, theta=0.05, omega=0.5), 1.01)
    assert 160.0 < output.limited_velocity_mm_s <= 180.0 + 1e-9
    before = output.limited_velocity_mm_s
    controller.accept_applied_velocity(before)
    recovered = controller.update(state(1.02, theta=0.30, omega=0.4), 1.02)
    assert controller.mode == "swing_up"
    assert recovered.limited_velocity_mm_s > 160.0
    assert abs(recovered.limited_velocity_mm_s - before) <= 15.0 + 1e-9
    controller.accept_applied_velocity(recovered.limited_velocity_mm_s)
    following = controller.update(state(1.03, theta=0.30, omega=0.4), 1.03)
    assert abs(following.limited_velocity_mm_s - recovered.limited_velocity_mm_s) <= 15.0 + 1e-9
    assert abs(following.limited_velocity_mm_s) <= 240.0


def test_balance_reserve_cannot_exceed_host_or_drop_below_swing_limit():
    config = configuration(max_velocity_mm_s=160.0, balance_max_velocity_mm_s=500.0)
    config["max_velocity_mm_s"] = 240.0
    controller = HybridSwingUpController(config, GAINS)
    assert controller.max_velocity == 160.0 and controller.balance_velocity == 240.0
    config["swing_up"]["balance_max_velocity_mm_s"] = 100.0
    with pytest.raises(ValueError, match="at least the swing-up"):
        HybridSwingUpController(config, GAINS)


def test_outer_safety_clamp_is_not_integrated_as_hidden_velocity():
    controller = HybridSwingUpController(configuration(), GAINS)
    controller.update(state(), 1.0)
    assert controller.update(state(1.01), 1.01).limited_velocity_mm_s == pytest.approx(0.6)
    controller.accept_applied_velocity(0.0)
    assert controller.update(state(1.02), 1.02).limited_velocity_mm_s == pytest.approx(0.6)


@pytest.mark.parametrize("field", ["theta_rad", "omega_rad_s", "cart_position_mm",
                                   "cart_velocity_mm_s", "timestamp_s"])
def test_nonfinite_measurements_latch_zero_output(field):
    controller = HybridSwingUpController(configuration(), GAINS)
    sample = state()
    setattr(sample, field, float("nan"))
    output = controller.update(sample, 1.0)
    assert controller.mode == "fault"
    assert not output.active and output.limited_velocity_mm_s == 0.0
    assert not controller.update(state(1.01), 1.01).active
    controller.reset()
    assert controller.update(state(1.02), 1.02).active


@pytest.mark.parametrize("case", ["stale", "tracking", "angular_speed", "excursion", "bad_dt"])
def test_fault_conditions_stop_without_another_motion_command(case):
    controller = HybridSwingUpController(configuration(), GAINS)
    controller.update(state(), 1.0)
    sample = state(1.01)
    now = 1.01
    if case == "stale":
        sample.timestamp_s = 0.5
    elif case == "tracking":
        sample.valid = False
    elif case == "angular_speed":
        sample.omega_rad_s = 17.0
    elif case == "excursion":
        sample.cart_position_mm = controller.max_excursion
    else:
        now = 1.0
    output = controller.update(sample, now)
    assert not output.active
    assert controller.fault_reason
    assert output.limited_velocity_mm_s == 0.0


def test_timeout_counts_swing_time_across_recovery_attempts():
    controller = HybridSwingUpController(configuration(max_duration_s=0.04), GAINS)
    controller.update(state(), 1.0)
    controller.update(state(1.02), 1.02)
    controller.update(state(1.03, theta=0.0), 1.03)
    assert controller.mode == "balance"
    controller.update(state(1.04, theta=0.4), 1.04)
    output = controller.update(state(1.06, theta=0.4), 1.06)
    assert controller.mode == "fault"
    assert "time limit" in output.reason


@pytest.mark.parametrize("key,value", [
    ("pendulum_length_m", float("nan")), ("energy_gain", -1.0),
    ("max_duration_s", 0.0), ("release_angle_rad", 0.01),
    ("kick_direction", 0.0), ("centering_acceleration_fraction", 1.1),
    ("enabled", "false"),
    ("upright_centering_window_rad", -0.01),
    ("upright_centering_window_rad", 1.001),
    ("energy_target_margin", -0.01), ("energy_target_margin", 0.051),
    ("energy_rate_lag_s", -0.01), ("energy_rate_lag_s", 0.061),
    ("balance_rate_lag_s", -0.01), ("balance_rate_lag_s", 0.061),
    ("balance_rate_lag_s", float("nan")),
    ("capture_velocity_headroom_fraction", -0.1),
    ("capture_velocity_headroom_fraction", 1.0),
    ("prediction_camera_latency_s", 0.061), ("prediction_damping_s", 0.51),
    ("balance_prediction_camera_latency_s", 0.061),
    ("balance_prediction_camera_latency_s", float("nan")),
    ("prediction_coulomb_rad_s2", 2.01),
])
def test_invalid_swing_config_is_rejected(key, value):
    with pytest.raises(ValueError):
        HybridSwingUpController(configuration(**{key: value}), GAINS)


def test_nonlinear_model_swings_from_exact_downward_rest_and_balances():
    config, gains = reference_configuration()
    result = simulate(config, gains)
    assert result.completed and result.captured
    assert result.fault is None
    assert result.continuous_balance_s >= 3.0
    assert result.max_speed_mm_s <= config["max_velocity_mm_s"]
    assert result.max_cart_excursion_mm < config["swing_up"]["max_cart_excursion_mm"]
    assert result.max_acceleration_mm_s2 <= config["max_acceleration_mm_s2"] + 1e-9


def test_restrictive_model_reports_timeout_instead_of_claiming_success():
    config, gains = reference_configuration()
    config["swing_up"].update(max_duration_s=0.10, kick_acceleration_mm_s2=0.0)
    result = simulate(config, gains, duration_s=1.0)
    assert not result.completed and not result.captured
    assert result.fault == "swing-up time limit"


def test_measured_delay_model_captures_and_balances_with_bounded_prediction():
    config, gains = reference_configuration()
    config.update(max_velocity_mm_s=80.0, max_acceleration_mm_s2=1000.0)
    config["swing_up"].update(
        pendulum_length_m=0.187, max_velocity_mm_s=80.0,
        max_acceleration_mm_s2=750.0, max_cart_excursion_mm=80.0,
        kick_acceleration_mm_s2=200.0, kick_duration_s=0.3,
        capture_angle_rad=0.06, capture_omega_rad_s=0.8,
        capture_velocity_headroom_fraction=0.2,
        energy_gain=32.0, energy_target_margin=0.02, energy_rate_lag_s=0.03,
        prediction_camera_latency_s=0.04, prediction_damping_s=0.06,
        prediction_coulomb_rad_s2=0.36,
    )
    gains = PIDGains.from_mapping({**gains.as_dict(), "kd_position": 0.00025})
    plant = dict(duration_s=35.5, balance_time_s=5.0, plant_length_m=0.187,
                 camera_delay_s=0.04, camera_timestamp_offset_s=0.04,
                 omega_measurement_delay_s=0.03, damping_s=0.06,
                 coulomb_friction_rad_s2=0.36)
    result = simulate(config, gains, **plant)
    assert result.completed and result.continuous_balance_s >= 5.0
    assert result.fault is None
    assert result.max_cart_excursion_mm < 80.0
    assert result.max_speed_mm_s <= 80.0
    assert result.max_acceleration_mm_s2 <= 1000.0 + 1e-9
    config["swing_up"]["prediction_camera_latency_s"] = 0.0
    uncompensated = simulate(config, gains, **plant)
    assert not uncompensated.completed


def test_runtime_requires_acceleration_capture_mode(tmp_path):
    config = load_config()
    config["controller"]["swing_up"]["enabled"] = True
    runtime = CartPoleRuntime(config, root=tmp_path)
    try:
        with pytest.raises(RuntimeError, match="acceleration output mode"):
            runtime.begin_control()
    finally:
        runtime.close()


@pytest.mark.parametrize("kp,kd", [(0.0, 3300.0), (22000.0, 0.0),
                                   (float("nan"), 3300.0), (22000.0, float("nan"))])
def test_runtime_rejects_missing_or_nonfinite_capture_gains(tmp_path, kp, kd):
    config = load_config()
    config["controller"].update(output_mode="acceleration", output_sign=-1)
    config["controller"]["gains"].update(kp_angle=kp, kd_angle=kd)
    config["controller"]["swing_up"]["enabled"] = True
    runtime = CartPoleRuntime(config, root=tmp_path)
    try:
        with pytest.raises(RuntimeError):
            runtime.begin_control()
        assert runtime.safety.state is not SafetyState.RUNNING
    finally:
        runtime.close()


def test_runtime_latches_swing_timeout_and_stops_motor(tmp_path):
    config = load_config()
    config["controller"].update(output_mode="acceleration", output_sign=-1)
    config["controller"]["gains"].update(GAINS.as_dict())
    config["controller"]["swing_up"].update(enabled=True, max_duration_s=0.03)
    runtime = CartPoleRuntime(config, root=tmp_path)

    class FakeMotor:
        def __init__(self):
            self._status = MotorStatus(connected=True, homed=True, watchdog_ok=True)
            self.stopped = False

        @property
        def status(self):
            # Supply a live stationary camera sample so this integration test
            # exercises the swing timer, rather than a stale-camera stop.
            runtime._latest_state = state(time.monotonic(), x=250.0)
            return self._status

        def set_velocity(self, velocity):
            self._status.target_velocity_mm_s = velocity

        def stop(self):
            self.stopped = True
            self._status.target_velocity_mm_s = 0.0
            runtime._stop_event.set()

        def close(self):
            pass

    motor = FakeMotor()
    runtime.motor = motor
    now = time.monotonic()
    runtime._latest_state = state(now, x=250.0)
    runtime._latest_tracking = TrackingFrame(now, {}, TrackingStatus.OK, 128.0, 0.0, 1)
    runtime._calibration_verified = True
    runtime.safety.update_prerequisites(connected=True, homed=True, calibrated=True,
                                        tracking_ok=True, power_ok=True)
    assert runtime.arm()
    assert runtime.begin_control()
    thread = threading.Thread(target=runtime._control_loop, daemon=True)
    try:
        thread.start()
        thread.join(timeout=1.0)
        assert not thread.is_alive(), (
            runtime.safety.state, runtime.safety.reason, runtime._control_error,
            runtime.controller.snapshot(), runtime._last_motor_command,
        )
        assert runtime.safety.state is SafetyState.FAULT
        assert "swing-up time limit" in runtime.safety.reason
        assert motor.stopped and motor.status.target_velocity_mm_s == 0.0
        assert runtime.snapshot()["swing_up"]["mode"] == "fault"
    finally:
        runtime._stop_event.set()
        thread.join(timeout=1.0)
        runtime.close()
