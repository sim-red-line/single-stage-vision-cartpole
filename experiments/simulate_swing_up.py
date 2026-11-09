"""Offline nonlinear swing-up model. This module never opens hardware or an API.

The plant is a point pendulum with an acceleration-limited velocity servo.
It deliberately does not model belt compliance, missed steps, camera noise or
driver dynamics. Optional dry bearing friction supports sensitivity checks.
A simulation pass is not a hardware commissioning result.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import asdict, dataclass, replace

from raspberry_pi.models import CartPoleState, PIDGains, TrackingStatus
from raspberry_pi.pid_controller import HybridSwingUpController


def reference_configuration() -> tuple[dict, PIDGains]:
    """Explicit simulation envelope; does not alter config.local.yaml."""
    return (
        {
            "loop_hz": 100,
            "output_mode": "acceleration",
            "output_sign": -1,
            "position_output_sign": 1,
            "cart_center_mm": 0.0,
            "engage_angle_rad": 0.12,
            "max_angle_setpoint_rad": 0.10,
            "max_velocity_mm_s": 600.0,
            "max_acceleration_mm_s2": 2000.0,
            "derivative_cutoff_hz": 18.0,
            "anti_windup_gain": 0.35,
            "swing_up": {
                "enabled": True,
                "pendulum_length_m": 0.226,
                "energy_gain": 8.0,
                "reference_omega_rad_s": 8.0,
                "max_acceleration_mm_s2": 2000.0,
                "max_velocity_mm_s": 600.0,
                "capture_angle_rad": 0.12,
                "capture_omega_rad_s": 0.8,
                "release_angle_rad": 0.28,
                "max_omega_rad_s": 16.0,
                "kick_acceleration_mm_s2": 400.0,
                "kick_duration_s": 0.20,
                "cart_position_gain_s2": 0.8,
                "cart_velocity_gain_s": 1.5,
                "centering_acceleration_fraction": 0.3,
                "max_cart_excursion_mm": 200.0,
                "max_duration_s": 30.0,
            },
        },
        PIDGains(kp_angle=22000.0, kd_angle=3300.0,
                 kp_position=0.0000077, kd_position=0.0000306),
    )


@dataclass
class SimulationResult:
    captured: bool
    first_capture_s: float | None
    continuous_balance_s: float
    completed: bool
    elapsed_s: float
    final_mode: str
    fault: str | None
    final_theta_rad: float
    max_cart_excursion_mm: float
    max_speed_mm_s: float
    max_acceleration_mm_s2: float
    max_rise_from_down_deg: float
    transitions: list[dict]


def simulate(
    config: dict,
    gains: PIDGains,
    *,
    duration_s: float = 30.0,
    initial_angle_rad: float = math.pi,
    camera_hz: float = 30.0,
    camera_delay_s: float = 0.02,
    damping_s: float = 0.02,
    balance_time_s: float = 3.0,
    plant_length_m: float | None = None,
    coulomb_friction_rad_s2: float = 0.0,
    omega_measurement_delay_s: float = 0.0,
    camera_timestamp_offset_s: float = 0.0,
) -> SimulationResult:
    controller = HybridSwingUpController(config, gains)
    dt = 1.0 / float(config.get("loop_hz", 100))
    center = float(config["cart_center_mm"])
    # A physical rod's equivalent point-mass length can differ from the
    # sticker-to-pivot distance used by the controller. Keep those independent
    # when checking sensitivity to an experimentally measured swing period.
    length_m = controller.length_m if plant_length_m is None else float(plant_length_m)
    if not math.isfinite(length_m) or length_m <= 0.0:
        raise ValueError("plant_length_m must be positive and finite")
    if not math.isfinite(coulomb_friction_rad_s2) or coulomb_friction_rad_s2 < 0.0:
        raise ValueError("coulomb_friction_rad_s2 must be nonnegative and finite")
    if not math.isfinite(omega_measurement_delay_s) or not 0.0 <= omega_measurement_delay_s <= 0.1:
        raise ValueError("omega_measurement_delay_s must be in [0, 0.1]")
    if not math.isfinite(camera_timestamp_offset_s) or not 0.0 <= camera_timestamp_offset_s <= camera_delay_s:
        raise ValueError("camera timestamp offset must be in [0, camera_delay_s]")
    length_mm = length_m * 1000.0
    theta, omega, x, velocity = initial_angle_rad, 0.0, center, 0.0
    maximum_excursion = maximum_speed = maximum_acceleration = 0.0
    maximum_rise = 0.0
    first_capture = None
    stable_time = 0.0
    transitions = []
    previous_mode = "idle"
    camera_period = 1.0 / camera_hz
    next_camera = 0.0
    history: deque[CartPoleState] = deque()
    rate_history: deque[tuple[float, float]] = deque([(0.0, omega)])
    delayed_rate = omega

    def measurement(now: float) -> CartPoleState:
        return CartPoleState(now, theta, delayed_rate, x, velocity, True, TrackingStatus.OK)

    latest = measurement(0.0)
    history.append(latest)
    completed = False
    now = 0.0
    for step in range(round(duration_s / dt) + 1):
        now = step * dt
        maximum_rise = max(maximum_rise, 180.0 - abs(math.degrees(math.atan2(math.sin(theta), math.cos(theta)))))
        if now > rate_history[-1][0]:
            rate_history.append((now, omega))
        rate_time = now - omega_measurement_delay_s
        while len(rate_history) >= 2 and rate_history[1][0] <= rate_time:
            rate_history.popleft()
        delayed_rate = rate_history[0][1]
        if len(rate_history) >= 2 and rate_time >= rate_history[0][0]:
            (ta, a), (tb, b) = rate_history[0], rate_history[1]
            delayed_rate = a + (b - a) * (rate_time - ta) / (tb - ta)
        if now + 1e-9 >= next_camera:
            history.append(measurement(now))
            next_camera += camera_period
        while len(history) >= 2 and history[1].timestamp_s <= now - camera_delay_s + 1e-9:
            history.popleft()
        latest = history[0]
        observed = replace(latest, timestamp_s=latest.timestamp_s + camera_timestamp_offset_s)
        control = controller.update(observed, now)
        if controller.mode != previous_mode:
            transitions.append({"t_s": round(now, 4), "mode": controller.mode})
            previous_mode = controller.mode
        if controller.fault_reason:
            break
        controller.accept_applied_velocity(control.limited_velocity_mm_s)
        if controller.mode == "balance":
            if first_capture is None:
                first_capture = now
            if abs(math.atan2(math.sin(theta), math.cos(theta))) < 0.0872665:
                stable_time += dt
            else:
                stable_time = 0.0
        else:
            stable_time = 0.0
        if stable_time >= balance_time_s:
            completed = True
            break

        target = control.limited_velocity_mm_s
        actuator_limit = float(config["max_acceleration_mm_s2"])
        acceleration = min(actuator_limit, max(-actuator_limit, (target - velocity) / dt))
        maximum_acceleration = max(maximum_acceleration, abs(acceleration))
        x += velocity * dt + 0.5 * acceleration * dt * dt
        velocity += acceleration * dt

        # RK4 integration of the nonlinear pendulum under a held cart
        # acceleration. Unlike the balance-only model, sin/cos are not linearized.
        def derivative(angle: float, rate: float) -> tuple[float, float]:
            return rate, (
                9806.65 * math.sin(angle) - acceleration * math.cos(angle)
            ) / length_mm - damping_s * rate - coulomb_friction_rad_s2 * math.tanh(rate / 0.02)

        a1, w1 = derivative(theta, omega)
        a2, w2 = derivative(theta + 0.5 * dt * a1, omega + 0.5 * dt * w1)
        a3, w3 = derivative(theta + 0.5 * dt * a2, omega + 0.5 * dt * w2)
        a4, w4 = derivative(theta + dt * a3, omega + dt * w3)
        theta += dt * (a1 + 2 * a2 + 2 * a3 + a4) / 6.0
        omega += dt * (w1 + 2 * w2 + 2 * w3 + w4) / 6.0
        maximum_excursion = max(maximum_excursion, abs(x - center))
        maximum_speed = max(maximum_speed, abs(velocity))

    return SimulationResult(
        captured=first_capture is not None,
        first_capture_s=first_capture,
        continuous_balance_s=stable_time,
        completed=completed,
        elapsed_s=now,
        final_mode=controller.mode,
        fault=controller.fault_reason or None,
        final_theta_rad=math.atan2(math.sin(theta), math.cos(theta)),
        max_cart_excursion_mm=maximum_excursion,
        max_speed_mm_s=maximum_speed,
        max_acceleration_mm_s2=maximum_acceleration,
        max_rise_from_down_deg=maximum_rise,
        transitions=transitions,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--camera-delay", type=float, default=0.02)
    parser.add_argument("--plant-length", type=float)
    parser.add_argument("--coulomb-friction", type=float, default=0.0)
    parser.add_argument("--omega-delay", type=float, default=0.0)
    parser.add_argument("--acquisition-timestamps", action="store_true")
    args = parser.parse_args()
    if not 0 < args.duration <= 300 or not 0 <= args.camera_delay <= 0.1:
        parser.error("duration must be in (0, 300] and camera delay in [0, 0.1]")
    config, gains = reference_configuration()
    result = simulate(config, gains, duration_s=args.duration, camera_delay_s=args.camera_delay,
                      plant_length_m=args.plant_length,
                      coulomb_friction_rad_s2=args.coulomb_friction,
                      omega_measurement_delay_s=args.omega_delay,
                      camera_timestamp_offset_s=args.camera_delay if args.acquisition_timestamps else 0.0)
    print(json.dumps({
        "simulation_only": True,
        "model": "point pendulum, velocity servo, 30 Hz delayed camera",
        "plant_length_m": args.plant_length or config["swing_up"]["pendulum_length_m"],
        "coulomb_friction_rad_s2": args.coulomb_friction,
        "omega_measurement_delay_s": args.omega_delay,
        "acquisition_timestamps": args.acquisition_timestamps,
        "config": config,
        "gains": gains.as_dict(),
        "result": asdict(result),
    }, indent=2))


if __name__ == "__main__":
    main()
