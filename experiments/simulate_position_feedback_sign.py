from __future__ import annotations

import json

from raspberry_pi.models import CartPoleState, PIDGains, TrackingStatus
from raspberry_pi.pid_controller import CascadedPIDController


DT = 0.01
LENGTH_MM = 226.0
GRAVITY_MM_S2 = 9806.65


def simulate(position_output_sign: int) -> dict:
    config = {
        "output_mode": "acceleration",
        "output_sign": -1,
        "position_output_sign": position_output_sign,
        "cart_center_mm": 0.0,
        "engage_angle_rad": 0.12,
        "max_angle_setpoint_rad": 0.10,
        "max_velocity_mm_s": 80.0,
        "max_acceleration_mm_s2": 1000.0,
        "derivative_cutoff_hz": 18.0,
        "anti_windup_gain": 0.35,
    }
    controller = CascadedPIDController(
        config,
        PIDGains(
            kp_angle=22000.0,
            kd_angle=3300.0,
            kp_position=0.0000077,
            kd_position=0.0000306,
        ),
    )
    theta = 0.00365
    omega = -0.01625
    position = 15.0
    velocity = 0.0
    measured = (theta, omega, position, velocity)
    max_theta = abs(theta)
    max_velocity = 0.0
    saturated_steps = 0
    failed_at = None
    for step in range(501):
        now = step * DT
        # Match the 30 Hz camera feeding a 100 Hz control loop: measurements
        # are held between camera frames.
        if step == 0 or step % 3 == 0:
            measured = (theta, omega, position, velocity)
        state = CartPoleState(
            timestamp_s=now,
            theta_rad=measured[0],
            omega_rad_s=measured[1],
            cart_position_mm=measured[2],
            cart_velocity_mm_s=measured[3],
            valid=True,
            tracking_status=TrackingStatus.OK,
        )
        output = controller.update(state, now)
        target_velocity = output.limited_velocity_mm_s
        acceleration = (target_velocity - velocity) / DT
        velocity = target_velocity
        position += velocity * DT
        omega += (GRAVITY_MM_S2 * theta - acceleration) / LENGTH_MM * DT
        theta += omega * DT
        max_theta = max(max_theta, abs(theta))
        max_velocity = max(max_velocity, abs(velocity))
        saturated_steps += int(abs(target_velocity) >= 79.999)
        if failed_at is None and abs(theta) >= 0.12:
            failed_at = now
            break
    return {
        "position_output_sign": position_output_sign,
        "failed_at_s": failed_at,
        "final_theta_rad": theta,
        "final_position_mm": position,
        "final_velocity_mm_s": velocity,
        "max_abs_theta_rad": max_theta,
        "max_abs_velocity_mm_s": max_velocity,
        "saturation_fraction": saturated_steps / max(1, step + 1),
    }


print(json.dumps({str(sign): simulate(sign) for sign in (1, -1)}, indent=2))
