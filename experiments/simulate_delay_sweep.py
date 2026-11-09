from __future__ import annotations

import json
from collections import deque

from raspberry_pi.models import CartPoleState, PIDGains, TrackingStatus
from raspberry_pi.pid_controller import CascadedPIDController


DT = 0.01
LENGTH_MM = 226.0
GRAVITY_MM_S2 = 9806.65


def simulate(
    delay_s: float,
    kp_angle: float = 22000.0,
    kd_angle: float = 3300.0,
    initial_theta: float = 0.00365,
    initial_omega: float = -0.01625,
    initial_position: float = 15.0,
    initial_velocity: float = 0.0,
    duration_s: float = 5.0,
) -> dict:
    controller = CascadedPIDController(
        {
            "output_mode": "acceleration",
            "output_sign": -1,
            "position_output_sign": 1,
            "cart_center_mm": 0.0,
            "engage_angle_rad": 0.12,
            "max_angle_setpoint_rad": 0.10,
            "max_velocity_mm_s": 80.0,
            "max_acceleration_mm_s2": 1000.0,
            "derivative_cutoff_hz": 18.0,
            "anti_windup_gain": 0.35,
        },
        PIDGains(
            kp_angle=kp_angle,
            kd_angle=kd_angle,
            kp_position=0.0000077,
            kd_position=0.0000306,
        ),
    )
    theta = initial_theta
    omega = initial_omega
    position = initial_position
    velocity = initial_velocity
    delay_steps = max(0, round(delay_s / DT))
    history: deque[tuple[float, float, float, float]] = deque(
        [(theta, omega, position, velocity)] * (delay_steps + 1),
        maxlen=delay_steps + 1,
    )
    camera_sample = history[0]
    max_theta = abs(theta)
    max_velocity = 0.0
    saturated_steps = 0
    failed_at = None
    for step in range(round(duration_s / DT) + 1):
        now = step * DT
        if step == 0 or step % 3 == 0:
            camera_sample = history[0]
        state = CartPoleState(
            timestamp_s=now,
            theta_rad=camera_sample[0],
            omega_rad_s=camera_sample[1],
            cart_position_mm=camera_sample[2],
            cart_velocity_mm_s=camera_sample[3],
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
        history.append((theta, omega, position, velocity))
        max_theta = max(max_theta, abs(theta))
        max_velocity = max(max_velocity, abs(velocity))
        saturated_steps += int(abs(target_velocity) >= 79.999)
        if abs(theta) >= 0.12:
            failed_at = now
            break
    return {
        "delay_s": delay_s,
        "kp_angle": kp_angle,
        "kd_angle": kd_angle,
        "failed_at_s": failed_at,
        "final_theta_rad": theta,
        "final_position_mm": position,
        "final_velocity_mm_s": velocity,
        "max_abs_theta_rad": max_theta,
        "max_abs_velocity_mm_s": max_velocity,
        "saturation_fraction": saturated_steps / max(1, step + 1),
    }


if __name__ == "__main__":
    delays = [0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12]
    print(json.dumps([simulate(delay) for delay in delays], indent=2))
