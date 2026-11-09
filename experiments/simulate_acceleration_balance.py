from __future__ import annotations

import math

from raspberry_pi.models import CartPoleState, PIDGains, TrackingStatus
from raspberry_pi.pid_controller import CascadedPIDController


DT = 0.01
LENGTH_MM = 226.0
GRAVITY_MM_S2 = 9806.65

config = {
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

theta = 0.02
omega = 0.0
position = 0.0
velocity = 0.0
max_theta = abs(theta)
samples = []

for step in range(301):
    now = step * DT
    state = CartPoleState(
        timestamp_s=now,
        theta_rad=theta,
        omega_rad_s=omega,
        cart_position_mm=position,
        cart_velocity_mm_s=velocity,
        valid=True,
        tracking_status=TrackingStatus.OK,
    )
    output = controller.update(state, now)
    target_velocity = output.limited_velocity_mm_s
    acceleration = (target_velocity - velocity) / DT
    theta_acceleration = (
        GRAVITY_MM_S2 * theta - acceleration
    ) / LENGTH_MM
    velocity = target_velocity
    position += velocity * DT
    omega += theta_acceleration * DT
    theta += omega * DT
    max_theta = max(max_theta, abs(theta))
    if step <= 100 and step % 10 == 0:
        samples.append(
            {
                "t": round(now, 2),
                "theta": round(theta, 6),
                "omega": round(omega, 6),
                "acceleration": round(acceleration, 3),
                "velocity": round(velocity, 3),
            }
        )
    if not all(math.isfinite(value) for value in (theta, omega, position, velocity)):
        raise SystemExit("simulation became non-finite")

print(
    {
        "final_theta_rad": theta,
        "final_omega_rad_s": omega,
        "final_position_mm": position,
        "final_velocity_mm_s": velocity,
        "max_abs_theta_rad": max_theta,
        "samples": samples,
    }
)
