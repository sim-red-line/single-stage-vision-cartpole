"""Timestamp-driven cascaded position/angle controller.

The outer position loop produces a small pendulum angle setpoint. The inner loop
produces velocity or integrated acceleration. An optional energy-shaping stage
swings the pendulum up before handing over to the balance loop.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace

from .models import CartPoleState, ControlOutput, PIDGains


@dataclass(slots=True)
class PIDTerms:
    output: float
    p: float
    i: float
    d: float
    saturated: bool


class FilteredPID:
    """PID with derivative-on-measurement, back-calculation and bumpless gains."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float,
        output_max: float,
        derivative_cutoff_hz: float = 15.0,
        anti_windup_gain: float = 0.3,
    ) -> None:
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_min = float(output_min)
        self.output_max = float(output_max)
        self.derivative_cutoff_hz = float(derivative_cutoff_hz)
        self.anti_windup_gain = float(anti_windup_gain)
        self.integral = 0.0
        self._last_measurement: float | None = None
        self._filtered_derivative = 0.0
        self._last_output = 0.0

    def reset(
        self,
        output: float = 0.0,
        measurement: float | None = None,
        measurement_rate: float = 0.0,
    ) -> None:
        self.integral = 0.0
        self._last_measurement = measurement
        self._filtered_derivative = -float(measurement_rate)
        self._last_output = float(output)

    def set_gains(self, kp: float, ki: float, kd: float, error: float = 0.0) -> None:
        """Change gains while preserving the current output as closely as possible."""

        desired_i_term = self._last_output - float(kp) * error - float(kd) * self._filtered_derivative
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.integral = desired_i_term / self.ki if abs(self.ki) > 1e-12 else 0.0

    def update(
        self,
        setpoint: float,
        measurement: float,
        dt: float,
        measurement_rate: float | None = None,
    ) -> PIDTerms:
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("PID dt must be positive and finite")
        error = setpoint - measurement
        if measurement_rate is not None and math.isfinite(measurement_rate):
            # The vision estimator already computes a timestamp-correct,
            # low-pass-filtered rate at the camera frame rate.  Using it avoids
            # differentiating a 30 Hz sample-and-hold signal again in the
            # 100 Hz control loop, which otherwise produces large D spikes.
            raw_derivative = -float(measurement_rate)
        elif self._last_measurement is not None:
            raw_derivative = -(measurement - self._last_measurement) / dt
        else:
            raw_derivative = 0.0
        if self.derivative_cutoff_hz > 0:
            tau = 1.0 / (2.0 * math.pi * self.derivative_cutoff_hz)
            alpha = dt / (tau + dt)
        else:
            alpha = 1.0
        self._filtered_derivative += alpha * (raw_derivative - self._filtered_derivative)

        p_term = self.kp * error
        d_term = self.kd * self._filtered_derivative
        candidate_integral = self.integral + error * dt
        unsaturated = p_term + self.ki * candidate_integral + d_term
        output = min(self.output_max, max(self.output_min, unsaturated))
        saturated = not math.isclose(output, unsaturated, rel_tol=0.0, abs_tol=1e-12)

        # Integrate normally when not driving farther into saturation, then apply
        # a small back-calculation term to unwind existing integral state.
        drives_out = (output >= self.output_max and error < 0) or (
            output <= self.output_min and error > 0
        )
        if not saturated or drives_out:
            self.integral = candidate_integral
        if saturated and abs(self.ki) > 1e-12:
            self.integral += self.anti_windup_gain * (output - unsaturated) / self.ki

        i_term = self.ki * self.integral
        self._last_measurement = measurement
        self._last_output = output
        return PIDTerms(output, p_term, i_term, d_term, saturated)


class CascadedPIDController:
    def __init__(self, config: dict, gains: PIDGains) -> None:
        self.config = config
        self.output_mode = str(config.get("output_mode", "velocity"))
        if self.output_mode not in {"velocity", "acceleration"}:
            raise ValueError("controller.output_mode must be 'velocity' or 'acceleration'")
        max_angle = float(config["max_angle_setpoint_rad"])
        max_velocity = float(config["max_velocity_mm_s"])
        max_acceleration = float(config["max_acceleration_mm_s2"])
        cutoff = float(config.get("derivative_cutoff_hz", 18.0))
        aw = float(config.get("anti_windup_gain", 0.35))
        self.position = FilteredPID(
            gains.kp_position,
            gains.ki_position,
            gains.kd_position,
            -max_angle,
            max_angle,
            cutoff,
            aw,
        )
        self.angle = FilteredPID(
            gains.kp_angle,
            gains.ki_angle,
            gains.kd_angle,
            -(max_acceleration if self.output_mode == "acceleration" else max_velocity),
            max_acceleration if self.output_mode == "acceleration" else max_velocity,
            cutoff,
            aw,
        )
        self.gains = gains
        self._last_timestamp: float | None = None
        self._last_limited_velocity = 0.0
        self._active = False

    def reset(
        self,
        timestamp_s: float | None = None,
        *,
        velocity_mm_s: float = 0.0,
        state: CartPoleState | None = None,
    ) -> None:
        self.position.reset(
            measurement=state.cart_position_mm if state else None,
            measurement_rate=float(state.cart_velocity_mm_s or 0.0) if state else 0.0,
        )
        self.angle.reset(
            measurement=state.theta_rad if state else None,
            measurement_rate=float(state.omega_rad_s or 0.0) if state else 0.0,
        )
        self._last_timestamp = timestamp_s
        self.accept_applied_velocity(velocity_mm_s)
        self._active = state is not None

    def accept_applied_velocity(self, velocity_mm_s: float) -> None:
        """Track the command actually admitted by the outer rail safety layer."""
        if not math.isfinite(velocity_mm_s):
            raise ValueError("applied velocity must be finite")
        limit = float(self.config["max_velocity_mm_s"])
        self._last_limited_velocity = min(limit, max(-limit, velocity_mm_s))

    def apply_gains(self, gains: PIDGains, state: CartPoleState | None = None) -> None:
        position_error = 0.0
        angle_error = 0.0
        if state and state.valid:
            position_error = float(self.config["cart_center_mm"]) - float(state.cart_position_mm)
            angle_error = -float(state.theta_rad)
        self.position.set_gains(
            gains.kp_position, gains.ki_position, gains.kd_position, position_error
        )
        self.angle.set_gains(gains.kp_angle, gains.ki_angle, gains.kd_angle, angle_error)
        self.gains = gains

    def update(self, state: CartPoleState, now_s: float) -> ControlOutput:
        if not state.valid:
            self.reset(now_s)
            return ControlOutput(now_s, 0.0, 0.0, 0.0, active=False, reason=state.reason)
        if self._last_timestamp is None:
            self._last_timestamp = now_s
            return ControlOutput(now_s, 0.0, 0.0, 0.0, active=False, reason="first sample")
        dt = now_s - self._last_timestamp
        self._last_timestamp = now_s
        if dt <= 0.0 or dt > 0.25:
            self.reset(now_s)
            return ControlOutput(now_s, 0.0, 0.0, 0.0, active=False, reason="invalid dt")

        theta = float(state.theta_rad)
        if not self._active and abs(theta) > float(self.config["engage_angle_rad"]):
            return ControlOutput(
                now_s, 0.0, 0.0, 0.0, active=False, reason="outside engage angle"
            )
        self._active = True
        position_terms = self.position.update(
            float(self.config["cart_center_mm"]),
            float(state.cart_position_mm),
            dt,
            measurement_rate=(
                float(state.cart_velocity_mm_s)
                if state.cart_velocity_mm_s is not None
                else None
            ),
        )
        target_angle = float(self.config.get("position_output_sign", 1)) * position_terms.output
        angle_terms = self.angle.update(
            target_angle,
            theta,
            dt,
            measurement_rate=(
                float(state.omega_rad_s) if state.omega_rad_s is not None else None
            ),
        )
        signed_angle_output = float(self.config.get("output_sign", 1)) * angle_terms.output
        if self.output_mode == "acceleration":
            # The actuator accepts a velocity target.  Treating the angle PID
            # output directly as velocity cannot create a restoring term
            # proportional to pendulum angle.  Integrating an acceleration PID
            # output supplies that term while preserving the firmware velocity
            # interface and the host-side acceleration/velocity limits.
            requested = self._last_limited_velocity + signed_angle_output * dt
        else:
            requested = signed_angle_output

        max_delta = float(self.config["max_acceleration_mm_s2"]) * dt
        low = self._last_limited_velocity - max_delta
        high = self._last_limited_velocity + max_delta
        limited = min(high, max(low, requested))
        max_velocity = float(self.config["max_velocity_mm_s"])
        limited = min(max_velocity, max(-max_velocity, limited))
        slew_limited = not math.isclose(limited, requested, abs_tol=1e-9)
        self._last_limited_velocity = limited
        return ControlOutput(
            timestamp_s=now_s,
            target_angle_rad=target_angle,
            requested_velocity_mm_s=requested,
            limited_velocity_mm_s=limited,
            angle_p=angle_terms.p,
            angle_i=angle_terms.i,
            angle_d=angle_terms.d,
            position_p=position_terms.p,
            position_i=position_terms.i,
            position_d=position_terms.d,
            saturated=angle_terms.saturated or position_terms.saturated or slew_limited,
            active=True,
        )


class HybridSwingUpController:
    """Energy-shaping swing-up with a bumpless handoff to the balance PID.

    The angle convention is theta=0 at upright. The energy expression is
    normalized by pendulum mass, so only pendulum length is required. The
    output is a bounded cart-velocity target; all rail, speed, acceleration,
    camera, and power limits remain enforced by the runtime and RP2040.
    """

    def __init__(self, config: dict, gains: PIDGains) -> None:
        self.config = config
        swing = config.get("swing_up", {})
        enabled = swing.get("enabled", False)
        if not isinstance(enabled, bool):
            raise ValueError("controller.swing_up.enabled must be a boolean")
        self.enabled = enabled

        def number(name: str, default: float, *, zero_allowed: bool = False) -> float:
            value = float(swing.get(name, default))
            if not math.isfinite(value) or value < 0.0 or (value == 0.0 and not zero_allowed):
                raise ValueError(f"controller.swing_up.{name} must be finite and "
                                 + ("nonnegative" if zero_allowed else "positive"))
            return value

        self.length_m = number("pendulum_length_m", 0.226)
        self.energy_gain = number("energy_gain", 1.0)
        self.energy_target_margin = number("energy_target_margin", 0.0, zero_allowed=True)
        if self.energy_target_margin > 0.05:
            raise ValueError("swing-up energy_target_margin must be <= 0.05")
        self.energy_rate_lag = number("energy_rate_lag_s", 0.0, zero_allowed=True)
        if self.energy_rate_lag > 0.06:
            raise ValueError("swing-up energy_rate_lag_s must be <= 0.06")
        self.prediction_latency = number("prediction_camera_latency_s", 0.0, zero_allowed=True)
        self.balance_prediction_latency = number(
            "balance_prediction_camera_latency_s", 0.0, zero_allowed=True
        )
        # Keep balance feedback timing independent of energy shaping. Missing
        # or null preserves the previous shared-lag configuration behavior.
        self.balance_rate_lag = (
            self.energy_rate_lag if swing.get("balance_rate_lag_s") is None
            else number("balance_rate_lag_s", 0.0, zero_allowed=True)
        )
        if self.balance_rate_lag > 0.06:
            raise ValueError("balance_rate_lag_s must be <= 0.06")
        self.prediction_damping = number("prediction_damping_s", 0.0, zero_allowed=True)
        self.prediction_friction = number("prediction_coulomb_rad_s2", 0.0, zero_allowed=True)
        if (self.prediction_latency > 0.06 or self.balance_prediction_latency > 0.06
                or self.prediction_damping > 0.5 or self.prediction_friction > 2.0):
            raise ValueError("swing-up prediction parameters exceed their bounded ranges")
        if self.prediction_latency and self.balance_prediction_latency:
            raise ValueError("choose either full swing-up or balance-only prediction")
        self.reference_omega = number("reference_omega_rad_s", 8.0)
        global_acceleration = float(config["max_acceleration_mm_s2"])
        global_velocity = float(config["max_velocity_mm_s"])
        if not all(math.isfinite(v) and v > 0 for v in (global_acceleration, global_velocity)):
            raise ValueError("controller velocity and acceleration limits must be positive and finite")
        self.max_acceleration = min(number("max_acceleration_mm_s2", 250.0), global_acceleration)
        self.max_velocity = min(number("max_velocity_mm_s", 100.0), global_velocity)
        self.balance_velocity = min(
            number("balance_max_velocity_mm_s", self.max_velocity), global_velocity
        ) if self.enabled else global_velocity
        if self.enabled and self.balance_velocity < self.max_velocity:
            raise ValueError("balance velocity limit must be at least the swing-up limit")
        engage_angle = float(config["engage_angle_rad"])
        if not math.isfinite(engage_angle) or not 0 < engage_angle < math.pi:
            raise ValueError("controller.engage_angle_rad must be finite and in (0, pi)")
        self.capture_angle = min(number("capture_angle_rad", 0.16), engage_angle)
        self.capture_omega = number("capture_omega_rad_s", 1.5)
        self.capture_headroom = number("capture_velocity_headroom_fraction", 0.0, zero_allowed=True)
        if self.capture_headroom >= 1.0:
            raise ValueError("swing-up capture_velocity_headroom_fraction must be < 1")
        self.balance_acceleration = global_acceleration
        self.release_angle = number("release_angle_rad", 0.28)
        self.max_omega = number("max_omega_rad_s", 16.0)
        if not 0 < self.capture_angle < self.release_angle < math.pi:
            raise ValueError("swing-up angles must satisfy 0 < capture < release < pi")
        if self.capture_omega > self.max_omega:
            raise ValueError("swing-up capture angular speed exceeds its maximum angular speed")
        self.kick_acceleration = min(
            number("kick_acceleration_mm_s2", 60.0, zero_allowed=True), self.max_acceleration
        )
        self.kick_duration = number("kick_duration_s", 0.15, zero_allowed=True)
        kick_direction = float(swing.get("kick_direction", 1.0))
        if kick_direction not in {-1.0, 1.0}:
            raise ValueError("controller.swing_up.kick_direction must be +1 or -1")
        self.kick_direction = kick_direction
        self.kick_angle = number("kick_downward_tolerance_rad", 0.35)
        self.kick_omega = number("kick_omega_rad_s", 0.15)
        self.cart_kp = number("cart_position_gain_s2", 0.8, zero_allowed=True)
        self.cart_kd = number("cart_velocity_gain_s", 1.5, zero_allowed=True)
        self.centering_fraction = number("centering_acceleration_fraction", 0.3, zero_allowed=True)
        if self.centering_fraction > 1.0:
            raise ValueError("swing-up centering_acceleration_fraction must be <= 1")
        self.upright_centering_window = number(
            "upright_centering_window_rad", 0.0, zero_allowed=True
        )
        if self.upright_centering_window > 1.0:
            raise ValueError("swing-up upright_centering_window_rad must be <= 1.0")
        self.max_excursion = number("max_cart_excursion_mm", 160.0)
        self.max_duration = number("max_duration_s", 30.0)
        self.max_state_age = number("maximum_state_age_s", 0.12)
        self.cart_center = float(config["cart_center_mm"])
        if not math.isfinite(self.cart_center):
            raise ValueError("controller.cart_center_mm must be finite")
        # Balance can reserve speed above the swing-up target within the host
        # envelope. Recovery slews back to the lower swing target below.
        balance_config = dict(config)
        if self.enabled:
            balance_config["max_velocity_mm_s"] = self.balance_velocity
        self.balance = CascadedPIDController(balance_config, gains)
        self.gravity = 9.80665
        self.reset()

    @property
    def swinging(self) -> bool:
        return self.enabled and self.mode == "swing_up"

    def reset(self, timestamp_s: float | None = None) -> None:
        self.balance.reset(timestamp_s)
        self.mode = "idle"
        self._last_timestamp = timestamp_s
        self._kick_until: float | None = None
        self._velocity = 0.0
        self.swing_elapsed_s = 0.0
        self.energy_error: float | None = None
        self._energy_angles: deque[tuple[float, float]] = deque(maxlen=16)
        self._applied_accelerations: deque[tuple[float, float]] = deque(maxlen=64)
        self._applied_time = timestamp_s
        self._applied_velocity = 0.0
        self._actuator_time = timestamp_s
        self._actuator_velocity = 0.0
        self._actuator_segments: deque[tuple[float, float, float]] = deque(maxlen=256)
        self.predicted_theta: float | None = None
        self.predicted_omega: float | None = None
        self.fault_reason = ""

    def accept_applied_velocity(self, velocity_mm_s: float) -> None:
        now = self._last_timestamp
        if not math.isfinite(velocity_mm_s):
            raise ValueError("applied velocity must be finite")
        limit = self.balance_velocity
        velocity_mm_s = min(limit, max(-limit, velocity_mm_s))
        if now is not None and self._applied_time is not None and now > self._applied_time:
            acceleration = (velocity_mm_s - self._applied_velocity) / (now - self._applied_time)
            limit = float(self.config["max_acceleration_mm_s2"])
            self._applied_accelerations.append((now, min(limit, max(-limit, acceleration))))
        self._applied_time = now
        self._applied_velocity = velocity_mm_s
        self.balance.accept_applied_velocity(velocity_mm_s)
        self._velocity = velocity_mm_s

    def snapshot(self) -> dict:
        return {
            "enabled": self.enabled,
            "mode": self.mode if self.enabled else "balance_only",
            "energy_error": self.energy_error,
            "swing_elapsed_s": self.swing_elapsed_s,
            "fault": self.fault_reason or None,
            "pendulum_length_m": self.length_m,
            "capture_angle_rad": self.capture_angle,
            "capture_omega_rad_s": self.capture_omega,
            "capture_velocity_headroom_fraction": self.capture_headroom,
            "max_velocity_mm_s": self.max_velocity,
            "balance_max_velocity_mm_s": self.balance_velocity,
            "max_acceleration_mm_s2": self.max_acceleration,
            "max_cart_excursion_mm": self.max_excursion,
            "max_duration_s": self.max_duration,
            "energy_gain": self.energy_gain,
            "energy_target_margin": self.energy_target_margin,
            "energy_rate_lag_s": self.energy_rate_lag,
            "prediction_camera_latency_s": self.prediction_latency,
            "balance_prediction_camera_latency_s": self.balance_prediction_latency,
            "balance_rate_lag_s": self.balance_rate_lag,
            "prediction_damping_s": self.prediction_damping,
            "prediction_coulomb_rad_s2": self.prediction_friction,
            "predicted_theta_rad": self.predicted_theta,
            "predicted_omega_rad_s": self.predicted_omega,
            "kick_acceleration_mm_s2": self.kick_acceleration,
            "kick_duration_s": self.kick_duration,
            "kick_direction": self.kick_direction,
            "cart_position_gain_s2": self.cart_kp,
            "cart_velocity_gain_s": self.cart_kd,
            "upright_centering_window_rad": self.upright_centering_window,
        }

    def _fault(self, now_s: float, reason: str) -> ControlOutput:
        self.mode = "fault"
        self.fault_reason = reason
        self._velocity = 0.0
        return ControlOutput(now_s, 0.0, 0.0, 0.0, active=False, reason=reason)

    def _capture(self, state: CartPoleState, now_s: float) -> ControlOutput:
        try:
            balance_state = self._balance_state(state, now_s)
        except ValueError as exc:
            return self._fault(now_s, str(exc))
        self.mode = "balance"
        # Preserve the last admitted velocity rather than commanding STOP at
        # the top. Seed both derivative filters with physical estimated rates.
        self.balance.reset(now_s, velocity_mm_s=self._velocity, state=balance_state)
        return ControlOutput(
            now_s, 0.0, self._velocity, self._velocity,
            active=True, reason="swing-up captured; balance PID engaged",
        )

    def _energy_angle(
        self, timestamp_s: float, theta: float, *, rate_lag_s: float | None = None
    ) -> float:
        """Align potential energy with the estimator's delayed angular rate.

        A backward difference plus a causal rate filter lags the current angle.
        Interpolate measured angle history instead of predicting angular rates
        with an uncertain plant model. Capture still uses the current state.
        """
        if not self._energy_angles:
            self._energy_angles.append((timestamp_s, theta))
        elif timestamp_s > self._energy_angles[-1][0]:
            previous = self._energy_angles[-1][1]
            unwrapped = previous + math.atan2(math.sin(theta - previous), math.cos(theta - previous))
            self._energy_angles.append((timestamp_s, unwrapped))
        lag = self.energy_rate_lag if rate_lag_s is None else rate_lag_s
        target = timestamp_s - lag
        if target <= self._energy_angles[0][0]:
            return self._energy_angles[0][1]
        for (ta, a), (tb, b) in zip(self._energy_angles, list(self._energy_angles)[1:]):
            if ta <= target <= tb:
                return a + (b - a) * (target - ta) / (tb - ta)
        return self._energy_angles[-1][1]

    def _predict_angle(self, theta: float, omega: float, start_s: float, now_s: float) -> tuple[float, float]:
        """Propagate time-aligned vision using already admitted motor commands.

        The forecast never changes measured rail coordinates or safety age.
        A fixed 140 ms horizon bound prevents long extrapolation of lost vision.
        """
        horizon = now_s - start_s
        if not 0.0 <= horizon <= 0.14:
            raise ValueError("swing-up prediction horizon exceeded")
        steps = max(1, math.ceil(horizon / 0.005))
        dt = horizon / steps
        commands = list(self._applied_accelerations)
        cursor = 0
        acceleration = 0.0
        length_mm = self.length_m * 1000.0

        def angular_acceleration(angle: float, rate: float, cart_accel: float) -> float:
            return ((self.gravity * 1000.0 * math.sin(angle) - cart_accel * math.cos(angle))
                    / length_mm - self.prediction_damping * rate
                    - self.prediction_friction * math.tanh(rate / 0.02))

        for step in range(steps):
            middle_s = start_s + (step + 0.5) * dt
            while cursor < len(commands) and commands[cursor][0] <= middle_s:
                acceleration = commands[cursor][1]
                cursor += 1
            alpha = angular_acceleration(theta, omega, acceleration)
            middle_theta = theta + 0.5 * dt * omega
            middle_omega = omega + 0.5 * dt * alpha
            theta += dt * middle_omega
            omega += dt * angular_acceleration(middle_theta, middle_omega, acceleration)
        return math.atan2(math.sin(theta), math.cos(theta)), omega

    def _advance_balance_actuator(self, now_s: float) -> None:
        """Model the previous admitted target under the motor's acceleration cap.

        A rail STOP can drop the admitted target instantly. The physical motor
        continues braking, so differentiating successive targets is insufficient.
        Save both the acceleration interval and any remaining constant-speed part.
        """
        if self._actuator_time is None:
            self._actuator_time = now_s
            return
        start = self._actuator_time
        duration = now_s - start
        if duration <= 0.0:
            return
        difference = self._applied_velocity - self._actuator_velocity
        accelerating_s = min(duration, abs(difference) / self.balance_acceleration)
        acceleration = math.copysign(self.balance_acceleration, difference) if difference else 0.0
        if accelerating_s > 0.0:
            self._actuator_segments.append((start, start + accelerating_s, acceleration))
            self._actuator_velocity += acceleration * accelerating_s
        if accelerating_s < duration:
            self._actuator_segments.append((start + accelerating_s, now_s, 0.0))
        self._actuator_time = now_s

    def _balance_state(self, state: CartPoleState, now_s: float) -> CartPoleState:
        """Forecast only angle/rate for balance; keep measured rail state and age.

        The camera delay excludes frame age. The rate estimator's separate lag
        aligns its rate with historical measured angle before propagation. Energy
        shaping, capture eligibility, release and runtime safety use raw vision.
        """
        if not self.balance_prediction_latency:
            return state
        theta = self._energy_angle(
            float(state.timestamp_s), float(state.theta_rad), rate_lag_s=self.balance_rate_lag
        )
        omega = float(state.omega_rad_s)
        start = state.timestamp_s - self.balance_prediction_latency - self.balance_rate_lag
        horizon = now_s - start
        if not 0.0 <= horizon <= 0.14:
            raise ValueError("balance prediction horizon exceeded")
        segments = list(self._actuator_segments)
        cursor = 0
        steps = max(1, math.ceil(horizon / 0.0025))
        dt = horizon / steps
        length_mm = self.length_m * 1000.0

        def angular_acceleration(angle: float, rate: float, acceleration: float) -> float:
            return ((self.gravity * 1000.0 * math.sin(angle) - acceleration * math.cos(angle))
                    / length_mm - self.prediction_damping * rate
                    - self.prediction_friction * math.tanh(rate / 0.02))

        for index in range(steps):
            middle_s = start + (index + 0.5) * dt
            while cursor < len(segments) and segments[cursor][1] <= middle_s:
                cursor += 1
            acceleration = (
                segments[cursor][2]
                if cursor < len(segments) and segments[cursor][0] <= middle_s else 0.0
            )
            alpha = angular_acceleration(theta, omega, acceleration)
            middle_theta = theta + 0.5 * dt * omega
            middle_omega = omega + 0.5 * dt * alpha
            theta += dt * middle_omega
            omega += dt * angular_acceleration(middle_theta, middle_omega, acceleration)
        theta = math.atan2(math.sin(theta), math.cos(theta))
        if not math.isfinite(theta) or not math.isfinite(omega) or abs(omega) > self.max_omega:
            raise ValueError("invalid predicted balance state")
        self.predicted_theta, self.predicted_omega = theta, omega
        return replace(state, theta_rad=theta, omega_rad_s=omega)

    def apply_gains(self, gains: PIDGains, state: CartPoleState | None = None) -> None:
        self.balance.apply_gains(gains, state)

    def update(self, state: CartPoleState, now_s: float) -> ControlOutput:
        if not self.enabled:
            return self.balance.update(state, now_s)
        if self.fault_reason:
            return self._fault(now_s, self.fault_reason)
        values = (
            state.theta_rad, state.omega_rad_s, state.cart_position_mm,
            state.cart_velocity_mm_s, state.timestamp_s, now_s,
        )
        if not state.valid or any(value is None for value in values):
            return self._fault(now_s, state.reason or "invalid swing-up state")
        if not all(math.isfinite(float(value)) for value in values):
            return self._fault(now_s, "non-finite swing-up state")
        if now_s - state.timestamp_s > self.max_state_age:
            return self._fault(now_s, "swing-up camera state stale")

        # The energy is periodic; capture must be periodic too after a full
        # rotation and must not depend on the estimator's pi wrap direction.
        theta = math.atan2(math.sin(float(state.theta_rad)), math.cos(float(state.theta_rad)))
        state = replace(state, theta_rad=theta)
        omega = float(state.omega_rad_s)
        if abs(omega) > self.max_omega:
            return self._fault(now_s, "swing-up angular-speed limit")
        position = float(state.cart_position_mm)
        velocity = float(state.cart_velocity_mm_s)
        if abs(position - self.cart_center) >= self.max_excursion:
            return self._fault(now_s, "swing-up cart excursion limit")

        # The first active tick establishes the clock; idle time must not be
        # integrated as a startup acceleration interval.
        dt = 0.0 if self.mode == "idle" or self._last_timestamp is None else now_s - self._last_timestamp
        if self.mode != "idle" and self._last_timestamp is not None and (dt <= 0.0 or dt > 0.12):
            return self._fault(now_s, "invalid swing-up control interval")
        self._last_timestamp = now_s
        if self.balance_prediction_latency:
            self._advance_balance_actuator(now_s)
        energy_theta = self._energy_angle(float(state.timestamp_s), theta)
        if self.prediction_latency > 0.0:
            start_s = float(state.timestamp_s) - self.prediction_latency - self.energy_rate_lag
            try:
                theta, omega = self._predict_angle(energy_theta, omega, start_s, now_s)
            except ValueError as exc:
                return self._fault(now_s, str(exc))
            if not math.isfinite(theta) or not math.isfinite(omega) or abs(omega) > self.max_omega:
                return self._fault(now_s, "predicted swing-up angular-speed limit")
            state = replace(state, theta_rad=theta, omega_rad_s=omega)
            energy_theta = theta
            self.predicted_theta, self.predicted_omega = theta, omega
        self.energy_error = (
            0.5 * self.length_m * omega * omega / self.gravity + math.cos(energy_theta) - 1.0
            - self.energy_target_margin
        )
        can_capture = abs(theta) <= self.capture_angle and abs(omega) <= self.capture_omega
        if can_capture and self.capture_headroom > 0.0:
            # Near upright, z = omega + sqrt(g/L)*theta is the divergent
            # angular mode. L*z estimates the cart velocity change needed to
            # arrest it. Reserve room in both velocity and acceleration before
            # handing over; an angle/rate window alone can capture at a speed
            # limit where the balance PID has no authority in one direction.
            natural_rate = math.sqrt(self.gravity / self.length_m)
            delta_velocity = self.length_m * 1000.0 * (omega + natural_rate * theta)
            reserve = 1.0 - self.capture_headroom
            can_capture = (
                abs(self._velocity + delta_velocity) <= self.balance_velocity * reserve
                and abs(natural_rate * delta_velocity) <= self.balance_acceleration * reserve
            )
        if self.mode == "idle" or (self.mode == "balance" and abs(theta) > self.release_angle):
            if can_capture:
                return self._capture(state, now_s)
            self.mode = "swing_up"
            self.balance.reset()
            self._kick_until = None
            if math.pi - abs(theta) <= self.kick_angle and abs(omega) <= self.kick_omega:
                self._kick_until = now_s + self.kick_duration

        if self.mode == "balance":
            try:
                balance_state = self._balance_state(state, now_s)
            except ValueError as exc:
                return self._fault(now_s, str(exc))
            control = self.balance.update(balance_state, now_s)
            self._velocity = control.limited_velocity_mm_s
            if control.active:
                control.reason = "balance PID"
            return control

        if can_capture:
            return self._capture(state, now_s)
        self.swing_elapsed_s += dt
        if self.swing_elapsed_s >= self.max_duration:
            return self._fault(now_s, "swing-up time limit")

        # With theta=0 upright, e_dot = -a * omega * cos(theta) / g.
        # Positive gain on e*omega*cos(theta) therefore reduces e^2.
        phase = self.energy_error * (omega / self.reference_omega) * math.cos(theta)
        acceleration = self.max_acceleration * math.tanh(self.energy_gain * phase)
        kicking = self._kick_until is not None and now_s < self._kick_until
        if kicking:
            acceleration = self.kick_direction * self.kick_acceleration
        center_limit = self.max_acceleration * self.centering_fraction
        centering = self.cart_kp * (self.cart_center - position) - self.cart_kd * velocity
        centering = min(center_limit, max(-center_limit, centering))
        if self.upright_centering_window:
            # Preserve energy authority near upright. The rail/braking envelope
            # still independently limits the final admitted velocity.
            centering *= min(1.0, (abs(theta) / self.upright_centering_window) ** 2)
        raw_acceleration = acceleration + centering
        acceleration = min(self.max_acceleration, max(-self.max_acceleration, raw_acceleration))
        # Integrate the previous admitted command, NOT the sample-and-held
        # measured camera velocity. This works correctly at 100 Hz / 30 Hz.
        requested = self._velocity + acceleration * dt
        target = min(self.max_velocity, max(-self.max_velocity, requested))
        # A release can start above the swing target after balance used its
        # reserved speed. Decelerate under the swing acceleration cap.
        limited = min(self._velocity + self.max_acceleration * dt,
                      max(self._velocity - self.max_acceleration * dt, target))
        self._velocity = limited
        return ControlOutput(
            timestamp_s=now_s,
            target_angle_rad=0.0,
            requested_velocity_mm_s=requested,
            limited_velocity_mm_s=limited,
            saturated=(abs(raw_acceleration) > self.max_acceleration or not math.isclose(requested, limited)),
            active=True,
            reason=f"{'startup kick' if kicking else 'energy swing-up'} (E={self.energy_error:.3f})",
        )

