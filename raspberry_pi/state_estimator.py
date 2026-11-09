"""Convert calibrated marker coordinates into cart-pole state."""

from __future__ import annotations

import math
from typing import Any

from .calibration import CoordinateCalibration
from .color_tracker import detections_by_role
from .models import CartPoleState, MarkerDetection, MarkerRole, TrackingFrame, TrackingStatus


def _first_valid(items: list[MarkerDetection] | None) -> MarkerDetection | None:
    for item in items or []:
        if item.valid:
            return item
    return None


class StateEstimator:
    def __init__(self, coordinates: CoordinateCalibration, config: dict[str, Any]) -> None:
        self.coordinates = coordinates
        self.config = config
        self.cutoff_hz = float(config.get("derivative_cutoff_hz", 12.0))
        self.maximum_dt_s = float(config.get("maximum_dt_s", 0.12))
        self._last_timestamp: float | None = None
        self._last_theta_unwrapped: float | None = None
        self._last_position: float | None = None
        self._omega = 0.0
        self._velocity = 0.0

    def reset(self) -> None:
        self._last_timestamp = None
        self._last_theta_unwrapped = None
        self._last_position = None
        self._omega = 0.0
        self._velocity = 0.0

    def _invalid(self, tracking: TrackingFrame, reason: str) -> CartPoleState:
        # Deliberately do not expose the previous state as a new measurement.
        return CartPoleState(
            timestamp_s=tracking.timestamp_s,
            theta_rad=None,
            omega_rad_s=None,
            cart_position_mm=None,
            cart_velocity_mm_s=None,
            valid=False,
            tracking_status=tracking.status,
            reason=reason,
        )

    def update(self, tracking: TrackingFrame) -> CartPoleState:
        if tracking.status not in {TrackingStatus.OK, TrackingStatus.LOW_CONFIDENCE}:
            self.reset()
            return self._invalid(tracking, tracking.reason or str(tracking.status))
        if not self.coordinates.valid:
            self.reset()
            return self._invalid(tracking, "coordinate calibration is incomplete")
        grouped = detections_by_role(tracking)
        tip = _first_valid(grouped.get(MarkerRole.PENDULUM_TIP))
        pivot = _first_valid(grouped.get(MarkerRole.PIVOT))
        cart = _first_valid(grouped.get(MarkerRole.CART))
        if tip is None or tip.center_px is None:
            self.reset()
            return self._invalid(tracking, "pendulum tip marker missing")
        if pivot is not None and pivot.center_px is not None:
            pivot_px = pivot.center_px
        elif cart is not None and cart.center_px is not None and self.coordinates.pivot_y_px is not None:
            pivot_px = (cart.center_px[0], float(self.coordinates.pivot_y_px))
        else:
            self.reset()
            return self._invalid(tracking, "pivot cannot be measured or inferred")

        cart_point = (
            cart.center_px
            if cart is not None and cart.center_px is not None
            else pivot_px
        )
        try:
            position = self.coordinates.pixel_to_rail_mm(cart_point)
        except ValueError as exc:
            self.reset()
            return self._invalid(tracking, str(exc))

        dx = (tip.center_px[0] - pivot_px[0]) * self.coordinates.camera_x_sign
        dy_up = pivot_px[1] - tip.center_px[1]
        if math.hypot(dx, dy_up) < 5.0:
            self.reset()
            return self._invalid(tracking, "tip and pivot are too close for a reliable angle")
        if (
            self.coordinates.pendulum_length_mm is not None
            and self.coordinates.homography is None
        ):
            mm_per_pixel = self.coordinates.rail_length_mm / abs(
                float(self.coordinates.rail_right_px)
                - float(self.coordinates.rail_left_px)
            )
            observed_length = math.hypot(dx, dy_up) * mm_per_pixel
            expected_length = float(self.coordinates.pendulum_length_mm)
            tolerance = float(
                self.config.get("pendulum_length_tolerance_ratio", 0.40)
            )
            if abs(observed_length - expected_length) > expected_length * tolerance:
                self.reset()
                return self._invalid(
                    tracking,
                    f"observed pendulum length {observed_length:.1f}mm is inconsistent with calibration",
                )
        try:
            angle_reference = self.coordinates.vertical_reference_rad(position)
        except ValueError as exc:
            self.reset()
            return self._invalid(tracking, str(exc))
        theta_wrapped = math.atan2(dx, dy_up) - angle_reference

        if self._last_timestamp is None:
            self._last_timestamp = tracking.timestamp_s
            self._last_theta_unwrapped = theta_wrapped
            self._last_position = position
            return CartPoleState(
                tracking.timestamp_s,
                theta_wrapped,
                0.0,
                position,
                0.0,
                True,
                tracking.status,
            )
        dt = tracking.timestamp_s - self._last_timestamp
        if dt <= 0.0 or dt > self.maximum_dt_s:
            self.reset()
            return self._invalid(tracking, f"invalid camera timestamp interval ({dt:.4f}s)")
        previous_theta = float(self._last_theta_unwrapped)
        delta = ((theta_wrapped - previous_theta + math.pi) % (2.0 * math.pi)) - math.pi
        theta_unwrapped = previous_theta + delta
        raw_omega = delta / dt
        raw_velocity = (position - float(self._last_position)) / dt
        tau = 1.0 / (2.0 * math.pi * max(self.cutoff_hz, 0.1))
        alpha = dt / (tau + dt)
        self._omega += alpha * (raw_omega - self._omega)
        self._velocity += alpha * (raw_velocity - self._velocity)
        self._last_timestamp = tracking.timestamp_s
        self._last_theta_unwrapped = theta_unwrapped
        self._last_position = position
        return CartPoleState(
            tracking.timestamp_s,
            theta_wrapped,
            self._omega,
            position,
            self._velocity,
            True,
            tracking.status,
        )

