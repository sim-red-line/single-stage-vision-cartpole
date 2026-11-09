"""Central safety state machine for the Raspberry Pi side.

The RP2040 independently enforces its own limits. This layer can only make the
combined system more restrictive; it cannot relax firmware limits.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import StrEnum

from .models import CartPoleState


class SafetyState(StrEnum):
    DISCONNECTED = "DISCONNECTED"
    UNHOMED = "UNHOMED"
    CALIBRATION_REQUIRED = "CALIBRATION_REQUIRED"
    IDLE = "IDLE"
    ARMED = "ARMED"
    RUNNING = "RUNNING"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


@dataclass(slots=True)
class SafetyDecision:
    allowed_velocity_mm_s: float
    stop_required: bool
    state: SafetyState
    reason: str


class SafetyManager:
    def __init__(self, config: dict) -> None:
        self.config = config
        self.state = SafetyState.DISCONNECTED
        self.reason = "motor disconnected"
        self.connected = False
        self.homed = False
        self.calibrated = False
        self.tracking_ok = False
        self.power_ok = True
        self.power_reason = "Pi power status clean"
        self._estop_latched = False
        self._fault_latched = False
        self._lock = threading.RLock()
        self.last_transition_s = time.monotonic()

    def _set_state(self, state: SafetyState, reason: str) -> None:
        if state != self.state or reason != self.reason:
            self.state = state
            self.reason = reason
            self.last_transition_s = time.monotonic()

    def update_prerequisites(
        self,
        *,
        connected: bool | None = None,
        homed: bool | None = None,
        calibrated: bool | None = None,
        tracking_ok: bool | None = None,
        power_ok: bool | None = None,
        power_reason: str | None = None,
    ) -> SafetyState:
        with self._lock:
            if connected is not None:
                self.connected = connected
            if homed is not None:
                self.homed = homed
            if calibrated is not None:
                self.calibrated = calibrated
            if tracking_ok is not None:
                self.tracking_ok = tracking_ok
            if power_ok is not None:
                self.power_ok = power_ok
            if power_reason is not None:
                self.power_reason = power_reason
            if self._estop_latched:
                self._set_state(SafetyState.ESTOP, self.reason)
            elif self._fault_latched:
                self._set_state(SafetyState.FAULT, self.reason)
            elif not self.power_ok:
                self._set_state(SafetyState.FAULT, f"unsafe Pi power: {self.power_reason}")
            elif not self.connected:
                self._set_state(SafetyState.DISCONNECTED, "motor disconnected")
            elif not self.homed:
                self._set_state(SafetyState.UNHOMED, "homing required")
            elif not self.calibrated:
                self._set_state(
                    SafetyState.CALIBRATION_REQUIRED, "valid calibration required"
                )
            elif self.state not in {SafetyState.ARMED, SafetyState.RUNNING}:
                self._set_state(SafetyState.IDLE, "ready")
            return self.state

    def arm(self) -> bool:
        with self._lock:
            self.update_prerequisites()
            if self.state is not SafetyState.IDLE:
                return False
            if not self.tracking_ok:
                self.reason = "reliable live tracking required"
                return False
            self._set_state(SafetyState.ARMED, "armed; waiting for start")
            return True

    def start(self) -> bool:
        with self._lock:
            self.update_prerequisites()
            if self.state is not SafetyState.ARMED or not self.tracking_ok:
                self.reason = "system must be armed with reliable tracking"
                return False
            self._set_state(SafetyState.RUNNING, "controller active")
            return True

    def stop(self, reason: str = "operator stop") -> None:
        with self._lock:
            if self.state not in {SafetyState.ESTOP, SafetyState.FAULT}:
                if not self.power_ok:
                    target = SafetyState.FAULT
                elif not self.connected:
                    target = SafetyState.DISCONNECTED
                elif not self.homed:
                    target = SafetyState.UNHOMED
                elif not self.calibrated:
                    target = SafetyState.CALIBRATION_REQUIRED
                else:
                    target = SafetyState.IDLE
                self._set_state(target, reason)

    def fault(self, reason: str) -> None:
        with self._lock:
            self._fault_latched = True
            self._set_state(SafetyState.FAULT, reason)

    def clear_fault(self) -> bool:
        with self._lock:
            if self._estop_latched:
                return False
            self._fault_latched = False
            self.update_prerequisites()
            return self.state not in {SafetyState.FAULT, SafetyState.ESTOP}

    def estop(self, reason: str = "emergency stop") -> None:
        with self._lock:
            self._estop_latched = True
            self._set_state(SafetyState.ESTOP, reason)

    def clear_estop(self, physical_limits_clear: bool) -> bool:
        with self._lock:
            if not physical_limits_clear:
                self.reason = "cannot clear ESTOP while a limit input is active"
                return False
            self._estop_latched = False
            self._fault_latched = False
            # Firmware still requires its own explicit CLEAR_ESTOP.
            self.update_prerequisites(homed=False)
            return True

    def evaluate_velocity(
        self, requested_mm_s: float, state: CartPoleState, now_s: float,
        *, motion_bounds_mm: tuple[float, float] | None = None,
    ) -> SafetyDecision:
        with self._lock:
            if self.state is not SafetyState.RUNNING:
                return SafetyDecision(0.0, True, self.state, self.reason)
            if not state.valid:
                self.fault(f"invalid tracking state: {state.reason}")
                return SafetyDecision(0.0, True, self.state, self.reason)
            age = now_s - state.timestamp_s
            if age > float(self.config["tracking_grace_s"]):
                self.fault(f"camera state stale ({age:.3f}s)")
                return SafetyDecision(0.0, True, self.state, self.reason)
            x = float(state.cart_position_mm)
            v = float(state.cart_velocity_mm_s)
            rail_min = float(self.config["rail_min_mm"])
            rail_max = float(self.config["rail_max_mm"])
            if motion_bounds_mm is not None:
                low, high = motion_bounds_mm
                if not (math.isfinite(low) and math.isfinite(high) and low < high):
                    self.fault("invalid motion corridor")
                    return SafetyDecision(0.0, True, self.state, self.reason)
                # A narrower operating corridor shares the same conservative
                # stopping distance, latency and margin as the physical rail.
                rail_min, rail_max = max(rail_min, low), min(rail_max, high)
            if not rail_min <= x <= rail_max:
                self.fault(f"cart outside software limits ({x:.1f}mm)")
                return SafetyDecision(0.0, True, self.state, self.reason)

            accel = max(1.0, float(self.config["braking_acceleration_mm_s2"]))
            latency = float(self.config.get("command_latency_s", 0.0))
            margin = float(self.config["braking_margin_mm"])
            # Include continued travel during the command/vision latency.
            stopping_distance = v * v / (2.0 * accel) + abs(v) * latency + margin
            left_remaining = x - rail_min
            right_remaining = rail_max - x
            allowed = float(requested_mm_s)

            if allowed < 0:
                usable = max(0.0, left_remaining - margin)
                max_toward_edge = math.sqrt(max(0.0, 2.0 * accel * usable))
                allowed = max(allowed, -max_toward_edge)
                if v < 0 and left_remaining <= stopping_distance:
                    allowed = max(0.0, allowed)
            elif allowed > 0:
                usable = max(0.0, right_remaining - margin)
                max_toward_edge = math.sqrt(max(0.0, 2.0 * accel * usable))
                allowed = min(allowed, max_toward_edge)
                if v > 0 and right_remaining <= stopping_distance:
                    allowed = min(0.0, allowed)
            return SafetyDecision(allowed, False, self.state, "ok")

