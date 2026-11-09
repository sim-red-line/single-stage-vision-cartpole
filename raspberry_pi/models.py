"""Shared, dependency-light data models for the real-time and dashboard paths."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import StrEnum
from typing import Any


class MarkerRole(StrEnum):
    PENDULUM_TIP = "pendulum_tip"
    PIVOT = "pivot"
    CART = "cart"
    AUXILIARY = "auxiliary"


class TrackingStatus(StrEnum):
    OK = "ok"
    LOW_CONFIDENCE = "low_confidence"
    PARTIAL_LOSS = "partial_loss"
    LOST = "lost"
    AMBIGUOUS = "ambiguous"
    RECALIBRATION_REQUIRED = "recalibration_required"


@dataclass(slots=True)
class MarkerDetection:
    name: str
    role: MarkerRole
    center_px: tuple[float, float] | None
    area_px: float
    confidence: float
    status: TrackingStatus
    timestamp_s: float
    candidates: int = 0

    @property
    def valid(self) -> bool:
        return (
            self.center_px is not None
            and self.status in {TrackingStatus.OK, TrackingStatus.LOW_CONFIDENCE}
        )


@dataclass(slots=True)
class TrackingFrame:
    timestamp_s: float
    detections: dict[str, MarkerDetection]
    status: TrackingStatus
    mean_brightness: float
    processing_ms: float
    frame_sequence: int
    reason: str = ""


@dataclass(slots=True)
class CartPoleState:
    timestamp_s: float
    theta_rad: float | None
    omega_rad_s: float | None
    cart_position_mm: float | None
    cart_velocity_mm_s: float | None
    valid: bool
    tracking_status: TrackingStatus
    age_s: float = 0.0
    reason: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class PIDGains:
    kp_angle: float = 0.0
    ki_angle: float = 0.0
    kd_angle: float = 0.0
    kp_position: float = 0.0
    ki_position: float = 0.0
    kd_position: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return asdict(self)

    @classmethod
    def from_mapping(cls, value: dict[str, Any]) -> "PIDGains":
        fields = cls.__dataclass_fields__
        return cls(**{name: float(value[name]) for name in fields})


@dataclass(slots=True)
class ControlOutput:
    timestamp_s: float
    target_angle_rad: float
    requested_velocity_mm_s: float
    limited_velocity_mm_s: float
    angle_p: float = 0.0
    angle_i: float = 0.0
    angle_d: float = 0.0
    position_p: float = 0.0
    position_i: float = 0.0
    position_d: float = 0.0
    saturated: bool = False
    active: bool = False
    reason: str = ""


@dataclass(slots=True)
class RuntimeSnapshot:
    timestamp_s: float
    safety_state: str
    safety_reason: str
    tracking: dict[str, Any] = field(default_factory=dict)
    state: dict[str, Any] = field(default_factory=dict)
    control: dict[str, Any] = field(default_factory=dict)
    motor: dict[str, Any] = field(default_factory=dict)
    calibration: dict[str, Any] = field(default_factory=dict)
    experiment: dict[str, Any] = field(default_factory=dict)
    gain_source: str = "config"
    gains: dict[str, float] = field(default_factory=dict)
    fps: float = 0.0
    control_latency_ms: float = 0.0

