"""Color, lighting and camera-coordinate calibration.

Profiles are data only. Loading a profile never claims it is currently valid; a
live detection-quality check must pass before the safety state can become IDLE.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .models import MarkerRole


@dataclass(slots=True)
class HSVRange:
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


@dataclass(slots=True)
class MarkerColorProfile:
    name: str
    role: MarkerRole
    hsv_ranges: list[HSVRange]
    min_area_px: float
    max_area_px: float
    sample_hsv_median: tuple[float, float, float]
    required: bool = True


@dataclass(slots=True)
class CoordinateCalibration:
    rail_left_px: float | None = None
    rail_right_px: float | None = None
    rail_length_mm: float = 500.0
    camera_x_sign: int = 1
    pivot_y_px: float | None = None
    vertical_offset_rad: float = 0.0
    pendulum_length_mm: float | None = None
    homography: list[list[float]] | None = None
    vertical_offset_slope_rad_per_mm: float = 0.0
    vertical_offset_reference_mm: float = 0.0

    @property
    def valid(self) -> bool:
        return (
            self.rail_left_px is not None
            and self.rail_right_px is not None
            and abs(self.rail_right_px - self.rail_left_px) >= 20.0
            and self.rail_length_mm > 0.0
            and self.camera_x_sign in {-1, 1}
            and math.isfinite(self.vertical_offset_rad)
            and math.isfinite(self.vertical_offset_slope_rad_per_mm)
            and math.isfinite(self.vertical_offset_reference_mm)
            and (
                self.vertical_offset_slope_rad_per_mm == 0.0
                or (
                    math.isfinite(self.rail_length_mm)
                    and 0.0 <= self.vertical_offset_reference_mm <= self.rail_length_mm
                    and abs(self.vertical_offset_slope_rad_per_mm)
                    * max(self.vertical_offset_reference_mm,
                          self.rail_length_mm - self.vertical_offset_reference_mm)
                    <= math.radians(3.0)
                )
            )
        )

    def vertical_reference_rad(self, position_mm: float) -> float:
        """Measured plumb direction; optional angle correction leaves rail mapping intact."""
        if not math.isfinite(position_mm):
            raise ValueError("non-finite cart position for angle reference")
        correction = self.vertical_offset_slope_rad_per_mm * (
            position_mm - self.vertical_offset_reference_mm
        )
        if not math.isfinite(correction) or abs(correction) > math.radians(3.0):
            raise ValueError("position-dependent angle reference exceeds3degrees")
        reference = self.vertical_offset_rad + correction
        if not math.isfinite(reference):
            raise ValueError("non-finite angle reference")
        return reference

    def pixel_to_rail_mm(self, point: tuple[float, float]) -> float:
        if not self.valid:
            raise ValueError("coordinate calibration is incomplete")
        x, y = point
        if self.homography is not None:
            matrix = np.asarray(self.homography, dtype=np.float64)
            source = np.asarray([[[x, y]]], dtype=np.float64)
            x = float(cv2.perspectiveTransform(source, matrix)[0, 0, 0])
        ratio = (x - float(self.rail_left_px)) / (
            float(self.rail_right_px) - float(self.rail_left_px)
        )
        if self.camera_x_sign < 0:
            ratio = 1.0 - ratio
        return ratio * self.rail_length_mm


@dataclass(slots=True)
class CalibrationProfile:
    name: str
    markers: list[MarkerColorProfile]
    coordinates: CoordinateCalibration
    baseline_brightness: float
    created_at: str
    notes: str = ""
    schema_version: int = 1

    @property
    def structurally_valid(self) -> bool:
        roles = {marker.role for marker in self.markers if marker.required}
        has_geometry = MarkerRole.PENDULUM_TIP in roles and (
            MarkerRole.PIVOT in roles
            or MarkerRole.CART in roles
            or self.coordinates.pivot_y_px is not None
        )
        return bool(self.markers) and has_geometry and self.coordinates.valid

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        for marker in data["markers"]:
            marker["role"] = str(marker["role"])
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CalibrationProfile":
        markers = []
        for item in data.get("markers", []):
            ranges = [
                HSVRange(tuple(entry["lower"]), tuple(entry["upper"]))
                for entry in item.get("hsv_ranges", [])
            ]
            markers.append(
                MarkerColorProfile(
                    name=item["name"],
                    role=MarkerRole(item["role"]),
                    hsv_ranges=ranges,
                    min_area_px=float(item["min_area_px"]),
                    max_area_px=float(item["max_area_px"]),
                    sample_hsv_median=tuple(item["sample_hsv_median"]),
                    required=bool(item.get("required", True)),
                )
            )
        return cls(
            name=data["name"],
            markers=markers,
            coordinates=CoordinateCalibration(**data.get("coordinates", {})),
            baseline_brightness=float(data.get("baseline_brightness", 0.0)),
            created_at=data.get("created_at", "unknown"),
            notes=data.get("notes", ""),
            schema_version=int(data.get("schema_version", 1)),
        )


class ProfileStore:
    def __init__(self, directory: str | Path) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)

    def list_profiles(self) -> list[str]:
        return sorted(path.stem for path in self.directory.glob("*.json"))

    @staticmethod
    def _validate_name(name: str) -> None:
        if not name or any(
            character
            not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
            for character in name
        ):
            raise ValueError("profile name may contain only letters, numbers, '_' and '-'")

    def save(self, profile: CalibrationProfile) -> Path:
        self._validate_name(profile.name)
        target = self.directory / f"{profile.name}.json"
        temporary = target.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(profile.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8"
        )
        temporary.replace(target)
        return target

    def load(self, name: str) -> CalibrationProfile:
        self._validate_name(name)
        target = self.directory / f"{name}.json"
        return CalibrationProfile.from_dict(json.loads(target.read_text(encoding="utf-8")))


def _circular_hue_ranges(hues: np.ndarray, margin: int) -> list[tuple[int, int]]:
    """Return one or two OpenCV hue intervals around robust circular percentiles."""

    angles = hues.astype(np.float64) * (2.0 * math.pi / 180.0)
    center_angle = math.atan2(float(np.sin(angles).mean()), float(np.cos(angles).mean()))
    center_hue = (center_angle * 180.0 / (2.0 * math.pi)) % 180.0
    delta = ((hues.astype(np.float64) - center_hue + 90.0) % 180.0) - 90.0
    low_delta, high_delta = np.percentile(delta, [2.0, 98.0])
    low = center_hue + low_delta - margin
    high = center_hue + high_delta + margin
    width = high - low
    if width >= 179.0:
        return [(0, 179)]
    low_mod, high_mod = low % 180.0, high % 180.0
    if low_mod <= high_mod and low >= 0.0 and high < 180.0:
        return [(int(math.floor(low_mod)), int(math.ceil(high_mod)))]
    return [(0, int(math.ceil(high_mod))), (int(math.floor(low_mod)), 179)]


def calibrate_marker_from_roi(
    frame_bgr: np.ndarray,
    roi: tuple[int, int, int, int],
    *,
    name: str,
    role: MarkerRole,
    required: bool = True,
) -> MarkerColorProfile:
    x, y, width, height = roi
    image_h, image_w = frame_bgr.shape[:2]
    if width < 4 or height < 4:
        raise ValueError("ROI must be at least 4x4 pixels")
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(image_w, x + width), min(image_h, y + height)
    if x1 - x0 < 4 or y1 - y0 < 4:
        raise ValueError("ROI lies outside the image")
    hsv = cv2.cvtColor(frame_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    # Very dark/desaturated pixels mostly describe shadows/background, not hue.
    chromatic = hsv[(hsv[:, 1] >= 35) & (hsv[:, 2] >= 30)]
    if len(chromatic) < max(20, len(hsv) // 10):
        raise ValueError("ROI does not contain enough saturated, visible marker pixels")
    hue_ranges = _circular_hue_ranges(chromatic[:, 0], margin=4)
    s_low, s_high = np.percentile(chromatic[:, 1], [2.0, 99.0])
    v_low, v_high = np.percentile(chromatic[:, 2], [2.0, 99.0])
    s_low = max(25, int(math.floor(s_low)) - 18)
    s_high = min(255, int(math.ceil(s_high)) + 12)
    v_low = max(20, int(math.floor(v_low)) - 22)
    v_high = min(255, int(math.ceil(v_high)) + 18)
    ranges = [
        HSVRange((h_low, s_low, v_low), (h_high, s_high, v_high))
        for h_low, h_high in hue_ranges
    ]
    selected_area = float((x1 - x0) * (y1 - y0))
    median = tuple(float(value) for value in np.median(chromatic, axis=0))
    return MarkerColorProfile(
        name=name,
        role=role,
        hsv_ranges=ranges,
        min_area_px=max(12.0, selected_area * 0.08),
        max_area_px=max(50.0, selected_area * 4.0),
        sample_hsv_median=median,
        required=required,
    )


def frame_brightness(
    frame_bgr: np.ndarray, roi_fraction: list[float] | tuple[float, ...] | None = None
) -> float:
    """Return median value-channel brightness, optionally from a static ROI.

    A fixed background ROI prevents an operator entering the frame from being
    mistaken for a global lighting/exposure change during supervised trials.
    Fractions are ``[left, top, right, bottom]`` in normalized image space.
    """
    sample = frame_bgr
    if roi_fraction is not None:
        if len(roi_fraction) != 4:
            raise ValueError("brightness_roi_fraction must contain four values")
        left, top, right, bottom = (float(value) for value in roi_fraction)
        if not (0.0 <= left < right <= 1.0 and 0.0 <= top < bottom <= 1.0):
            raise ValueError(
                "brightness_roi_fraction must satisfy 0 <= left < right <= 1 "
                "and 0 <= top < bottom <= 1"
            )
        height, width = frame_bgr.shape[:2]
        x0 = min(width - 1, int(round(left * width)))
        y0 = min(height - 1, int(round(top * height)))
        x1 = max(x0 + 1, min(width, int(round(right * width))))
        y1 = max(y0 + 1, min(height, int(round(bottom * height))))
        sample = frame_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
    # Downsample before the median to bound CPU use on a Raspberry Pi.
    return float(np.median(hsv[::8, ::8, 2]))


@dataclass(slots=True)
class LightingAssessment:
    acceptable: bool
    recalibration_required: bool
    value_shift: int
    delta: float
    reason: str


class LightingMonitor:
    def __init__(self, baseline: float, config: dict[str, Any]) -> None:
        self.baseline = float(baseline)
        self.warning_delta = float(config.get("brightness_warning_delta", 28))
        self.stop_delta = float(config.get("brightness_stop_delta", 55))
        self.max_adjustment = int(config.get("bounded_value_adjustment", 18))

    def assess(self, current: float) -> LightingAssessment:
        delta = float(current) - self.baseline
        magnitude = abs(delta)
        if magnitude >= self.stop_delta:
            return LightingAssessment(
                False,
                True,
                0,
                delta,
                "brightness moved beyond the calibrated environment",
            )
        shift = int(max(-self.max_adjustment, min(self.max_adjustment, round(delta))))
        if magnitude >= self.warning_delta:
            return LightingAssessment(
                True,
                False,
                shift,
                delta,
                "bounded value-threshold compensation active",
            )
        return LightingAssessment(True, False, shift, delta, "lighting within profile")


def new_profile(
    name: str,
    markers: list[MarkerColorProfile],
    coordinates: CoordinateCalibration,
    baseline_brightness: float,
    notes: str = "",
) -> CalibrationProfile:
    return CalibrationProfile(
        name=name,
        markers=markers,
        coordinates=coordinates,
        baseline_brightness=baseline_brightness,
        created_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        notes=notes,
    )

