"""HSV marker segmentation with ambiguity and lighting-failure detection."""

from __future__ import annotations

import math
import time
from collections import defaultdict
from typing import Any

import cv2
import numpy as np

from .calibration import CalibrationProfile, LightingMonitor, frame_brightness
from .models import MarkerDetection, MarkerRole, TrackingFrame, TrackingStatus


_STATUS_COLOR = {
    TrackingStatus.OK: (30, 220, 70),
    TrackingStatus.LOW_CONFIDENCE: (0, 200, 255),
    TrackingStatus.PARTIAL_LOSS: (0, 140, 255),
    TrackingStatus.LOST: (0, 0, 255),
    TrackingStatus.AMBIGUOUS: (210, 50, 220),
    TrackingStatus.RECALIBRATION_REQUIRED: (0, 0, 180),
}


class ColorMarkerTracker:
    def __init__(self, profile: CalibrationProfile, config: dict[str, Any]) -> None:
        self.profile = profile
        self.config = config
        self.kernel_size = max(1, int(config.get("morphology_kernel", 5)))
        if self.kernel_size % 2 == 0:
            self.kernel_size += 1
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size)
        )
        self.min_confidence = float(config.get("min_confidence", 0.55))
        self.ambiguous_ratio = float(config.get("ambiguous_area_ratio", 0.72))
        self.brightness_roi = config.get("brightness_roi_fraction")
        self.lighting = LightingMonitor(profile.baseline_brightness, config)
        self._last_centers: dict[str, tuple[float, float]] = {}

    @staticmethod
    def _shifted_bounds(
        lower: tuple[int, int, int], upper: tuple[int, int, int], value_shift: int
    ) -> tuple[np.ndarray, np.ndarray]:
        # A brighter environment raises both observed V values and thresholds.
        low = (lower[0], lower[1], max(0, min(255, lower[2] + value_shift)))
        high = (upper[0], upper[1], max(0, min(255, upper[2] + value_shift)))
        return np.asarray(low, dtype=np.uint8), np.asarray(high, dtype=np.uint8)

    def _mask(self, hsv: np.ndarray, marker: Any, value_shift: int) -> np.ndarray:
        combined = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hsv_range in marker.hsv_ranges:
            low, high = self._shifted_bounds(
                hsv_range.lower, hsv_range.upper, value_shift
            )
            combined = cv2.bitwise_or(combined, cv2.inRange(hsv, low, high))
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(combined, cv2.MORPH_CLOSE, self.kernel)

    def _detect_one(
        self,
        hsv: np.ndarray,
        marker: Any,
        value_shift: int,
        timestamp_s: float,
    ) -> MarkerDetection:
        mask = self._mask(hsv, marker, value_shift)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[tuple[float, np.ndarray, float]] = []
        last = self._last_centers.get(marker.name)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not marker.min_area_px <= area <= marker.max_area_px:
                continue
            hull_area = max(area, float(cv2.contourArea(cv2.convexHull(contour))))
            solidity = area / hull_area if hull_area else 0.0
            x, y, width, height = cv2.boundingRect(contour)
            extent = area / max(1.0, float(width * height))
            area_center = math.sqrt(marker.min_area_px * marker.max_area_px)
            area_score = math.exp(-abs(math.log(max(area, 1.0) / area_center)))
            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-9:
                continue
            center = (
                moments["m10"] / moments["m00"],
                moments["m01"] / moments["m00"],
            )
            continuity = 0.5
            if last is not None:
                distance = math.hypot(center[0] - last[0], center[1] - last[1])
                continuity = math.exp(-distance / 120.0)
            score = 0.38 * solidity + 0.22 * extent + 0.22 * area_score + 0.18 * continuity
            candidates.append((area, contour, max(0.0, min(1.0, score))))
        candidates.sort(key=lambda item: (item[2], item[0]), reverse=True)
        if not candidates:
            return MarkerDetection(
                marker.name,
                marker.role,
                None,
                0.0,
                0.0,
                TrackingStatus.LOST,
                timestamp_s,
                0,
            )
        best_area, best_contour, confidence = candidates[0]
        if len(candidates) > 1:
            second_area, _, second_confidence = candidates[1]
            if (
                second_area / max(best_area, 1.0) >= self.ambiguous_ratio
                and second_confidence >= confidence * 0.85
            ):
                return MarkerDetection(
                    marker.name,
                    marker.role,
                    None,
                    best_area,
                    confidence,
                    TrackingStatus.AMBIGUOUS,
                    timestamp_s,
                    len(candidates),
                )
        moments = cv2.moments(best_contour)
        center = (
            float(moments["m10"] / moments["m00"]),
            float(moments["m01"] / moments["m00"]),
        )
        status = (
            TrackingStatus.OK
            if confidence >= self.min_confidence
            else TrackingStatus.LOW_CONFIDENCE
        )
        if status is TrackingStatus.OK:
            self._last_centers[marker.name] = center
        return MarkerDetection(
            marker.name,
            marker.role,
            center,
            best_area,
            confidence,
            status,
            timestamp_s,
            len(candidates),
        )

    def process(
        self, frame_bgr: np.ndarray, timestamp_s: float, sequence: int = 0
    ) -> TrackingFrame:
        started = time.perf_counter()
        brightness = frame_brightness(frame_bgr, self.brightness_roi)
        lighting = self.lighting.assess(brightness)
        if lighting.recalibration_required:
            detections = {
                marker.name: MarkerDetection(
                    marker.name,
                    marker.role,
                    None,
                    0.0,
                    0.0,
                    TrackingStatus.RECALIBRATION_REQUIRED,
                    timestamp_s,
                )
                for marker in self.profile.markers
            }
            return TrackingFrame(
                timestamp_s,
                detections,
                TrackingStatus.RECALIBRATION_REQUIRED,
                brightness,
                (time.perf_counter() - started) * 1000.0,
                sequence,
                lighting.reason,
            )
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        detections = {
            marker.name: self._detect_one(
                hsv, marker, lighting.value_shift, timestamp_s
            )
            for marker in self.profile.markers
        }
        required = [marker for marker in self.profile.markers if marker.required]
        required_statuses = [detections[marker.name].status for marker in required]
        valid_count = sum(detections[marker.name].valid for marker in required)
        if any(status is TrackingStatus.AMBIGUOUS for status in required_statuses):
            status = TrackingStatus.AMBIGUOUS
            reason = "more than one plausible object matches a required marker"
        elif valid_count == len(required) and required:
            status = (
                TrackingStatus.OK
                if all(s is TrackingStatus.OK for s in required_statuses)
                else TrackingStatus.LOW_CONFIDENCE
            )
            reason = lighting.reason
        elif valid_count > 0:
            status = TrackingStatus.PARTIAL_LOSS
            reason = "one or more required markers are missing"
        else:
            status = TrackingStatus.LOST
            reason = "all required markers are missing"
        return TrackingFrame(
            timestamp_s,
            detections,
            status,
            brightness,
            (time.perf_counter() - started) * 1000.0,
            sequence,
            reason,
        )

    def annotate(self, frame_bgr: np.ndarray, result: TrackingFrame) -> np.ndarray:
        output = frame_bgr.copy()
        for detection in result.detections.values():
            color = _STATUS_COLOR[detection.status]
            if detection.center_px is not None:
                center = tuple(int(round(value)) for value in detection.center_px)
                cv2.circle(output, center, 10, color, 2, cv2.LINE_AA)
                label = (
                    f"{detection.name}/{detection.role}: "
                    f"({center[0]},{center[1]}) {detection.confidence:.2f}"
                )
                cv2.putText(
                    output,
                    label,
                    (center[0] + 12, max(20, center[1] - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                    cv2.LINE_AA,
                )
        cv2.putText(
            output,
            f"tracking={result.status} brightness={result.mean_brightness:.0f} "
            f"processing={result.processing_ms:.1f}ms",
            (14, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            _STATUS_COLOR[result.status],
            2,
            cv2.LINE_AA,
        )
        return output

    def masks(self, frame_bgr: np.ndarray) -> dict[str, np.ndarray]:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        brightness = frame_brightness(frame_bgr, self.brightness_roi)
        assessment = self.lighting.assess(brightness)
        return {
            marker.name: self._mask(hsv, marker, assessment.value_shift)
            for marker in self.profile.markers
        }


def detections_by_role(result: TrackingFrame) -> dict[MarkerRole, list[MarkerDetection]]:
    grouped: dict[MarkerRole, list[MarkerDetection]] = defaultdict(list)
    for detection in result.detections.values():
        grouped[detection.role].append(detection)
    return grouped

