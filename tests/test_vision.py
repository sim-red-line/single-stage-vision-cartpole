import math

import cv2
import numpy as np

from raspberry_pi.calibration import (
    CoordinateCalibration,
    ProfileStore,
    calibrate_marker_from_roi,
    frame_brightness,
    new_profile,
)
import pytest
from raspberry_pi.color_tracker import ColorMarkerTracker
from raspberry_pi.models import MarkerRole, TrackingStatus
from raspberry_pi.state_estimator import StateEstimator


TRACKING_CONFIG = {
    "min_confidence": 0.5,
    "morphology_kernel": 3,
    "ambiguous_area_ratio": 0.72,
    "brightness_warning_delta": 28,
    "brightness_stop_delta": 55,
    "bounded_value_adjustment": 18,
}


def make_frame(tip=(320, 150), pivot=(320, 300)):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(frame, tip, 16, (0, 0, 255), -1)  # red tip
    cv2.circle(frame, pivot, 16, (255, 0, 0), -1)  # blue pivot
    return frame


def build_profile(frame):
    tip = calibrate_marker_from_roi(
        frame, (306, 136, 28, 28), name="tip", role=MarkerRole.PENDULUM_TIP
    )
    pivot = calibrate_marker_from_roi(
        frame, (306, 286, 28, 28), name="pivot", role=MarkerRole.PIVOT
    )
    coordinates = CoordinateCalibration(
        rail_left_px=0,
        rail_right_px=640,
        rail_length_mm=500,
        camera_x_sign=1,
    )
    return new_profile("test", [tip, pivot], coordinates, frame_brightness(frame))


def test_calibrated_colors_track_and_estimate_vertical_angle():
    first_frame = make_frame()
    profile = build_profile(first_frame)
    tracker = ColorMarkerTracker(profile, TRACKING_CONFIG)
    estimator = StateEstimator(
        profile.coordinates, {"derivative_cutoff_hz": 12.0, "maximum_dt_s": 0.12}
    )
    first_tracking = tracker.process(first_frame, timestamp_s=1.0, sequence=1)
    assert first_tracking.status is TrackingStatus.OK
    first_state = estimator.update(first_tracking)
    assert first_state.valid
    assert first_state.theta_rad == 0.0
    assert first_state.cart_position_mm == 250.0

    shifted = make_frame(tip=(335, 151))
    second_tracking = tracker.process(shifted, timestamp_s=1.02, sequence=2)
    second_state = estimator.update(second_tracking)
    assert second_state.valid
    assert second_state.theta_rad > 0.05
    assert second_state.omega_rad_s > 0.0


def test_ambiguous_same_color_objects_are_not_reused_as_measurement():
    frame = make_frame()
    profile = build_profile(frame)
    tracker = ColorMarkerTracker(profile, TRACKING_CONFIG)
    ambiguous = frame.copy()
    cv2.circle(ambiguous, (400, 150), 16, (0, 0, 255), -1)
    result = tracker.process(ambiguous, timestamp_s=2.0)
    assert result.detections["tip"].status is TrackingStatus.AMBIGUOUS
    assert result.detections["tip"].center_px is None
    assert result.status is TrackingStatus.AMBIGUOUS


def test_large_brightness_change_requires_recalibration():
    frame = make_frame()
    profile = build_profile(frame)
    tracker = ColorMarkerTracker(profile, TRACKING_CONFIG)
    bright = np.full_like(frame, 255)
    result = tracker.process(bright, timestamp_s=3.0)
    assert result.status is TrackingStatus.RECALIBRATION_REQUIRED


def test_brightness_roi_ignores_operator_occluding_other_frame_area():
    frame = np.full((100, 100, 3), 160, dtype=np.uint8)
    roi = [0.7, 0.1, 0.95, 0.35]
    baseline = frame_brightness(frame, roi)
    occluded = frame.copy()
    occluded[:, :70] = 10
    assert frame_brightness(occluded, roi) == baseline


def test_profile_store_rejects_path_traversal(tmp_path):
    store = ProfileStore(tmp_path)
    with pytest.raises(ValueError):
        store.load("../outside")

