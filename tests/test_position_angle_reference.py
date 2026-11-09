import math
from dataclasses import replace

import pytest

from raspberry_pi.calibration import CoordinateCalibration, CalibrationProfile
from raspberry_pi.models import MarkerDetection, MarkerRole, TrackingFrame, TrackingStatus
from raspberry_pi.state_estimator import StateEstimator
from raspberry_pi.web_server import CoordinateRequest


def calibration(**options):
    return CoordinateCalibration(rail_left_px=0, rail_right_px=500,
                                 rail_length_mm=500, **options)


def tracking(coords, position, true_angle, timestamp):
    px = position if coords.camera_x_sign == 1 else 500-position
    projected = true_angle + coords.vertical_reference_rad(position)
    pivot = (px, 250.0)
    tip = (px + coords.camera_x_sign*150*math.sin(projected),
           250.0 - 150*math.cos(projected))
    detections = {
        name: MarkerDetection(name, role, point, 400.0, .95, TrackingStatus.OK, timestamp)
        for name, role, point in [("pivot", MarkerRole.PIVOT, pivot),
                                  ("tip", MarkerRole.PENDULUM_TIP, tip)]
    }
    return TrackingFrame(timestamp, detections, TrackingStatus.OK, 100, 1, 1)


@pytest.mark.parametrize("sign", [1, -1])
@pytest.mark.parametrize("angle", [0.0, .15, math.pi, -math.pi+.03])
def test_translated_fixed_pole_has_no_false_angle_or_angular_velocity(sign, angle):
    coords = calibration(camera_x_sign=sign, vertical_offset_rad=.012,
                         vertical_offset_slope_rad_per_mm=-.00008,
                         vertical_offset_reference_mm=250)
    estimator = StateEstimator(coords, {})
    for i, position in enumerate([210.0, 212.0, 214.0, 216.0]):
        state = estimator.update(tracking(coords, position, angle, 1+i/30))
        assert state.valid
        difference = math.atan2(math.sin(state.theta_rad-angle), math.cos(state.theta_rad-angle))
        assert difference == pytest.approx(0, abs=1e-12)
        assert state.omega_rad_s == pytest.approx(0, abs=1e-11)
        assert state.cart_position_mm == pytest.approx(position)


def test_disabled_correction_preserves_legacy_angle_and_rail_coordinates():
    coords = calibration(vertical_offset_rad=.02)
    event = tracking(coords, 320, .13, 1)
    state = StateEstimator(coords, {}).update(event)
    pivot, tip = event.detections['pivot'].center_px, event.detections['tip'].center_px
    assert state.theta_rad == math.atan2(tip[0]-pivot[0], pivot[1]-tip[1])-.02
    corrected = replace(coords, vertical_offset_slope_rad_per_mm=-.00008,
                        vertical_offset_reference_mm=250)
    for point in [(0, 250), (120, 200), (500, 250)]:
        assert corrected.pixel_to_rail_mm(point) == coords.pixel_to_rail_mm(point)


@pytest.mark.parametrize("options", [
    {'vertical_offset_slope_rad_per_mm': float('nan')},
    {'vertical_offset_slope_rad_per_mm': float('inf')},
    {'vertical_offset_reference_mm': float('nan')},
    {'vertical_offset_reference_mm': float('inf')},
    {'vertical_offset_rad': float('nan')},
    {'vertical_offset_slope_rad_per_mm': .00008, 'vertical_offset_reference_mm': -1},
    {'vertical_offset_slope_rad_per_mm': .00008, 'vertical_offset_reference_mm': 501},
    {'vertical_offset_slope_rad_per_mm': .001, 'vertical_offset_reference_mm': 250},
])
def test_invalid_reference_is_rejected_by_coordinate_calibration(options):
    assert not calibration(**options).valid


def test_extrapolated_angle_outside_bound_is_invalid_not_stale_state():
    coords = calibration(vertical_offset_slope_rad_per_mm=.0001,
                         vertical_offset_reference_mm=250)
    estimator = StateEstimator(coords, {})
    assert estimator.update(tracking(coords, 250, 0, 1)).valid
    event = tracking(calibration(), 1000, 0, 1.033)
    state = estimator.update(event)
    assert not state.valid and state.theta_rad is None and state.omega_rad_s is None
    assert 'angle reference' in state.reason


def test_profile_and_api_roundtrip_preserve_optional_reference():
    coords = calibration(vertical_offset_rad=.02,
                         vertical_offset_slope_rad_per_mm=-.00008,
                         vertical_offset_reference_mm=250)
    profile = CalibrationProfile('test', [], coords, 100, 'test')
    payload = profile.to_dict()
    restored = CalibrationProfile.from_dict(payload)
    assert restored.coordinates == coords
    assert CoordinateRequest(**payload['coordinates']).model_dump()['vertical_offset_slope_rad_per_mm'] == -.00008
    del payload['coordinates']['vertical_offset_slope_rad_per_mm']
    del payload['coordinates']['vertical_offset_reference_mm']
    legacy = CalibrationProfile.from_dict(payload)
    assert legacy.coordinates.vertical_offset_slope_rad_per_mm == 0
