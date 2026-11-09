import pytest

from raspberry_pi.models import CartPoleState, TrackingStatus
from raspberry_pi.safety import SafetyManager, SafetyState


CONFIG = {
    "rail_min_mm": 10.0,
    "rail_max_mm": 490.0,
    "braking_acceleration_mm_s2": 2000.0,
    "braking_margin_mm": 25.0,
    "command_latency_s": 0.08,
    "tracking_grace_s": 0.12,
}


def valid_state(timestamp=1.0, x=250.0, velocity=0.0):
    return CartPoleState(
        timestamp,
        0.01,
        0.0,
        x,
        velocity,
        True,
        TrackingStatus.OK,
    )


def running_manager():
    safety = SafetyManager(CONFIG)
    safety.update_prerequisites(
        connected=True, homed=True, calibrated=True, tracking_ok=True
    )
    assert safety.state is SafetyState.IDLE
    assert safety.arm()
    assert safety.start()
    return safety


def test_prerequisites_gate_arming():
    safety = SafetyManager(CONFIG)
    safety.update_prerequisites(connected=True, homed=False, calibrated=False)
    assert safety.state is SafetyState.UNHOMED
    assert not safety.arm()
    safety.update_prerequisites(homed=True, calibrated=False)
    assert safety.state is SafetyState.CALIBRATION_REQUIRED


def test_unclean_pi_power_blocks_arming_and_fault_clear():
    safety = SafetyManager(CONFIG)
    safety.update_prerequisites(
        connected=True,
        homed=True,
        calibrated=True,
        tracking_ok=True,
        power_ok=False,
        power_reason="undervoltage occurred since boot",
    )
    assert safety.state is SafetyState.FAULT
    assert not safety.arm()
    assert not safety.clear_fault()
    assert "undervoltage" in safety.reason


def test_dynamic_braking_prevents_motion_toward_near_edge():
    safety = running_manager()
    decision = safety.evaluate_velocity(
        200.0, valid_state(timestamp=1.0, x=472.0, velocity=150.0), now_s=1.02
    )
    assert decision.allowed_velocity_mm_s <= 0.0
    assert not decision.stop_required


def test_stale_camera_state_latches_fault():
    safety = running_manager()
    decision = safety.evaluate_velocity(10.0, valid_state(timestamp=1.0), now_s=1.5)
    assert decision.stop_required
    assert safety.state is SafetyState.FAULT
    assert "stale" in safety.reason


def test_estop_cannot_clear_with_active_limit():
    safety = running_manager()
    safety.estop("test")
    assert safety.state is SafetyState.ESTOP
    assert not safety.clear_estop(physical_limits_clear=False)
    assert safety.state is SafetyState.ESTOP
    assert safety.clear_estop(physical_limits_clear=True)
    assert safety.state is SafetyState.UNHOMED


def test_recorded_swing_drift_brakes_before_centered_excursion_fault():
    safety = running_manager()
    decision = safety.evaluate_velocity(
        130.37, valid_state(x=365.249, velocity=148.093), now_s=1.02,
        motion_bounds_mm=(136.369, 376.369),
    )
    assert decision.allowed_velocity_mm_s == 0.0
    assert not decision.stop_required
    # Returning inward remains permitted inside the original physical rail.
    inward = safety.evaluate_velocity(
        -40.0, valid_state(x=365.249, velocity=148.093), now_s=1.02,
        motion_bounds_mm=(136.369, 376.369),
    )
    assert inward.allowed_velocity_mm_s == -40.0


def test_centered_corridor_does_not_change_commands_with_stopping_room():
    safety = running_manager()
    decision = safety.evaluate_velocity(
        160.0, valid_state(x=256.369, velocity=160.0), now_s=1.02,
        motion_bounds_mm=(136.369, 376.369),
    )
    assert decision.allowed_velocity_mm_s == 160.0


def test_motion_corridor_cannot_relax_original_rail_limits():
    safety = running_manager()
    decision = safety.evaluate_velocity(
        20.0, valid_state(x=495.0), now_s=1.02, motion_bounds_mm=(-1000.0, 1000.0),
    )
    assert decision.stop_required
    assert safety.state is SafetyState.FAULT


@pytest.mark.parametrize("bounds", [(float("nan"), 400.0), (10.0, float("inf")), (400.0, 10.0)])
def test_invalid_motion_corridor_latches_fault(bounds):
    safety = running_manager()
    decision = safety.evaluate_velocity(20.0, valid_state(), now_s=1.02, motion_bounds_mm=bounds)
    assert decision.stop_required
    assert safety.state is SafetyState.FAULT

