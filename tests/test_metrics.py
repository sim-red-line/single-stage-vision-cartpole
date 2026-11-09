import math

from raspberry_pi.metrics import calculate_metrics


def test_metrics_require_measured_continuous_tracking_for_success():
    samples = []
    for index in range(61):
        t = index * 0.1
        samples.append(
            {
                "elapsed_s": t,
                "tracking_valid": not (20 <= index <= 22),
                "theta_rad": math.radians(2.0),
                "omega_rad_s": 0.1,
                "cart_position_mm": 250.0,
                "cart_velocity_mm_s": 0.0,
                "limited_velocity_mm_s": 0.0,
                "output_saturated": False,
                "processing_ms": 5.0,
                "fps": 10.0,
            }
        )
    metrics = calculate_metrics(samples)
    assert metrics.continuous_stable_time_s < 5.0
    assert not metrics.stabilization_success
    assert metrics.tracking_loss_count == 1

