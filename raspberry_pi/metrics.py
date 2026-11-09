"""Evidence-based experiment metrics; no synthetic success generation."""

from __future__ import annotations

import math
import statistics
from dataclasses import asdict, dataclass, field
from typing import Any, Iterable


def _finite(values: Iterable[Any]) -> list[float]:
    result: list[float] = []
    for value in values:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(numeric):
            result.append(numeric)
    return result


def _rms(values: list[float]) -> float | None:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


@dataclass(slots=True)
class ExperimentMetrics:
    duration_s: float
    sample_count: int
    continuous_stable_time_s: float
    rms_angle_rad: float | None
    mean_abs_angle_rad: float | None
    max_abs_angle_rad: float | None
    angular_velocity_std_rad_s: float | None
    stabilization_success: bool
    mean_cart_deviation_mm: float | None
    max_cart_deviation_mm: float | None
    max_cart_speed_mm_s: float | None
    velocity_saturation_ratio: float
    direction_reversals: int
    max_command_acceleration_mm_s2: float | None
    average_fps: float | None
    average_processing_ms: float | None
    average_control_latency_ms: float | None
    average_measurement_interval_ms: float | None
    marker_detection_rate: float
    tracking_loss_count: int
    output_saturation_ratio: float
    output_saturation_time_s: float
    failure_type: str | None
    observations: list[dict[str, Any]] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def calculate_metrics(
    samples: list[dict[str, Any]],
    *,
    cart_center_mm: float = 250.0,
    stable_angle_rad: float = math.radians(5.0),
    success_time_s: float = 5.0,
    velocity_limit_mm_s: float = 600.0,
) -> ExperimentMetrics:
    if not samples:
        return ExperimentMetrics(
            duration_s=0.0,
            sample_count=0,
            continuous_stable_time_s=0.0,
            rms_angle_rad=None,
            mean_abs_angle_rad=None,
            max_abs_angle_rad=None,
            angular_velocity_std_rad_s=None,
            stabilization_success=False,
            mean_cart_deviation_mm=None,
            max_cart_deviation_mm=None,
            max_cart_speed_mm_s=None,
            velocity_saturation_ratio=0.0,
            direction_reversals=0,
            max_command_acceleration_mm_s2=None,
            average_fps=None,
            average_processing_ms=None,
            average_control_latency_ms=None,
            average_measurement_interval_ms=None,
            marker_detection_rate=0.0,
            tracking_loss_count=0,
            output_saturation_ratio=0.0,
            output_saturation_time_s=0.0,
            failure_type="no_samples",
            observations=[{"kind": "insufficient_data", "evidence": "experiment has no samples"}],
        )
    ordered = sorted(samples, key=lambda item: float(item.get("elapsed_s", 0.0)))
    times = _finite(item.get("elapsed_s") for item in ordered)
    duration = max(times) - min(times) if len(times) >= 2 else 0.0
    valid = [item for item in ordered if bool(item.get("tracking_valid", False))]
    angles = _finite(item.get("theta_rad") for item in valid)
    omegas = _finite(item.get("omega_rad_s") for item in valid)
    positions = _finite(item.get("cart_position_mm") for item in valid)
    velocities = _finite(item.get("cart_velocity_mm_s") for item in valid)
    commands = _finite(item.get("limited_velocity_mm_s") for item in ordered)

    longest_stable = 0.0
    stable_start: float | None = None
    previous_time: float | None = None
    for item in ordered:
        timestamp = float(item.get("elapsed_s", 0.0))
        angle = item.get("theta_rad")
        is_stable = bool(item.get("tracking_valid", False)) and angle is not None and abs(float(angle)) <= stable_angle_rad
        # A data gap breaks continuity; it is not silently bridged.
        if previous_time is not None and timestamp - previous_time > 0.25:
            stable_start = None
        if is_stable:
            if stable_start is None:
                stable_start = timestamp
            longest_stable = max(longest_stable, timestamp - stable_start)
        else:
            stable_start = None
        previous_time = timestamp

    tracking_flags = [bool(item.get("tracking_valid", False)) for item in ordered]
    losses = sum(
        1
        for index, good in enumerate(tracking_flags)
        if not good and (index == 0 or tracking_flags[index - 1])
    )
    reversals = 0
    last_sign = 0
    for command in commands:
        sign = 1 if command > 1.0 else -1 if command < -1.0 else 0
        if sign and last_sign and sign != last_sign:
            reversals += 1
        if sign:
            last_sign = sign
    accelerations: list[float] = []
    for previous, current in zip(ordered, ordered[1:]):
        dt = float(current.get("elapsed_s", 0.0)) - float(previous.get("elapsed_s", 0.0))
        if dt > 0:
            before = previous.get("limited_velocity_mm_s")
            after = current.get("limited_velocity_mm_s")
            if before is not None and after is not None:
                accelerations.append(abs(float(after) - float(before)) / dt)
    intervals = [
        (b - a) * 1000.0 for a, b in zip(times, times[1:]) if 0.0 < b - a < 1.0
    ]
    processing = _finite(item.get("processing_ms") for item in ordered)
    control_latencies = _finite(item.get("control_latency_ms") for item in ordered)
    fps_values = _finite(item.get("fps") for item in ordered)
    saturated_count = sum(bool(item.get("output_saturated", False)) for item in ordered)
    saturation_time = 0.0
    for previous, current in zip(ordered, ordered[1:]):
        if bool(previous.get("output_saturated", False)):
            saturation_time += max(
                0.0,
                min(
                    0.25,
                    float(current.get("elapsed_s", 0.0))
                    - float(previous.get("elapsed_s", 0.0)),
                ),
            )
    velocity_sat_count = sum(abs(value) >= velocity_limit_mm_s * 0.995 for value in commands)
    deviations = [abs(value - cart_center_mm) for value in positions]

    observations: list[dict[str, Any]] = []
    failure_type: str | None = None
    detection_rate = len(valid) / len(ordered)
    rms_angle = _rms(angles)
    tracking_caused_stop = bool(tracking_flags and not tracking_flags[-1] and losses > 0)
    if detection_rate < 0.9 or tracking_caused_stop:
        failure_type = "tracking_loss"
        observations.append(
            {
                "kind": "tracking_loss",
                "evidence": f"valid marker state in {detection_rate:.1%} of samples; {losses} loss episodes",
            }
        )
    if commands and saturated_count / len(ordered) > 0.2:
        observations.append(
            {
                "kind": "output_saturation",
                "evidence": f"controller output saturated for {saturated_count / len(ordered):.1%} of samples",
            }
        )
    if angles and reversals >= max(4, int(duration * 1.5)):
        observations.append(
            {
                "kind": "possible_oscillation",
                "evidence": f"{reversals} command direction reversals over {duration:.2f}s",
            }
        )
    if deviations and max(deviations) > 180.0:
        failure_type = failure_type or "cart_position_limit"
        observations.append(
            {
                "kind": "cart_drift",
                "evidence": f"maximum center deviation {max(deviations):.1f}mm",
            }
        )
    success = longest_stable >= success_time_s and detection_rate >= 0.9
    if not success and failure_type is None:
        failure_type = "insufficient_stable_time"
    return ExperimentMetrics(
        duration_s=duration,
        sample_count=len(ordered),
        continuous_stable_time_s=longest_stable,
        rms_angle_rad=rms_angle,
        mean_abs_angle_rad=statistics.fmean(abs(value) for value in angles) if angles else None,
        max_abs_angle_rad=max((abs(value) for value in angles), default=None),
        angular_velocity_std_rad_s=statistics.pstdev(omegas) if len(omegas) >= 2 else None,
        stabilization_success=success,
        mean_cart_deviation_mm=statistics.fmean(deviations) if deviations else None,
        max_cart_deviation_mm=max(deviations, default=None),
        max_cart_speed_mm_s=max((abs(value) for value in velocities), default=None),
        velocity_saturation_ratio=velocity_sat_count / max(1, len(commands)),
        direction_reversals=reversals,
        max_command_acceleration_mm_s2=max(accelerations, default=None),
        average_fps=statistics.fmean(fps_values) if fps_values else None,
        average_processing_ms=statistics.fmean(processing) if processing else None,
        average_control_latency_ms=(
            statistics.fmean(control_latencies) if control_latencies else None
        ),
        average_measurement_interval_ms=statistics.fmean(intervals) if intervals else None,
        marker_detection_rate=detection_rate,
        tracking_loss_count=losses,
        output_saturation_ratio=saturated_count / len(ordered),
        output_saturation_time_s=saturation_time,
        failure_type=failure_type,
        observations=observations,
    )


def representative_series(samples: list[dict[str, Any]], points: int = 80) -> list[dict[str, Any]]:
    if len(samples) <= points:
        selected = samples
    else:
        step = (len(samples) - 1) / (points - 1)
        selected = [samples[round(index * step)] for index in range(points)]
    keys = (
        "elapsed_s",
        "theta_rad",
        "omega_rad_s",
        "cart_position_mm",
        "cart_velocity_mm_s",
        "limited_velocity_mm_s",
        "tracking_valid",
    )
    return [{key: item.get(key) for key in keys} for item in selected]

