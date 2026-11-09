"""Validate angular forecasts against later recorded frames; never opens hardware."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from raspberry_pi.config import load_config
from raspberry_pi.models import CartPoleState, PIDGains, TrackingStatus
from raspberry_pi.pid_controller import HybridSwingUpController


def replay(csv_path: Path, config: dict, latency_s: float, rate_lag_s: float) -> dict:
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    now = np.array([float(row["timestamp_monotonic_s"]) for row in rows])
    observed_at = now - np.array([float(row["control_latency_ms"]) / 1000 for row in rows])
    angle = np.unwrap([float(row["theta_rad"]) for row in rows])
    config = {**config, "swing_up": {**config["swing_up"],
              "prediction_camera_latency_s": 0.0,
              "balance_prediction_camera_latency_s": latency_s,
              "balance_rate_lag_s": rate_lag_s, "max_duration_s": 3600.0}}
    controller = HybridSwingUpController(config, PIDGains.from_mapping(config["gains"]))
    raw_errors, predicted_errors = [], []
    for index, row in enumerate(rows):
        sample = CartPoleState(
            float(observed_at[index]), float(row["theta_rad"]), float(row["omega_rad_s"]),
            float(row["cart_position_mm"]), float(row["cart_velocity_mm_s"]),
            True, TrackingStatus.OK,
        )
        controller.update(sample, float(now[index]))
        # Raw future-frame time includes the identified intrinsic camera delay.
        truth_at = now[index] + latency_s
        if (float(row["angle_p"]) or float(row["angle_d"])) and truth_at <= observed_at[-1]:
            predicted = controller._balance_state(sample, float(now[index]))
            truth = float(np.interp(truth_at, observed_at, angle))
            wrap = lambda value: math.atan2(math.sin(value), math.cos(value))
            raw_errors.append(wrap(sample.theta_rad - truth))
            predicted_errors.append(wrap(predicted.theta_rad - truth))
        # Only recorded admitted targets drive the actuator model in this replay.
        controller.accept_applied_velocity(float(row["limited_velocity_mm_s"]))
    metrics = lambda values: {
        "rms_deg": float(np.sqrt(np.mean(np.square(values))) * 180 / math.pi),
        "p95_abs_deg": float(np.percentile(np.abs(values), 95) * 180 / math.pi),
    } if values else None
    return {"experiment_id": csv_path.stem, "balance_samples": len(raw_errors),
            "camera_latency_s": latency_s, "rate_lag_s": rate_lag_s,
            "raw": metrics(raw_errors), "forecast": metrics(predicted_errors)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, nargs="+")
    parser.add_argument("--config", default="config.local.yaml")
    parser.add_argument("--camera-latency", type=float, default=0.026)
    parser.add_argument("--rate-lag", type=float, default=0.03)
    args = parser.parse_args()
    config = load_config(args.config)["controller"]
    results = [replay(path, config, args.camera_latency, args.rate_lag) for path in args.csv]
    print(json.dumps({"offline_only": True, "results": results}, indent=2))


if __name__ == "__main__":
    main()
