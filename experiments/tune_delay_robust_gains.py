from __future__ import annotations

import json

from .simulate_delay_sweep import simulate


KP_VALUES = [11000.0, 12000.0, 13000.0, 14000.0, 15000.0, 17000.0, 19000.0, 22000.0]
KD_VALUES = [800.0, 1100.0, 1400.0, 1700.0, 2000.0, 2400.0, 2800.0, 3300.0]
SCENARIOS = [
    # Effective delay includes exposure, capture buffering, processing, and the
    # latest-state age observed by the 100 Hz controller.
    {"delay_s": 0.08, "initial_theta": 0.004, "initial_omega": 0.0, "initial_position": 15.0},
    {"delay_s": 0.10, "initial_theta": 0.004, "initial_omega": -0.02, "initial_position": 15.0},
    {"delay_s": 0.12, "initial_theta": 0.008, "initial_omega": 0.0, "initial_position": 0.0},
    {"delay_s": 0.10, "initial_theta": -0.008, "initial_omega": 0.05, "initial_position": -20.0},
]


ranked = []
for kp in KP_VALUES:
    for kd in KD_VALUES:
        runs = [simulate(kp_angle=kp, kd_angle=kd, **scenario) for scenario in SCENARIOS]
        failures = sum(run["failed_at_s"] is not None for run in runs)
        worst_angle = max(run["max_abs_theta_rad"] for run in runs)
        worst_velocity = max(run["max_abs_velocity_mm_s"] for run in runs)
        worst_saturation = max(run["saturation_fraction"] for run in runs)
        worst_position = max(abs(run["final_position_mm"]) for run in runs)
        ranked.append(
            {
                "kp_angle": kp,
                "kd_angle": kd,
                "failures": failures,
                "worst_angle_rad": worst_angle,
                "worst_velocity_mm_s": worst_velocity,
                "worst_saturation_fraction": worst_saturation,
                "worst_final_position_mm": worst_position,
            }
        )

ranked.sort(
    key=lambda item: (
        item["failures"],
        item["worst_angle_rad"],
        item["worst_saturation_fraction"],
        item["worst_final_position_mm"],
    )
)
print(json.dumps(ranked[:20], indent=2))
