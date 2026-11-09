from __future__ import annotations

import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np

from raspberry_pi.calibration import CalibrationProfile
from raspberry_pi.color_tracker import ColorMarkerTracker


ROOT = Path(__file__).resolve().parents[1]
PROFILES = ["upright-lighting-20261031", "fixed-exposure-20261031"]
VIDEOS = ["20261030T154400Z-1a0b40c7", "20261030T155423Z-29d98ef0"]
CONFIG = {
    "morphology_kernel": 3,
    "min_confidence": 0.55,
    "ambiguous_area_ratio": 0.72,
    "brightness_warning_delta": 255,
    "brightness_stop_delta": 256,
    "bounded_value_adjustment": 0,
}


def evaluate(profile_name: str, experiment_id: str) -> dict:
    payload = json.loads(
        (ROOT / "calibration" / "profiles" / f"{profile_name}.json").read_text(
            encoding="utf-8"
        )
    )
    # Lighting is deliberately bypassed here; this script compares only the
    # geometry produced by each HSV mask on recorded raw frames.
    payload = copy.deepcopy(payload)
    payload["baseline_brightness"] = 128.0
    tracker = ColorMarkerTracker(CalibrationProfile.from_dict(payload), CONFIG)
    capture = cv2.VideoCapture(
        str(ROOT / "experiments" / "videos" / f"{experiment_id}.avi")
    )
    raw_angles = []
    pivot_x = []
    tip_x = []
    frames = 0
    valid = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        result = tracker.process(frame, frames / 30.0, frames)
        pivot = result.detections["pivot"]
        tip = result.detections["tip"]
        if pivot.valid and tip.valid and pivot.center_px and tip.center_px:
            dx = tip.center_px[0] - pivot.center_px[0]
            dy_up = pivot.center_px[1] - tip.center_px[1]
            raw_angles.append(math.atan2(dx, dy_up))
            pivot_x.append(pivot.center_px[0])
            tip_x.append(tip.center_px[0])
            valid += 1
        frames += 1
    capture.release()
    values = np.asarray(raw_angles, dtype=float)
    return {
        "frames": frames,
        "valid": valid,
        "valid_rate": valid / max(1, frames),
        "raw_angle_first_rad": float(values[0]) if valid else None,
        "raw_angle_median_rad": float(np.median(values)) if valid else None,
        "raw_angle_mean_rad": float(np.mean(values)) if valid else None,
        "raw_angle_min_rad": float(np.min(values)) if valid else None,
        "raw_angle_max_rad": float(np.max(values)) if valid else None,
        "pivot_x_first": float(pivot_x[0]) if valid else None,
        "tip_x_first": float(tip_x[0]) if valid else None,
    }


report = {
    profile: {video: evaluate(profile, video) for video in VIDEOS}
    for profile in PROFILES
}
print(json.dumps(report, indent=2))
