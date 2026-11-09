from __future__ import annotations

import copy
import json
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from raspberry_pi.calibration import CalibrationProfile
from raspberry_pi.color_tracker import ColorMarkerTracker


ROOT = Path(__file__).resolve().parents[1]
source = json.loads(
    (ROOT / "calibration/profiles/color-sweep-20261030.json").read_text(
        encoding="utf-8"
    )
)
candidate = copy.deepcopy(source)
candidate["name"] = "upright-lighting-20261031"
candidate["baseline_brightness"] = 130.0
candidate["notes"] = (
    "HSV and morphology update derived from two supervised upright frames, "
    "including the dark-glove condition that changed camera auto exposure."
)
for marker in candidate["markers"]:
    if marker["name"] == "pivot":
        marker["hsv_ranges"] = [
            {"lower": [96, 80, 20], "upper": [112, 255, 255]},
        ]
        marker["min_area_px"] = 40.0
    if marker["name"] == "tip":
        marker["hsv_ranges"] = [
            {"lower": [166, 20, 35], "upper": [174, 140, 255]},
        ]
        marker["min_area_px"] = 35.0

tracking_config = {
    "morphology_kernel": 3,
    "min_confidence": 0.55,
    "ambiguous_area_ratio": 0.72,
    "brightness_warning_delta": 28,
    "brightness_stop_delta": 55,
    "bounded_value_adjustment": 18,
}


def evaluate_image(path: Path) -> dict:
    # OpenCV's Windows file APIs do not reliably accept Korean path segments.
    frame = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"cannot read {path}")
    tracker = ColorMarkerTracker(CalibrationProfile.from_dict(candidate), tracking_config)
    result = tracker.process(frame, time.monotonic())
    return {
        "status": str(result.status),
        "brightness": result.mean_brightness,
        "detections": {
            name: {
                "status": str(item.status),
                "center": item.center_px,
                "area": item.area_px,
                "confidence": item.confidence,
                "candidates": item.candidates,
            }
            for name, item in result.detections.items()
        },
    }


def evaluate_video(path: Path) -> dict:
    capture = cv2.VideoCapture(str(path.relative_to(ROOT)))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {path}")
    tracker = ColorMarkerTracker(CalibrationProfile.from_dict(candidate), tracking_config)

    statuses: Counter[str] = Counter()
    marker_statuses = {"pivot": Counter(), "tip": Counter()}
    frames = 0
    max_candidates = {"pivot": 0, "tip": 0}
    min_confidence = {"pivot": 1.0, "tip": 1.0}
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        result = tracker.process(frame, frames / 30.0, frames)
        statuses[str(result.status)] += 1
        for name in ("pivot", "tip"):
            detection = result.detections[name]
            marker_statuses[name][str(detection.status)] += 1
            max_candidates[name] = max(max_candidates[name], detection.candidates)
            if detection.valid:
                min_confidence[name] = min(min_confidence[name], detection.confidence)
        frames += 1
    capture.release()
    return {
        "frames": frames,
        "statuses": dict(statuses),
        "marker_statuses": {name: dict(value) for name, value in marker_statuses.items()},
        "max_candidates": max_candidates,
        "min_valid_confidence": min_confidence,
    }


output = {
    "candidate_profile": candidate,
    "current_frames": {
        name: evaluate_image(ROOT / "experiments" / name)
        for name in ("cartpole-current.jpg", "cartpole-held-current.jpg")
    },
    "videos": {},
}
for video_path in sorted((ROOT / "experiments/videos").glob("*.avi")):
    output["videos"][video_path.name] = evaluate_video(video_path)

print(json.dumps(output, ensure_ascii=False, indent=2))
