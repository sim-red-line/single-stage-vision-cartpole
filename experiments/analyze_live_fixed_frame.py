from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

from raspberry_pi.calibration import frame_brightness


ROOT = Path(__file__).resolve().parents[1]
IMAGE = (
    Path(sys.argv[1])
    if len(sys.argv) > 1
    else ROOT / "experiments" / "cartpole-live.jpg"
)


def load_image(path: Path) -> np.ndarray:
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot decode {path}")
    return image


def roi_stats(hsv: np.ndarray, bounds: tuple[int, int, int, int]) -> dict:
    x0, y0, x1, y1 = bounds
    pixels = hsv[y0:y1, x0:x1].reshape(-1, 3)
    colorful = pixels[(pixels[:, 1] >= 20) & (pixels[:, 2] >= 20)]
    hue_hist = np.bincount(colorful[:, 0], minlength=180)
    top_hues = np.argsort(hue_hist)[-12:][::-1]
    return {
        "bounds": bounds,
        "pixels": int(len(pixels)),
        "colorful_pixels": int(len(colorful)),
        "p05": np.percentile(colorful, 5, axis=0).round(1).tolist(),
        "median": np.median(colorful, axis=0).round(1).tolist(),
        "p95": np.percentile(colorful, 95, axis=0).round(1).tolist(),
        "top_hues": [
            [int(hue), int(hue_hist[hue])] for hue in top_hues if hue_hist[hue] > 0
        ],
    }


def candidates(
    hsv: np.ndarray,
    lower: tuple[int, int, int],
    upper: tuple[int, int, int],
    min_area: float,
    max_area: float,
) -> dict:
    mask = cv2.inRange(
        hsv,
        np.asarray(lower, dtype=np.uint8),
        np.asarray(upper, dtype=np.uint8),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if min_area <= area <= max_area:
            moments = cv2.moments(contour)
            if moments["m00"]:
                results.append(
                    {
                        "area": round(area, 1),
                        "center": [
                            round(moments["m10"] / moments["m00"], 1),
                            round(moments["m01"] / moments["m00"], 1),
                        ],
                    }
                )
    return {
        "lower": lower,
        "upper": upper,
        "mask_pixels": int(np.count_nonzero(mask)),
        "candidates": sorted(results, key=lambda item: item["area"], reverse=True),
    }


frame = load_image(IMAGE)
hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
report = {
    "image": str(IMAGE),
    "shape": list(frame.shape),
    "brightness": {
        "full": frame_brightness(frame),
        "static_roi": frame_brightness(frame, [0.70, 0.10, 0.95, 0.35]),
    },
    "rois": {
        "tip_tight": roi_stats(hsv_frame, (310, 97, 327, 114)),
        "tip_wide": roi_stats(hsv_frame, (304, 90, 332, 121)),
        "pivot": roi_stats(hsv_frame, (304, 232, 336, 266)),
    },
    "ranges": {
        "tip_old": candidates(hsv_frame, (166, 20, 35), (174, 140, 255), 20, 250),
        "tip_160_179": candidates(hsv_frame, (160, 25, 30), (179, 200, 255), 20, 250),
        "tip_155_179": candidates(hsv_frame, (155, 30, 30), (179, 220, 255), 20, 250),
        "tip_fixed_candidate": candidates(
            hsv_frame, (158, 45, 55), (179, 200, 255), 30, 250
        ),
        "pivot_old": candidates(hsv_frame, (96, 80, 20), (112, 255, 255), 40, 240),
        "pivot_fixed_candidate": candidates(
            hsv_frame, (96, 160, 70), (112, 255, 255), 30, 240
        ),
    },
}
print(json.dumps(report, ensure_ascii=False, indent=2))
