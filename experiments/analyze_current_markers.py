from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np


image_path = Path("/tmp/cartpole-current.jpg")
image = cv2.imread(str(image_path))
if image is None:
    raise SystemExit(f"cannot read {image_path}")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

regions = {
    "pivot": (298, 237, 321, 263),
    "tip": (297, 92, 321, 118),
}

for marker_name, (x0, y0, x1, y1) in regions.items():
    pixels = hsv[y0:y1, x0:x1].reshape(-1, 3)
    colorful = pixels[(pixels[:, 1] >= 30) & (pixels[:, 2] >= 20)]
    print(marker_name)
    print("  colorful_pixels", len(colorful))
    print("  hsv_median", np.median(colorful, axis=0).round(1).tolist())
    print("  hsv_p05", np.percentile(colorful, 5, axis=0).round(1).tolist())
    print("  hsv_p95", np.percentile(colorful, 95, axis=0).round(1).tolist())

for profile_name in ("default", "color-sweep-20261030"):
    payload = json.loads(
        Path(f"calibration/profiles/{profile_name}.json").read_text(encoding="utf-8")
    )
    print(profile_name)
    for marker in payload["markers"]:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for bounds in marker["hsv_ranges"]:
            mask |= cv2.inRange(
                hsv,
                np.array(bounds["lower"], dtype=np.uint8),
                np.array(bounds["upper"], dtype=np.uint8),
            )
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = sorted((cv2.contourArea(item) for item in contours), reverse=True)
        accepted = [
            area
            for area in areas
            if marker["min_area_px"] <= area <= marker["max_area_px"]
        ]
        print(
            " ",
            marker["name"],
            "mask_pixels",
            int(mask.sum() // 255),
            "largest_areas",
            [round(value, 1) for value in areas[:5]],
            "accepted",
            [round(value, 1) for value in accepted[:5]],
        )
