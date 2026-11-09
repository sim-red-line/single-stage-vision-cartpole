from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TRIALS = {
    "successful_3s": "20261030T154400Z-1a0b40c7",
    "failed_5s": "20261030T155423Z-29d98ef0",
}


def read_log(experiment_id: str) -> dict[str, np.ndarray]:
    path = ROOT / "experiments" / "logs" / f"{experiment_id}.csv"
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    numeric = [
        "elapsed_s",
        "theta_rad",
        "omega_rad_s",
        "cart_position_mm",
        "cart_velocity_mm_s",
        "requested_velocity_mm_s",
        "limited_velocity_mm_s",
        "angle_p",
        "angle_d",
        "motor_position_mm",
        "motor_velocity_mm_s",
    ]
    return {
        name: np.asarray([float(row[name]) for row in rows], dtype=float)
        for name in numeric
    }


def summarize(data: dict[str, np.ndarray]) -> dict:
    t = data["elapsed_s"]
    theta = data["theta_rad"]
    command = data["limited_velocity_mm_s"]
    motor_velocity = data["motor_velocity_mm_s"]
    motor_position = data["motor_position_mm"]
    saturated = np.abs(command) >= 79.99
    zero_crossings = int(np.count_nonzero(np.diff(np.signbit(theta))))
    direction_changes = int(np.count_nonzero(np.diff(np.signbit(command))))
    points = []
    for target in np.arange(0.0, t[-1] + 1e-9, 0.2):
        index = int(np.argmin(np.abs(t - target)))
        points.append(
            {
                "t": round(float(t[index]), 3),
                "theta": round(float(theta[index]), 5),
                "omega": round(float(data["omega_rad_s"][index]), 4),
                "command": round(float(command[index]), 2),
                "motor_velocity": round(float(motor_velocity[index]), 2),
                "motor_position": round(float(motor_position[index]), 2),
            }
        )
    first_saturation = None
    if np.any(saturated):
        first_saturation = float(t[int(np.argmax(saturated))])
    return {
        "duration_s": float(t[-1]),
        "initial_theta_rad": float(theta[0]),
        "final_theta_rad": float(theta[-1]),
        "max_abs_theta_rad": float(np.max(np.abs(theta))),
        "max_abs_command_mm_s": float(np.max(np.abs(command))),
        "max_abs_motor_velocity_mm_s": float(np.max(np.abs(motor_velocity))),
        "motor_travel_mm": float(motor_position[-1] - motor_position[0]),
        "theta_zero_crossings": zero_crossings,
        "command_direction_changes": direction_changes,
        "first_velocity_saturation_s": first_saturation,
        "velocity_saturation_fraction": float(np.mean(saturated)),
        "points_200ms": points,
    }


def make_contact_sheet(experiment_id: str, output: Path) -> None:
    video_path = ROOT / "experiments" / "videos" / f"{experiment_id}.avi"
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {video_path}")
    frame_count = max(1, int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
    fps = max(1.0, float(capture.get(cv2.CAP_PROP_FPS)))
    indices = np.linspace(0, frame_count - 1, min(12, frame_count), dtype=int)
    tiles = []
    for index in indices:
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = capture.read()
        if not ok:
            continue
        tile = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0, 0), (130, 24), (0, 0, 0), -1)
        cv2.putText(
            tile,
            f"t={index / fps:.2f}s",
            (6, 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        tiles.append(tile)
    capture.release()
    columns = 4
    rows = int(np.ceil(len(tiles) / columns))
    blank = np.zeros_like(tiles[0])
    while len(tiles) < rows * columns:
        tiles.append(blank.copy())
    sheet = np.vstack(
        [np.hstack(tiles[row * columns : (row + 1) * columns]) for row in range(rows)]
    )
    ok, encoded = cv2.imencode(".png", sheet)
    if not ok:
        raise RuntimeError(f"cannot write {output}")
    encoded.tofile(output)


report = {}
for label, experiment_id in TRIALS.items():
    report[label] = summarize(read_log(experiment_id))
    make_contact_sheet(
        experiment_id, ROOT / "experiments" / f"contact-sheet-{label}.png"
    )

print(json.dumps(report, indent=2))
