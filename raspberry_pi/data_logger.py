"""Non-blocking CSV logger and atomic experiment-summary writer."""

from __future__ import annotations

import csv
import json
import queue
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .metrics import calculate_metrics, representative_series


CSV_FIELDS = [
    "experiment_id",
    "elapsed_s",
    "timestamp_monotonic_s",
    "theta_rad",
    "omega_rad_s",
    "cart_position_mm",
    "cart_velocity_mm_s",
    "target_angle_rad",
    "requested_velocity_mm_s",
    "limited_velocity_mm_s",
    "angle_p",
    "angle_i",
    "angle_d",
    "position_p",
    "position_i",
    "position_d",
    "output_saturated",
    "tracking_valid",
    "tracking_status",
    "tracking_confidence",
    "processing_ms",
    "fps",
    "control_latency_ms",
    "motor_position_mm",
    "motor_velocity_mm_s",
    "safety_state",
]


@dataclass(slots=True)
class ActiveExperiment:
    experiment_id: str
    started_monotonic_s: float
    started_utc: str
    csv_path: Path
    gains: dict[str, float]
    notes: str


class ExperimentLogger:
    def __init__(self, logs_dir: str | Path, summaries_dir: str | Path, config: dict) -> None:
        self.logs_dir = Path(logs_dir)
        self.summaries_dir = Path(summaries_dir)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.summaries_dir.mkdir(parents=True, exist_ok=True)
        self.queue: queue.Queue[tuple[Path, dict[str, Any]] | None] = queue.Queue(
            maxsize=int(config.get("queue_size", 10000))
        )
        self.flush_period_s = float(config.get("flush_period_s", 0.5))
        self.metric_config = {
            "stable_angle_rad": float(config.get("stable_angle_rad", 0.0872665)),
            "success_time_s": float(config.get("success_time_s", 5.0)),
            "cart_center_mm": float(config.get("cart_center_mm", 250.0)),
            "velocity_limit_mm_s": float(config.get("velocity_limit_mm_s", 600.0)),
        }
        self._active: ActiveExperiment | None = None
        self._samples: list[dict[str, Any]] = []
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._dropped_rows = 0
        self._worker = threading.Thread(target=self._writer_loop, name="csv-writer", daemon=True)
        self._worker.start()

    @property
    def active(self) -> ActiveExperiment | None:
        with self._lock:
            return self._active

    @property
    def dropped_rows(self) -> int:
        return self._dropped_rows

    def start(self, gains: dict[str, float], notes: str = "") -> ActiveExperiment:
        with self._lock:
            if self._active is not None:
                raise RuntimeError("an experiment is already active")
            timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
            experiment_id = f"{timestamp}-{uuid.uuid4().hex[:8]}"
            csv_path = self.logs_dir / f"{experiment_id}.csv"
            with csv_path.open("x", newline="", encoding="utf-8") as handle:
                csv.DictWriter(handle, fieldnames=CSV_FIELDS).writeheader()
            self._active = ActiveExperiment(
                experiment_id,
                time.monotonic(),
                datetime.now(UTC).isoformat(),
                csv_path,
                dict(gains),
                notes,
            )
            self._samples = []
            self._dropped_rows = 0
            return self._active

    def append(self, sample: dict[str, Any]) -> None:
        with self._lock:
            active = self._active
            if active is None:
                return
            row = {field: sample.get(field) for field in CSV_FIELDS}
            row["experiment_id"] = active.experiment_id
            row["elapsed_s"] = float(sample.get("timestamp_monotonic_s", time.monotonic())) - active.started_monotonic_s
            self._samples.append(dict(row))
            try:
                self.queue.put_nowait((active.csv_path, row))
            except queue.Full:
                self._dropped_rows += 1

    def finish(self, extra: dict[str, Any] | None = None) -> dict[str, Any]:
        with self._lock:
            active = self._active
            if active is None:
                raise RuntimeError("no experiment is active")
            self._active = None
            samples = self._samples
            self._samples = []
        self.queue.join()
        metrics = calculate_metrics(samples, **self.metric_config)
        summary = {
            "schema_version": 1,
            "experiment_id": active.experiment_id,
            "started_utc": active.started_utc,
            "finished_utc": datetime.now(UTC).isoformat(),
            "gains": active.gains,
            "notes": active.notes,
            "log_file": str(active.csv_path),
            "dropped_log_rows": self._dropped_rows,
            "metrics": metrics.as_dict(),
            "representative_series": representative_series(samples),
        }
        if extra:
            summary["context"] = extra
        target = self.summaries_dir / f"{active.experiment_id}.json"
        temporary = target.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(target)
        summary["summary_file"] = str(target)
        return summary

    def _writer_loop(self) -> None:
        handles: dict[Path, Any] = {}
        writers: dict[Path, csv.DictWriter] = {}
        last_flush = time.monotonic()
        try:
            while not self._stop.is_set() or not self.queue.empty():
                try:
                    item = self.queue.get(timeout=0.1)
                except queue.Empty:
                    item = None
                if item is not None:
                    path, row = item
                    if path not in handles:
                        handles[path] = path.open("a", newline="", encoding="utf-8")
                        writers[path] = csv.DictWriter(handles[path], fieldnames=CSV_FIELDS)
                    writers[path].writerow(row)
                    self.queue.task_done()
                now = time.monotonic()
                if now - last_flush >= self.flush_period_s:
                    for handle in handles.values():
                        handle.flush()
                    last_flush = now
        finally:
            for handle in handles.values():
                handle.flush()
                handle.close()

    def close(self) -> None:
        self._stop.set()
        self._worker.join(timeout=2.0)

