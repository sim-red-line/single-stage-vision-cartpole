"""Verify recorded raw-angle duration within confirmed balance-mode intervals.

Read-only: no API, hardware or motor access. This checks the measurement
criterion; it does not separate passive friction from closed-loop control.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import statistics


def verify(root: Path, ident: str, target_s: float = 15.0) -> dict:
    if not re.fullmatch(r"[A-Za-z0-9_-]+", ident):
        raise ValueError("invalid experiment identifier")
    if not math.isfinite(target_s) or not 0 < target_s <= 60:
        raise ValueError("target must be finite and within0..60seconds")
    monitor = json.loads((root/f"experiments/logs/{ident}.monitor.json").read_text(encoding="utf-8"))
    rows = list(csv.DictReader((root/f"experiments/logs/{ident}.csv").open(encoding="utf-8")))
    result = monitor["result"]
    assert rows and result["experiment_id"] == ident
    assert all(row["experiment_id"] == ident for row in rows)
    assert result["balance_hold_target_s"] == target_s
    intervals = []
    start = last = previous = None
    for sample in monitor["samples"]:
        now = float(sample["t"])
        good = (sample["mode"] == "balance" and sample["safety"] == "RUNNING"
                and 0 <= now-sample["state_timestamp_s"] <= .12)
        gap = previous is not None and now-previous > .25
        if start is not None and (not good or gap):
            intervals.append((start, last))
            start = last = None
        if good:
            start = now if start is None else start
            last = now
        previous = now
    if start is not None:
        intervals.append((start, last))
    best = (0.0, None, None)
    start = previous = None
    for row in rows:
        now = float(row["timestamp_monotonic_s"])
        angle = float(row["theta_rad"])
        angle = math.atan2(math.sin(angle), math.cos(angle))
        good = (any(lo <= now <= hi for lo, hi in intervals)
                and row["safety_state"] == "RUNNING"
                and row["tracking_valid"] == "True" and row["tracking_status"] == "ok"
                and 0 <= float(row["control_latency_ms"]) <= 120
                and abs(angle) <= math.radians(5))
        if previous is not None and now-previous > .12:
            start = None
        if good:
            start = now if start is None else start
            if now-start > best[0]:
                best = (now-start, start, now)
        else:
            start = None
        previous = now
    stopped = result["final_motor_target_mm_s"] == result["final_motor_velocity_mm_s"] == 0
    met = best[0] >= target_s and result["observed_continuous_stable_s"] >= target_s and stopped
    stable = [r for r in rows if best[1] is not None and best[1] <= float(r["timestamp_monotonic_s"]) <= best[2]]
    angles = [math.degrees(math.atan2(math.sin(float(r["theta_rad"])), math.cos(float(r["theta_rad"])))) for r in stable]
    origin = float(rows[0]["timestamp_monotonic_s"])
    return {"offline_only": True, "experiment_id": ident, "target_s": target_s,
            "angle_duration_criterion_met": met, "raw_continuous_stable_s": best[0],
            "interval_elapsed_s": None if best[1] is None else [best[1]-origin, best[2]-origin],
            "stable_samples": len(stable),
            "stable_rms_deg": math.sqrt(statistics.fmean(a*a for a in angles)) if angles else None,
            "stable_max_abs_deg": max(map(abs, angles), default=None),
            "stable_motor_range_mm": [min(float(r["motor_position_mm"]) for r in stable), max(float(r["motor_position_mm"]) for r in stable)] if stable else None,
            "runner_hold_s": result["observed_continuous_stable_s"],
            "reported_motor_stopped": stopped,
            "isolates_passive_support": False,
            "method": "Fresh raw CSV angles within confirmed balance-mode intervals; gaps reset continuity."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment_id")
    parser.add_argument("--target", type=float, default=15.0)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    report = verify(root, args.experiment_id, args.target)
    output = root/f"experiments/maintenance/{args.experiment_id}.verified.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["angle_duration_criterion_met"] else 1)
