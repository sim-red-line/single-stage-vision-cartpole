"""Offline, mode-separated comparison of a frozen baseline cohort."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics


def summarize_monitor(data: dict) -> dict:
    samples = data["samples"]
    result = data["result"]
    intervals = []
    start = last = None
    stable_start = None
    stable_time = 0.0
    balance_angles, balance_commands = [], []
    first_time = samples[0]["t"] if samples else 0.0
    previous_time = None
    capture_times = []
    previous_mode = None
    all_angles = []
    for sample in samples:
        now = float(sample["t"])
        angle = sample.get("theta")
        finite = angle is not None and math.isfinite(float(angle))
        wrapped = math.atan2(math.sin(float(angle)), math.cos(float(angle))) if finite else None
        if finite:
            all_angles.append(wrapped)
        gap = previous_time is not None and now - previous_time > .25
        if sample.get("mode") == "balance" and previous_mode != "balance" and sample.get("safety") == "RUNNING":
            capture_times.append(now - first_time)
        balance = (sample.get("mode") == "balance" and sample.get("safety") == "RUNNING"
                   and finite and 0 <= now - sample["state_timestamp_s"] <= .12)
        if gap and start is not None:
            intervals.append([start - first_time, last - first_time])
            start = last = stable_start = None
        if balance:
            if start is None:
                start = now
            last = now
            balance_angles.append(wrapped)
            balance_commands.append(float(sample.get("limited") or 0.0))
            if abs(wrapped) <= math.radians(5):
                stable_start = now if stable_start is None else stable_start
                stable_time = max(stable_time, now - stable_start)
            else:
                stable_start = None
        else:
            if start is not None:
                intervals.append([start - first_time, last - first_time])
            start = last = stable_start = None
        previous_time = now
        previous_mode = sample.get("mode")
    if start is not None:
        intervals.append([start - first_time, last - first_time])
    return {
        "experiment_id": result.get("experiment_id"),
        "captured": bool(capture_times), "capture_count": len(capture_times),
        "time_to_first_capture_s": capture_times[0] if capture_times else None,
        "balance_intervals_s": intervals,
        "verified_balance_time_s": sum(b - a for a, b in intervals),
        "longest_balance_interval_s": max((b-a for a,b in intervals), default=0),
        "continuous_stable_balance_s": stable_time,
        "balance_rms_deg": (math.degrees(math.sqrt(statistics.fmean(a*a for a in balance_angles)))
                            if balance_angles else None),
        "balance_speed_saturation_fraction": (
            sum(abs(v) >= 240 * .995 for v in balance_commands) / len(balance_commands)
            if balance_commands else None
        ),
        "max_rise_deg": 180 - math.degrees(min(map(abs, all_angles))) if all_angles else None,
        "pid_assessment": "balance_observed" if capture_times else "not_evaluable_no_capture",
        "start_motor_mm": result.get("start_position_mm"),
        "initial_frame_age_ms": ((samples[0]["t"] - samples[0]["state_timestamp_s"]) * 1000
                                 if samples else None),
        "initial_omega_rad_s": samples[0].get("omega") if samples else None,
        "startup_kick_observed": any("startup kick" in str(s.get("reason", ""))
                                     for s in samples if s["t"] - first_time < .5),
        "end_motor_mm": result.get("end_position_mm"),
        "stopped": (result.get("final_motor_target_mm_s") == 0
                    and result.get("final_motor_velocity_mm_s") == 0),
        "runner_success": result.get("stabilization_success"),
        "abort_reason": result.get("abort_reason"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path,
                        default=Path("experiments/maintenance/baseline-repeats-20261031.manifest.json"))
    parser.add_argument("--output", type=Path,
                        default=Path("experiments/maintenance/baseline-repeats-20261031.analysis.json"))
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    rows = []
    for entry in manifest["results"]:
        ident = entry["experiment_id"]
        monitor = json.loads(Path(f"experiments/logs/{ident}.monitor.json").read_text())
        row = summarize_monitor(monitor)
        preparation = json.loads(Path(entry["preparation_report"]).read_text())
        row["camera_start_median_mm"] = preparation["camera_position_median_mm"]
        row["start_brightness"] = preparation["after"]["tracking"]["brightness"]
        rows.append(row)
    captures = sum(row["captured"] for row in rows)
    report = {"offline_only": True, "baseline_id": manifest["baseline_id"],
              "trials": rows, "capture_success_count": captures, "completed_trials": len(rows),
              "pid_tuning_eligible": any(row["longest_balance_interval_s"] >= 1 for row in rows),
              "note": "Balance intervals use confirmed monitor samples; boundaries have ~50 ms resolution. "
                      "No-capture trials provide no PID quality score."}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
