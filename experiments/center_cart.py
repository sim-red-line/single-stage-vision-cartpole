"""Supervised, recorded 20 mm/s centering with the existing RP2040 POS mode.

Stop the dashboard server first; this program owns the camera and serial port.
It never homes, changes coordinates, widens swing limits, or arms the PID.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import select
import signal
import socket
import sys
import time
import uuid

from raspberry_pi.config import load_config
from raspberry_pi.runtime import CartPoleRuntime


def check_ready(
    status: dict, config: dict, *, stationary: bool, target_mm: float | None = None,
    trial_start: bool = False,
) -> None:
    motor, state = status["motor"], status["state"]
    if status["safety_state"] != "IDLE" or status["experiment"]["active"] is not None:
        raise RuntimeError("centering requires idle, disarmed control without an experiment")
    if not (motor["connected"] and motor["homed"] and motor["watchdog_ok"]):
        raise RuntimeError("centering requires a live, homed motor and watchdog")
    if any(motor[k] for k in ("fault", "estop", "left_limit", "right_limit")):
        raise RuntimeError("motor fault, ESTOP or physical limit is active")
    if motor.get("raw", {}).get("firmware") != "cartpole-safe-8":
        raise RuntimeError("20 mm/s POS mode has not been verified for this firmware")
    if not status["system_health"]["safe_for_control"] or status["tracking"]["status"] != "ok":
        raise RuntimeError("power or tracking is unsafe")
    age = time.monotonic() - float(state["timestamp_s"])
    if not state["valid"] or not math.isfinite(age) or not 0 <= age <= config["safety"]["tracking_grace_s"]:
        raise RuntimeError("live camera state is invalid or stale")
    theta = float(state["theta_rad"])
    omega = float(state["omega_rad_s"])
    if (not math.isfinite(theta) or abs(math.atan2(math.sin(theta), math.cos(theta))) < math.pi - 0.26
            or not math.isfinite(omega) or abs(omega) > (0.2 if stationary else 2.0)):
        raise RuntimeError("pendulum must hang down for low-speed centering")
    low, high = config["safety"]["rail_min_mm"], config["safety"]["rail_max_mm"]
    center = float(config["controller"]["cart_center_mm"])
    target = center if target_mm is None else float(target_mm)
    for value in (target, motor["position_mm"], state["cart_position_mm"]):
        if value is None or not math.isfinite(float(value)) or not low <= float(value) <= high:
            raise RuntimeError("centering path must stay inside existing software rail limits")
    if abs(float(motor["position_mm"]) - target) > 100.0:
        raise RuntimeError("centering recovery is bounded to 100 mm")
    if trial_start and abs(target - center) > 20.0:
        raise RuntimeError("trial start must stay within 20 mm of configured center")
    if stationary and not trial_start and not min(float(motor["position_mm"]), center) <= target <= max(float(motor["position_mm"]), center):
        raise RuntimeError("centering waypoint must lie toward the configured center")
    speeds = [float(motor["velocity_mm_s"]), float(motor["target_velocity_mm_s"])]
    if not all(math.isfinite(v) for v in speeds) or max(map(abs, speeds)) > (0.05 if stationary else 20.5):
        raise RuntimeError("centering speed exceeds its bounded envelope")


def main() -> None:
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"positioning interrupted by signal {signum}")

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config.local.yaml")
    parser.add_argument("--stage", action="store_true", required=True)
    targets = parser.add_mutually_exclusive_group()
    targets.add_argument("--target-mm", type=float, help="Optional intermediate point toward center; each move remains <=100 mm")
    targets.add_argument("--trial-start-mm", type=float,
                         help="Trial starting point within center +/-20 mm; speed <=20 mm/s, move <=100 mm")
    args = parser.parse_args()
    config = load_config(args.config)
    trial_start = args.trial_start_mm is not None
    target = args.trial_start_mm if trial_start else (
        float(config["controller"]["cart_center_mm"]) if args.target_mm is None else args.target_mm
    )
    try:
        connection = socket.create_connection(("127.0.0.1", int(config["web"]["port"])), timeout=0.3)
    except OSError:
        pass
    else:
        connection.close()
        raise RuntimeError("stop the dashboard server before taking exclusive hardware ownership")
    root = Path.cwd()
    runtime = CartPoleRuntime(config, root=root)
    ident = "center-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
    report = {"id": ident, "centered": False, "reached_target": False, "samples": [],
              "purpose": "trial_start" if trial_start else "centering"}
    try:
        runtime.start()
        deadline = time.monotonic() + 12.0
        while True:
            status = runtime.snapshot()
            try:
                check_ready(status, config, stationary=True, target_mm=target, trial_start=trial_start)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        print(f"CENTER_STAGED_READY target={target:.3f} mm, speed<=20 mm/s, duration<=8 s; send GO", flush=True)
        readable, _, _ = select.select([sys.stdin], [], [], 60.0)
        if not readable or sys.stdin.readline().strip() != "GO":
            raise RuntimeError("centering cancelled before movement")
        status = runtime.snapshot()
        check_ready(status, config, stationary=True, target_mm=target, trial_start=trial_start)
        initial = float(status["motor"]["position_mm"])
        report["start_position_mm"], report["target_position_mm"] = initial, target
        camera = config["camera"]
        video = runtime.video_recorder.start(ident, (int(camera["width"]), int(camera["height"])), float(camera["fps"]))
        if not video.enabled or video.error or not video.path:
            raise RuntimeError("centering requires a working video recording")
        # The verified firmware independently caps POS mode to 20 mm/s and
        # retains its heartbeat, switches, soft limits and stopping logic.
        runtime.motor._write(f"POS:{target:.3f}")
        deadline, settled_since = time.monotonic() + 8.0, None
        while time.monotonic() < deadline:
            status = runtime.snapshot()
            check_ready(status, config, stationary=False, target_mm=target, trial_start=trial_start)
            motor = status["motor"]
            position = float(motor["position_mm"])
            report["samples"].append({"t": time.monotonic(), "motor": motor, "state": status["state"]})
            if not min(initial, target) - 5.0 <= position <= max(initial, target) + 5.0:
                raise RuntimeError("cart left the planned centering corridor")
            if (abs(position - target) <= 0.8 and abs(motor["velocity_mm_s"]) <= 0.05
                    and abs(motor["target_velocity_mm_s"]) <= 0.05):
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= 0.3:
                    report["reached_target"] = True
                    report["centered"] = abs(target - float(config["controller"]["cart_center_mm"])) <= 0.8
                    break
            else:
                settled_since = None
            time.sleep(0.03)
        if not report["reached_target"]:
            raise RuntimeError("centering time limit reached")
    except BaseException as exc:
        report["error"] = str(exc)
        raise
    finally:
        runtime.operator_stop("bounded centering finished")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            motor = runtime.motor.status
            if abs(motor.velocity_mm_s) <= 0.05 and abs(motor.target_velocity_mm_s) <= 0.05:
                break
            time.sleep(0.03)
        report["final_motor"] = runtime.motor.status.as_dict()
        report["video"] = asdict(runtime.video_recorder.finish())
        runtime.close()
        path = root / "experiments" / "maintenance" / f"{ident}.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print("CENTER_RESULT", json.dumps({k: v for k, v in report.items() if k != "samples"}), flush=True)


if __name__ == "__main__":
    main()
