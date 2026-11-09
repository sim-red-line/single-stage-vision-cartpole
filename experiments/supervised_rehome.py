"""One foreground, bounded firmware home; ESTOP aborts an unfinished home."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import time

from experiments.run_upright_direction_trial import request


def interrupted(signum, frame):
    raise KeyboardInterrupt(f"home monitor interrupted: {signum}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dashboard-pid", type=int, required=True)
    args = parser.parse_args()
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "samples": [],
              "completed": False, "home_requested": False}
    path = Path("experiments/maintenance") / (
        "hold15-rehome-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
    before = request("/api/status")
    motor, state = before["motor"], before["state"]
    if (before["safety_state"] != "UNHOMED" or not motor["connected"]
            or motor["estop"] or motor["fault"] or not motor["watchdog_ok"]
            or motor["left_limit"] or motor["right_limit"]
            or motor["velocity_mm_s"] != 0 or motor["target_velocity_mm_s"] != 0
            or not before["system_health"]["safe_for_control"]
            or not state["valid"] or not before["calibration"]["live_verified"]
            or abs(math.pi - abs(math.atan2(math.sin(state["theta_rad"]),
                                          math.cos(state["theta_rad"])))) > .10
            or abs(state["omega_rad_s"]) > .2):
        raise RuntimeError("homing preflight failed; no HOME sent")
    report["before"] = before
    print("HOME_READY countdown10s firmware seek40/backoff20mm_s", flush=True)
    slot = time.monotonic() + 10
    completed = False
    try:
        while time.monotonic() < slot:
            time.sleep(min(.1, max(0, slot - time.monotonic())))
        s = request("/api/status")
        if (time.monotonic() - slot > 1 or s["safety_state"] != "UNHOMED"
                or not s["system_health"]["safe_for_control"]
                or not s["state"]["valid"] or not s["calibration"]["live_verified"]
                or abs(math.pi - abs(math.atan2(math.sin(s["state"]["theta_rad"]),
                                               math.cos(s["state"]["theta_rad"])))) > .10
                or abs(s["state"]["omega_rad_s"]) > .2
                or s["motor"]["velocity_mm_s"] != 0
                or s["motor"]["target_velocity_mm_s"] != 0):
            raise RuntimeError("homing start deadline or stopped prerequisite changed")
        report["home_requested"] = True  # Cover an accepted command with a lost reply.
        request("/api/control/home", {})
        deadline = time.monotonic() + 55
        good_since = None
        last_print = 0.0
        while time.monotonic() < deadline:
            s = request("/api/status"); motor = s["motor"]; now = time.monotonic()
            phase = motor.get("raw", {}).get("home_state", "unknown")
            report["samples"].append({"t": now, "motor": motor,
                                       "power": s["system_health"], "tracking": s["tracking"]["status"]})
            if (not motor["connected"] or not motor["watchdog_ok"]
                    or motor["estop"] or motor["fault"]
                    or now - motor["last_rx_s"] > .75
                    or not s["system_health"]["safe_for_control"]):
                raise RuntimeError("home monitor hardware/power interlock")
            # Limit contacts are expected during the firmware's autonomous home.
            good = (motor["homed"] and phase == "none" and not motor["left_limit"]
                    and not motor["right_limit"] and motor["velocity_mm_s"] == 0
                    and motor["target_velocity_mm_s"] == 0)
            good_since = (good_since or now) if good else None
            if good_since and now - good_since >= 1.0:
                completed = True; report["completed"] = True; report["after"] = s
                break
            if now - last_print >= 2:
                print("HOME_PROGRESS", phase, motor["position_mm"], motor["velocity_mm_s"], flush=True)
                last_print = now
            time.sleep(.1)
        if not completed:
            raise RuntimeError("firmware home exceeded55s monitor deadline")
    except BaseException as exc:
        report["error"] = repr(exc)
    finally:
        if report["home_requested"] and not completed:
            try:
                request("/api/control/estop", {})
                report["abort_estop_sent"] = True
            except Exception as exc:
                report["abort_estop_error"] = str(exc)
                # Stop heartbeats so the independent400ms RP2040 watchdog aborts.
                cmdline = Path(f"/proc/{args.dashboard_pid}/cmdline")
                if cmdline.exists() and b"raspberry_pi.main" in cmdline.read_bytes():
                    os.kill(args.dashboard_pid, signal.SIGTERM)
                    time.sleep(2)
                    if cmdline.exists() and b"raspberry_pi.main" in cmdline.read_bytes():
                        os.kill(args.dashboard_pid, signal.SIGKILL)
                    time.sleep(.6)
                    report["abort_watchdog_fallback"] = True
        path.write_text(json.dumps(report, indent=2))
    if not completed:
        raise RuntimeError(f"homing aborted; inspect {path}")
    print("HOME_COMPLETED", json.dumps({"report": str(path), "motor": report["after"]["motor"]}), flush=True)


if __name__ == "__main__":
    for event in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(event, interrupted)
    main()
