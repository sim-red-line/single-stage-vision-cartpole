"""Prepare the recorded baseline start under exclusive, low-speed ownership.

This command never ARM/STARTs the controller. It returns with the dashboard
idle after approaching the reviewed start from the right, via a260mm waypoint.
The default is244.35mm; explicit targets retain the existing center +/-20mm band.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
from datetime import datetime, timezone

from experiments.run_upright_direction_trial import preflight, request
from raspberry_pi.config import load_config


BASELINE_HASH = "a7836e973d6e3e95ff110f65606bb0103ee44049acd0ce6ba486f1a4fc8c8e2f"
TARGET_MM = 244.35


def validate_start_target(target: float, center: float) -> float:
    if not math.isfinite(target) or not math.isfinite(center) or abs(target - center) > 20:
        raise ValueError("trial start must remain within20mm of configured center")
    return target


def verify_frozen_files(root: Path, manifest: Path, config_sha256: str) -> dict:
    expected = json.loads(manifest.read_text())
    if expected["files_sha256"].get("config.local.yaml") != config_sha256:
        raise RuntimeError("manifest does not match the explicit configuration digest")
    if hashlib.sha256((root / "config.local.yaml").read_bytes()).hexdigest() != config_sha256:
        raise RuntimeError("expected configuration changed; no positioning")
    for name, digest in expected["files_sha256"].items():
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError("frozen baseline source/configuration changed: " + name)
    return expected


def wait_rest(timeout_s: float = 45.0) -> dict:
    deadline = time.monotonic() + timeout_s
    good_since = None
    while time.monotonic() < deadline:
        status = request("/api/status")
        motor, state = status["motor"], status["state"]
        if (not motor["connected"] or not motor["homed"] or not motor["watchdog_ok"]
                or any(motor[k] for k in ("fault", "estop", "left_limit", "right_limit"))
                or not status["system_health"]["safe_for_control"]
                or status["experiment"]["active"] is not None):
            raise RuntimeError("baseline preparation hardware prerequisite failed")
        if status["safety_state"] == "FAULT":
            if status["safety_reason"] != "swing-up time limit":
                raise RuntimeError("unexpected fault; no automatic clearing")
            if motor["velocity_mm_s"] == motor["target_velocity_mm_s"] == 0:
                request("/api/control/clear-fault", {})
            good_since = None
        else:
            theta, omega = state.get("theta_rad"), state.get("omega_rad_s")
            good = (status["safety_state"] == "IDLE" and state["valid"]
                    and status["tracking"]["status"] == "ok"
                    and status["calibration"]["live_verified"]
                    and motor["velocity_mm_s"] == motor["target_velocity_mm_s"] == 0
                    and theta is not None and omega is not None
                    and abs(math.pi - abs(math.atan2(math.sin(theta), math.cos(theta)))) <= .035
                    and abs(omega) <= .08
                    and 0 <= time.monotonic() - state["timestamp_s"] <= .12)
            if good:
                good_since = good_since or time.monotonic()
                if time.monotonic() - good_since >= 1.0:
                    return status
            else:
                good_since = None
        time.sleep(.04)
    raise RuntimeError("baseline starting rest did not settle within its deadline")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path(
        "experiments/maintenance/baseline-repeats-20261031.manifest.json"))
    parser.add_argument("--expected-config-sha256", default=BASELINE_HASH,
                        help="explicit digest for a separately reviewed configuration cohort")
    parser.add_argument("--target-mm", type=float, default=TARGET_MM)
    args = parser.parse_args()
    root = Path.cwd()
    manifest = root / args.manifest
    expected = verify_frozen_files(root, manifest, args.expected_config_sha256)
    target_mm = validate_start_target(
        args.target_mm, float(load_config(root / "config.local.yaml")["controller"]["cart_center_mm"])
    )
    ident = "baseline-start-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    report = {"id": ident, "target_mm": target_mm, "moves": [], "ready": False}
    report["config_sha256"] = args.expected_config_sha256
    report["controller_sha256"] = hashlib.sha256(
        (root / "raspberry_pi/pid_controller.py").read_bytes()
    ).hexdigest()
    report["manifest"] = str(manifest)
    report["manifest_sha256"] = hashlib.sha256(manifest.read_bytes()).hexdigest()
    path = root / "experiments/maintenance" / (ident + ".json")
    child = None
    server_stopped = False
    server_args = None
    server_log = None

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"baseline preparation interrupted by signal {signum}")

    signal.signal(signal.SIGHUP, interrupted)
    signal.signal(signal.SIGTERM, interrupted)

    def move(target: float, *, trial_start: bool) -> dict:
        nonlocal child
        step = len(report["moves"]) + 1
        log_path = path.with_name(ident + f"-move{step}.stdout.txt")
        flag = "--trial-start-mm" if trial_start else "--target-mm"
        with log_path.open("w") as log:
            child = subprocess.Popen(
                [str(root / ".venv/bin/python"), "-m", "experiments.center_cart",
                 "--stage", flag, str(target)], cwd=root, stdin=subprocess.PIPE,
                stdout=log, stderr=subprocess.STDOUT, text=True,
            )
            deadline = time.monotonic() + 20
            while "CENTER_STAGED_READY" not in log_path.read_text():
                if child.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("positioning preflight failed: " + log_path.read_text()[-1200:])
                time.sleep(.1)
            print(f"BASELINE_POSITION_READY target={target:.3f} speed<=20 countdown=10s", flush=True)
            time.sleep(10)
            child.communicate(input="GO\n", timeout=15)
            if child.returncode:
                raise RuntimeError("positioning failed: " + log_path.read_text()[-1500:])
        lines = [line for line in log_path.read_text().splitlines() if line.startswith("CENTER_RESULT ")]
        result = json.loads(lines[-1].split(" ", 1)[1])
        if not result["reached_target"] or result.get("error"):
            raise RuntimeError("positioning target was not reached")
        motor = result["final_motor"]
        if motor["velocity_mm_s"] != 0 or motor["target_velocity_mm_s"] != 0:
            raise RuntimeError("positioning stop not confirmed")
        report["moves"].append(result)
        path.write_text(json.dumps(report, indent=2))
        child = None
        print("BASELINE_POSITION_DONE", motor["position_mm"], flush=True)
        return motor

    try:
        request("/api/control/stop", {})
        status = wait_rest()
        preflight(status, energy_swing_up=True, commissioning_profile="capture240")
        if expected is not None:
            if status["gains"] != expected["gains"]:
                raise RuntimeError("baseline gains changed; no positioning")
            for key, value in expected["swing_up_settings"].items():
                if status["swing_up"].get(key) != value:
                    raise RuntimeError("baseline runtime setting changed: " + key)
        report["before"] = status
        pids = subprocess.check_output(
            ["pgrep", "-f", "^/home/admin/vision-cartpole/.venv/bin/python -m raspberry_pi.main"],
            text=True,
        ).split()
        if len(pids) != 1:
            raise RuntimeError("dashboard ownership is ambiguous")
        pid = int(pids[0])
        server_args = Path("/proc", str(pid), "cmdline").read_bytes().decode().strip("\0").split("\0")
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 8
        while Path("/proc", str(pid)).exists():
            if time.monotonic() > deadline:
                raise RuntimeError("old dashboard has not exited")
            time.sleep(.1)
        server_stopped = True
        motor = status["motor"]
        while abs(motor["position_mm"] - 260.0) > 95.0:
            toward_center = 256.369 - motor["position_mm"]
            waypoint = motor["position_mm"] + math.copysign(min(90.0, abs(toward_center)), toward_center)
            motor = move(waypoint, trial_start=False)
        move(260.0, trial_start=True)
        move(target_mm, trial_start=True)
    except BaseException as exc:
        report["error"] = repr(exc)
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
                time.sleep(.6)  # Allow the firmware heartbeat watchdog to stop POS.
        if server_stopped:
            server_log = path.with_name(ident + "-runtime.log").open("ab", buffering=0)
            server = subprocess.Popen(server_args, cwd=root, stdin=subprocess.DEVNULL,
                                      stdout=server_log, stderr=subprocess.STDOUT, start_new_session=True)
            report["runtime_pid"] = server.pid
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if report.get("error"):
        raise RuntimeError(report["error"])
    deadline = time.monotonic() + 15
    while True:
        try:
            startup = request("/api/status")
            if startup["motor"]["connected"] and startup["calibration"]["live_verified"]:
                break
        except OSError:
            pass
        if time.monotonic() > deadline:
            raise RuntimeError("dashboard did not become ready after positioning")
        time.sleep(.1)
    status = wait_rest(20)
    preflight(status, energy_swing_up=True, commissioning_profile="capture240")
    if abs(status["motor"]["position_mm"] - target_mm) > .8:
        raise RuntimeError("baseline motor starting position differs from its target")
    camera = []
    for _ in range(20):
        sample = request("/api/status")
        preflight(sample, energy_swing_up=True, commissioning_profile="capture240")
        camera.append(sample["state"]["cart_position_mm"])
        time.sleep(.05)
    report.update(ready=True, after=status, camera_position_median_mm=statistics.median(camera),
                  camera_position_spread_mm=max(camera) - min(camera))
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print("BASELINE_START_READY", json.dumps({"motor_mm": status["motor"]["position_mm"],
          "camera_median_mm": report["camera_position_median_mm"], "report": str(path),
          "runtime_pid": report["runtime_pid"]}), flush=True)


if __name__ == "__main__":
    main()
