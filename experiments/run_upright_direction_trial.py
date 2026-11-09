"""Run one tightly bounded upright balance direction trial through the local API."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import select
import sys
import time
import urllib.error
import urllib.request


BASE = "http://127.0.0.1:8000"
DURATION_S = 5.0
DEFAULT_BALANCE_HOLD_S = 5.0
MAX_BALANCE_HOLD_S = 15.0
START_ANGLE_RAD = 0.015
START_OMEGA_RAD_S = 0.15
ABORT_ANGLE_RAD = 0.12
ABORT_CART_DISPLACEMENT_MM = 80.0
MAX_SWING_CART_DISPLACEMENT_MM = 120.0
DEFAULT_COMMISSIONING_PROFILE = "commissioned160"
TRIAL_ENVELOPES = {
    "commissioned160": {"max_velocity_mm_s": 160.0, "max_acceleration_mm_s2": 1500.0,
                        "max_cart_excursion_mm": MAX_SWING_CART_DISPLACEMENT_MM},
    "capture240": {"max_velocity_mm_s": 240.0, "max_acceleration_mm_s2": 1500.0,
                   "max_cart_excursion_mm": 160.0},
}


def bounded_balance_hold(value: str) -> float:
    seconds = float(value)
    if not math.isfinite(seconds) or not 1.0 <= seconds <= MAX_BALANCE_HOLD_S:
        raise argparse.ArgumentTypeError("balance hold must be finite and within 1..15 seconds")
    return seconds


def swing_trial_hard_limit(duration_s: float, hold_s: float) -> float:
    if not math.isfinite(duration_s) or not 0.5 <= duration_s <= 30.0:
        raise ValueError("swing duration must be within 0.5..30 seconds")
    bounded_balance_hold(str(hold_s))
    return duration_s + hold_s + 0.5


def verified_balance_observation(status: dict, sampled_s: float) -> bool:
    state, motor = status["state"], status["motor"]
    theta, stamp = state.get("theta_rad"), state.get("timestamp_s")
    if theta is None or stamp is None or not math.isfinite(float(theta)):
        return False
    age = sampled_s - float(stamp)
    wrapped = math.atan2(math.sin(float(theta)), math.cos(float(theta)))
    return (
        status["swing_up"]["mode"] == "balance" and status["safety_state"] == "RUNNING"
        and state.get("valid") and 0.0 <= age <= 0.12
        and abs(wrapped) <= math.radians(5.0) and status["tracking"]["status"] == "ok"
        and motor["connected"] and motor["homed"] and motor["watchdog_ok"]
        and not any(motor[k] for k in ("fault", "estop", "left_limit", "right_limit"))
        and status["system_health"]["safe_for_control"]
    )


def request(path: str, payload: dict | None = None, *, method: str | None = None) -> dict:
    import os
    deadline_text = os.environ.get("CARTPOLE_START_DEADLINE_UNIX")
    if deadline_text is not None and path in {"/api/control/arm", "/api/control/start"}:
        deadline = float(deadline_text)
        if not math.isfinite(deadline) or time.time() > deadline:
            raise RuntimeError("scheduled ARM/START deadline expired; no late motion")
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        BASE + path,
        data=data,
        headers={"Content-Type": "application/json"} if data is not None else {},
        method=method or ("POST" if data is not None or path.startswith("/api/control/") else "GET"),
    )
    try:
        with urllib.request.urlopen(req, timeout=2.0) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{path} returned HTTP {exc.code}: {body}") from exc


def preflight(
    status: dict, *, preparatory_only: bool = False, energy_swing_up: bool = False,
    commissioning_profile: str = DEFAULT_COMMISSIONING_PROFILE,
) -> None:
    if commissioning_profile not in TRIAL_ENVELOPES:
        raise RuntimeError("unknown commissioning profile")
    state = status["state"]
    motor = status["motor"]
    failures = []
    allowed_states = {"IDLE", "CALIBRATION_REQUIRED"} if preparatory_only else {"IDLE"}
    if status["safety_state"] not in allowed_states:
        failures.append(f"safety={status['safety_state']}")
    if not preparatory_only:
        if not state["valid"] or state["theta_rad"] is None:
            failures.append("invalid vision state")
        elif not math.isfinite(float(state["theta_rad"])):
            failures.append("nonfinite pendulum angle")
        elif energy_swing_up:
            theta = float(state["theta_rad"])
            wrapped = math.atan2(math.sin(theta), math.cos(theta))
            if abs(wrapped) < math.pi - 0.25:
                failures.append("pendulum must hang down before this swing-up trial")
        elif abs(float(state["theta_rad"])) > START_ANGLE_RAD:
            failures.append(f"pendulum not upright ({state['theta_rad']:.4f} rad)")
        omega = state.get("omega_rad_s")
        omega_limit = 0.2 if energy_swing_up else START_OMEGA_RAD_S
        if omega is None or not math.isfinite(float(omega)) or abs(float(omega)) > omega_limit:
            failures.append("pendulum must be held steady before starting")
    if energy_swing_up:
        swing = status.get("swing_up", {})
        if swing.get("enabled") is not True:
            failures.append("energy swing-up is not enabled")
        if not 0 < float(swing.get("max_duration_s", 0)) <= 30.0:
            failures.append("bounded internal swing-up timeout is not configured")
        for name, ceiling in TRIAL_ENVELOPES[commissioning_profile].items():
            value = float(swing.get(name, 0))
            if not math.isfinite(value) or not 0 < value <= ceiling:
                failures.append(f"swing-up {name} is outside the commissioned trial envelope")
        balance_velocity = float(swing.get("balance_max_velocity_mm_s", swing.get("max_velocity_mm_s", 0)))
        if (not math.isfinite(balance_velocity) or balance_velocity <= 0
                or balance_velocity > TRIAL_ENVELOPES[commissioning_profile]["max_velocity_mm_s"]):
            failures.append("balance velocity is outside the commissioned trial envelope")
    if not motor["connected"] or not motor["homed"] or not motor.get("watchdog_ok") or motor["estop"] or motor["fault"]:
        failures.append("motor is not ready")
    if motor["left_limit"] or motor["right_limit"]:
        failures.append("limit input active")
    speeds = [float(motor["target_velocity_mm_s"]), float(motor["velocity_mm_s"])]
    if any(not math.isfinite(v) or abs(v) > 0.05 for v in speeds):
        failures.append("motor is not stationary")
    if not preparatory_only and not status["calibration"]["live_verified"]:
        failures.append("calibration not live-verified")
    if not status["system_health"]["safe_for_control"]:
        failures.append("Pi power health not safe")
    if any(not math.isfinite(float(status["gains"][k])) or status["gains"][k] <= 0
           for k in ("kp_angle", "kd_angle")):
        failures.append("balance gains are not active")
    if status.get("controller_config", {}).get("output_mode") != "acceleration":
        failures.append("acceleration-output controller is not active")
    if failures:
        raise RuntimeError("preflight failed: " + "; ".join(failures))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--stage", action="store_true")
    parser.add_argument("--notes", default="")
    parser.add_argument("--swing-up", action="store_true")
    parser.add_argument("--balance-hold-seconds", type=bounded_balance_hold,
                        default=DEFAULT_BALANCE_HOLD_S)
    parser.add_argument("--commissioning-profile", choices=tuple(TRIAL_ENVELOPES),
                        default=DEFAULT_COMMISSIONING_PROFILE)
    parser.add_argument("--energy-gain", type=float)
    parser.add_argument("--energy-margin", type=float)
    parser.add_argument("--energy-rate-lag", type=float)
    parser.add_argument("--kick-acceleration", type=float)
    parser.add_argument("--kick-duration", type=float)
    args = parser.parse_args()
    if args.commissioning_profile != DEFAULT_COMMISSIONING_PROFILE and not args.swing_up:
        parser.error("expanded commissioning profiles require --swing-up")
    duration_s = float(args.duration)
    max_duration_s = 30.0 if args.swing_up else 10.0
    if not 0.5 <= duration_s <= max_duration_s:
        raise RuntimeError(f"duration must be between 0.5 and {max_duration_s:g} seconds")
    hold_s = args.balance_hold_seconds
    if not args.swing_up and hold_s != DEFAULT_BALANCE_HOLD_S:
        parser.error("balance hold verification requires --swing-up")
    max_run_s = swing_trial_hard_limit(duration_s, hold_s) if args.swing_up else duration_s
    energy_parameters = {
        "energy_gain": args.energy_gain,
        "energy_target_margin": args.energy_margin,
        "energy_rate_lag_s": args.energy_rate_lag,
        "kick_acceleration_mm_s2": args.kick_acceleration,
        "kick_duration_s": args.kick_duration,
    }
    energy_parameters = {name: value for name, value in energy_parameters.items() if value is not None}
    if energy_parameters and not args.swing_up:
        parser.error("energy parameters require --swing-up")
    if args.swing_up:
        energy_parameters["max_duration_s"] = duration_s
        request("/api/control/swing-up/parameters", energy_parameters, method="PUT")
    initial = request("/api/status")
    # Complete hardware preparation with the pendulum hanging down.  No ARM
    # command is sent while waiting; GO still rechecks every upright/vision
    # interlock immediately before the bounded trial.
    preflight(initial, preparatory_only=args.stage, energy_swing_up=args.swing_up,
              commissioning_profile=args.commissioning_profile)
    displacement_limit = (
        min(TRIAL_ENVELOPES[args.commissioning_profile]["max_cart_excursion_mm"],
            float(initial["swing_up"]["max_cart_excursion_mm"]))
        if args.swing_up else ABORT_CART_DISPLACEMENT_MM
    )
    if args.stage:
        print(
            "STAGED_READY motor stopped; pendulum may stay down; send GO within 60 seconds",
            flush=True,
        )
        readable, _, _ = select.select([sys.stdin], [], [], 60.0)
        if not readable or sys.stdin.readline().strip() != "GO":
            raise RuntimeError("staged start cancelled; motor was never armed")
        initial = request("/api/status")
        preflight(initial, energy_swing_up=args.swing_up,
                  commissioning_profile=args.commissioning_profile)
    samples: list[dict] = []
    experiment_started = False
    control_armed = False
    control_started = False
    abort_reason = "duration complete"
    summary = None
    try:
        request("/api/control/arm", {})
        control_armed = True
        request(
            "/api/experiments/start",
            {
                "notes": args.notes or (
                    f"{duration_s:g} s supervised "
                    f"{'energy swing-up' if args.swing_up else 'upright direction'} trial, "
                    f"Kp={initial['gains']['kp_angle']:g} "
                    f"Kd={initial['gains']['kd_angle']:g}"
                )
            },
        )
        experiment_started = True
        request("/api/control/start", {})
        control_started = True
        run_started_s = time.monotonic()
        deadline = run_started_s + duration_s
        hard_deadline = run_started_s + max_run_s
        stable_since = None
        previous_poll_s = None
        best_stable_s = 0.0
        stabilized = False
        while time.monotonic() < deadline:
            status = request("/api/status")
            state = status["state"]
            control = status["control"]
            motor = status["motor"]
            theta = state.get("theta_rad")
            sampled_s = time.monotonic()
            poll_gap_ok = previous_poll_s is None or sampled_s - previous_poll_s <= 0.25
            previous_poll_s = sampled_s
            sample = {
                "t": round(sampled_s, 4),
                "state_timestamp_s": state.get("timestamp_s"),
                "motor_status_timestamp_s": motor.get("last_rx_s"),
                "safety": status["safety_state"],
                "theta": theta,
                "omega": state.get("omega_rad_s"),
                "requested": control.get("requested_velocity_mm_s"),
                "limited": control.get("limited_velocity_mm_s"),
                "motor_target": motor.get("target_velocity_mm_s"),
                "motor_velocity": motor.get("velocity_mm_s"),
                "position": motor.get("position_mm"),
                "reason": control.get("reason"),
                "mode": status.get("swing_up", {}).get("mode"),
                "energy_error": status.get("swing_up", {}).get("energy_error"),
                "predicted_theta": status.get("swing_up", {}).get("predicted_theta_rad"),
                "predicted_omega": status.get("swing_up", {}).get("predicted_omega_rad_s"),
            }
            samples.append(sample)
            if args.swing_up and sample["mode"] == "balance":
                if not poll_gap_ok:
                    stable_since = None
                if verified_balance_observation(status, sampled_s):
                    if stable_since is None:
                        stable_since = sampled_s
                    stable_elapsed = sampled_s - stable_since
                    best_stable_s = max(best_stable_s, stable_elapsed)
                    if stable_elapsed >= hold_s:
                        stabilized = True
                        abort_reason = f"swing-up captured and balanced for {hold_s:g} seconds"
                        break
                    deadline = min(hard_deadline, max(deadline, stable_since + hold_s + 0.1))
                else:
                    stable_since = None
                    deadline = min(hard_deadline, max(deadline, sampled_s + hold_s + 0.1))
            else:
                stable_since = None
            if status["safety_state"] != "RUNNING":
                abort_reason = f"safety left RUNNING: {status['safety_state']} ({status['safety_reason']})"
                break
            if not state.get("valid") or theta is None or not math.isfinite(float(theta)):
                abort_reason = "vision state invalid"
                break
            if not args.swing_up and abs(float(theta)) > ABORT_ANGLE_RAD:
                abort_reason = f"angle abort at {float(theta):.4f} rad"
                break
            if abs(float(motor["position_mm"]) - float(initial["motor"]["position_mm"])) > displacement_limit:
                abort_reason = (
                    "cart displacement abort at "
                    f"{float(motor['position_mm']) - float(initial['motor']['position_mm']):.1f} mm"
                )
                break
            if status["tracking"]["status"] != "ok":
                abort_reason = f"tracking={status['tracking']['status']}"
                break
            if motor["fault"] or motor["left_limit"] or motor["right_limit"]:
                abort_reason = f"motor fault/limit: {motor['fault']}"
                break
            if not status["system_health"]["safe_for_control"]:
                abort_reason = "Pi power became unsafe"
                break
            time.sleep(0.05)
    finally:
        if control_started or control_armed:
            try:
                request("/api/control/stop", {})
            except Exception as exc:
                abort_reason += f"; stop error: {exc}"
        if experiment_started:
            try:
                summary = request("/api/experiments/finish", {})
            except Exception as exc:
                abort_reason += f"; finish error: {exc}"

    stop_deadline = time.monotonic() + 2.0
    final = request("/api/status")
    while (
        abs(float(final["motor"]["target_velocity_mm_s"])) > 0.05
        or abs(float(final["motor"]["velocity_mm_s"])) > 0.05
    ) and time.monotonic() < stop_deadline:
        time.sleep(0.05)
        final = request("/api/status")
    active = [s for s in samples if s["theta"] is not None]
    output = {
        "abort_reason": abort_reason,
        "sample_count": len(samples),
        "trial_type": "energy swing-up" if args.swing_up else "upright direction",
        "swing_up_settings": initial.get("swing_up"),
        "observed_modes": sorted({s["mode"] for s in samples if s["mode"]}),
        "stabilization_success": stabilized if args.swing_up else None,
        "balance_hold_target_s": hold_s if args.swing_up else None,
        "observed_continuous_stable_s": best_stable_s if args.swing_up else None,
        "hard_runtime_limit_s": max_run_s,
        "max_abs_theta_rad": max((abs(float(s["theta"])) for s in active), default=None),
        "max_abs_requested_mm_s": max((abs(float(s["requested"] or 0.0)) for s in samples), default=None),
        "max_abs_motor_velocity_mm_s": max((abs(float(s["motor_velocity"] or 0.0)) for s in samples), default=None),
        "start_position_mm": initial["motor"]["position_mm"],
        "end_position_mm": final["motor"]["position_mm"],
        "final_safety": final["safety_state"],
        "final_motor_target_mm_s": final["motor"]["target_velocity_mm_s"],
        "final_motor_velocity_mm_s": final["motor"]["velocity_mm_s"],
        "experiment_id": None if summary is None else summary.get("experiment_id"),
        "max_deviation_from_down_deg": max(
            (180.0 - abs(math.degrees(math.atan2(math.sin(float(s["theta"])), math.cos(float(s["theta"]))))))
            for s in active
        ) if active else None,
        "first_samples": samples[:5],
        "last_samples": samples[-5:],
    }
    if output["experiment_id"]:
        monitor = Path("experiments/logs") / f"{output['experiment_id']}.monitor.json"
        try:
            monitor.parent.mkdir(parents=True, exist_ok=True)
            monitor.write_text(json.dumps({"result": output, "samples": samples}, indent=2),
                               encoding="utf-8")
            output["monitor_file"] = str(monitor)
        except OSError as exc:
            output["monitor_error"] = str(exc)
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
