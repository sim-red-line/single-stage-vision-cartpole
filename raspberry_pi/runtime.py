"""Long-running camera, control, safety, and experiment-recording orchestration."""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path
from typing import Any

import cv2

from .calibration import (
    CalibrationProfile,
    CoordinateCalibration,
    HSVRange,
    MarkerColorProfile,
    ProfileStore,
    calibrate_marker_from_roi,
    frame_brightness,
    new_profile,
)
from .camera import CameraCapture, CameraError, FramePacket, encode_jpeg
from .color_tracker import ColorMarkerTracker
from .config import project_root
from .data_logger import ExperimentLogger
from .models import (
    CartPoleState,
    ControlOutput,
    MarkerRole,
    PIDGains,
    RuntimeSnapshot,
    TrackingFrame,
    TrackingStatus,
)
from .motor_interface import DisconnectedMotorInterface, MotorInterface
from .pid_controller import HybridSwingUpController
from .safety import SafetyManager, SafetyState
from .state_estimator import StateEstimator
from .system_health import PiPowerMonitor, PiPowerStatus
from .video_recorder import ExperimentVideoRecorder


class CartPoleRuntime:
    def __init__(self, config: dict[str, Any], root: str | Path | None = None) -> None:
        self.config = config
        self.root = Path(root) if root else project_root()
        self.profile_store = ProfileStore(self.root / "calibration" / "profiles")
        self.logger = ExperimentLogger(
            self.root / "experiments" / "logs",
            self.root / "experiments" / "summaries",
            config["logging"],
        )
        self.video_recorder = ExperimentVideoRecorder(
            self.root / "experiments" / "videos", config["logging"]
        )
        self.gains = PIDGains.from_mapping(config["controller"]["gains"])
        self.controller = HybridSwingUpController(config["controller"], self.gains)
        self.safety = SafetyManager(config["safety"])
        self.motor: MotorInterface | DisconnectedMotorInterface
        if bool(config["hardware"].get("enabled", False)):
            self.motor = MotorInterface(config["hardware"])
        else:
            self.motor = DisconnectedMotorInterface()
        self.camera = CameraCapture(config["camera"])
        power_required = bool(config["hardware"].get("enabled", False)) and bool(
            config["safety"].get("require_clean_pi_power", True)
        )
        self.power_monitor = PiPowerMonitor(required=power_required)
        self._power_status: PiPowerStatus = self.power_monitor.status

        self._lock = threading.RLock()
        self._configuration_lock = threading.RLock()
        self._controller_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._camera_thread: threading.Thread | None = None
        self._control_thread: threading.Thread | None = None
        self._health_thread: threading.Thread | None = None
        self._started = False
        self._latest_raw_frame: FramePacket | None = None
        self._latest_jpeg: bytes | None = None
        self._latest_mask_jpeg: dict[str, bytes] = {}
        self._latest_tracking: TrackingFrame | None = None
        self._latest_state = CartPoleState(
            0.0, None, None, None, None, False, TrackingStatus.LOST, reason="no camera sample"
        )
        self._latest_control = ControlOutput(
            0.0, 0.0, 0.0, 0.0, active=False, reason="controller idle"
        )
        self._recent = deque(maxlen=1800)
        self._camera_error = ""
        self._control_error = ""
        self._video_error = ""
        self._fps = 0.0
        self._control_latency_ms = 0.0
        self._last_camera_timestamp: float | None = None
        self._last_dashboard_encode_s = 0.0
        self._last_mask_encode_s = 0.0
        self._verification_good_frames = 0
        self._calibration_verified = False
        self.profile: CalibrationProfile | None = None
        self.tracker: ColorMarkerTracker | None = None
        self.estimator: StateEstimator | None = None
        self._staged_markers: dict[str, MarkerColorProfile] = {}
        self._last_summary: dict[str, Any] | None = None
        self._last_motor_command: float | None = None
        self._gain_source = "config"
        self._recalibration_count = 0
        self._experiment_swing_settings: dict[str, Any] = {}

        requested_profile = str(config["tracking"].get("profile", "default"))
        if requested_profile in self.profile_store.list_profiles():
            self.load_profile(requested_profile)

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._stop_event.clear()
        self._power_status = self.power_monitor.poll()
        self.safety.update_prerequisites(
            power_ok=self._power_status.safe_for_control,
            power_reason=self._power_status.reason,
        )
        if self._power_status.required and not self._power_status.safe_for_control:
            self._control_error = f"motor connection blocked: {self._power_status.reason}"
        else:
            try:
                self.motor.connect()
            except Exception as exc:
                self._control_error = f"motor connection failed: {exc}"
        self._camera_thread = threading.Thread(
            target=self._camera_loop, name="camera-processing", daemon=True
        )
        self._control_thread = threading.Thread(
            target=self._control_loop, name="pid-control", daemon=True
        )
        self._health_thread = threading.Thread(
            target=self._health_loop, name="pi-power-monitor", daemon=True
        )
        self._health_thread.start()
        self._camera_thread.start()
        self._control_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self.operator_stop("runtime shutdown")
        if self.logger.active is not None:
            try:
                video = asdict(self.video_recorder.finish())
                self._last_summary = self.logger.finish(
                    {
                        "aborted": True,
                        "abort_reason": "runtime shutdown",
                        "video": video,
                    }
                )
            except Exception:
                pass
        if self._camera_thread:
            self._camera_thread.join(timeout=2.0)
        if self._control_thread:
            self._control_thread.join(timeout=2.0)
        if self._health_thread:
            self._health_thread.join(timeout=2.0)
        self.camera.close()
        self.motor.close()
        self.logger.close()
        self.video_recorder.close()
        self._started = False

    def _health_loop(self) -> None:
        interval = max(0.25, float(self.config["safety"].get("power_poll_s", 1.0)))
        while not self._stop_event.wait(interval):
            status = self.power_monitor.poll()
            with self._lock:
                self._power_status = status
            if status.required and not status.safe_for_control:
                self.safety.fault(f"unsafe Pi power: {status.reason}")
                try:
                    self.motor.stop()
                except Exception:
                    pass
                self._last_motor_command = 0.0

    # ------------------------------------------------------------------ camera
    def _camera_loop(self) -> None:
        camera_open = False
        while not self._stop_event.is_set():
            if not bool(self.config["camera"].get("enabled", True)):
                self._camera_error = "camera disabled in config"
                self._stop_event.wait(0.5)
                continue
            if not camera_open:
                try:
                    self.camera.open()
                    camera_open = True
                    self._camera_error = ""
                except Exception as exc:
                    self._camera_error = str(exc)
                    self._stop_event.wait(2.0)
                    continue
            try:
                packet = self.camera.read()
            except CameraError as exc:
                self._camera_error = str(exc)
                self.camera.close()
                camera_open = False
                self._stop_event.wait(0.2)
                continue
            with self._lock:
                self._latest_raw_frame = packet
                tracker = self.tracker
                estimator = self.estimator
            self.video_recorder.append(packet.image)
            if self._last_camera_timestamp is not None:
                instant_fps = 1.0 / max(1e-6, packet.timestamp_s - self._last_camera_timestamp)
                self._fps = instant_fps if self._fps == 0.0 else 0.15 * instant_fps + 0.85 * self._fps
            self._last_camera_timestamp = packet.timestamp_s
            dashboard_due = (
                packet.timestamp_s - self._last_dashboard_encode_s
                >= 1.0 / max(1.0, float(self.config["camera"].get("dashboard_fps", 20)))
            )
            mask_due = (
                packet.timestamp_s - self._last_mask_encode_s
                >= 1.0 / max(0.5, float(self.config["camera"].get("mask_preview_fps", 5)))
            )

            if tracker is None or estimator is None:
                jpeg = None
                mask_previews = None
                if dashboard_due:
                    annotated = packet.image.copy()
                    cv2.putText(
                        annotated,
                        "CALIBRATION REQUIRED",
                        (20, 42),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    jpeg = encode_jpeg(
                        annotated, int(self.config["camera"].get("jpeg_quality", 78))
                    )
                    self._last_dashboard_encode_s = packet.timestamp_s
                if mask_due:
                    mask_previews = self._staged_mask_previews(packet.image)
                    self._last_mask_encode_s = packet.timestamp_s
                with self._lock:
                    if jpeg is not None:
                        self._latest_jpeg = jpeg
                    if mask_previews is not None:
                        self._latest_mask_jpeg = mask_previews
                continue
            try:
                tracking = tracker.process(packet.image, packet.timestamp_s, packet.sequence)
                state = estimator.update(tracking)
            except Exception as exc:
                self._camera_error = f"vision pipeline error: {exc}"
                self._verification_good_frames = 0
                self._calibration_verified = False
                continue

            if (
                self.profile is not None
                and self.profile.structurally_valid
                and tracking.status is TrackingStatus.OK
                and state.valid
            ):
                self._verification_good_frames += 1
            else:
                self._verification_good_frames = 0
            self._calibration_verified = self._verification_good_frames >= 15
            jpeg = None
            mask_jpegs = None
            if dashboard_due:
                annotated = tracker.annotate(packet.image, tracking)
                jpeg = encode_jpeg(
                    annotated, int(self.config["camera"].get("jpeg_quality", 78))
                )
                self._last_dashboard_encode_s = packet.timestamp_s
            if mask_due:
                masks = tracker.masks(packet.image)
                mask_jpegs = {
                    name: encode_jpeg(mask, 80) for name, mask in masks.items()
                }
                self._last_mask_encode_s = packet.timestamp_s
            with self._lock:
                self._latest_tracking = tracking
                self._latest_state = state
                if jpeg is not None:
                    self._latest_jpeg = jpeg
                if mask_jpegs is not None:
                    self._latest_mask_jpeg = mask_jpegs

    # ---------------------------------------------------------------- control
    def _control_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.config["controller"].get("loop_hz", 100)))
        next_run = time.monotonic()
        was_running = False
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now < next_run:
                self._stop_event.wait(next_run - now)
                continue
            next_run += period
            if now - next_run > period * 4:
                next_run = now + period

            motor_status = self.motor.status
            with self._lock:
                state = self._latest_state
                tracking = self._latest_tracking
                calibrated = self._calibration_verified
                power_status = self._power_status
            tracking_ok = bool(state.valid and tracking and tracking.status is TrackingStatus.OK)
            control_link_ok = bool(motor_status.connected and motor_status.watchdog_ok)
            if motor_status.connected and motor_status.estop:
                self.safety.estop("RP2040 reports ESTOP")
            elif motor_status.connected and motor_status.fault:
                self.safety.fault(f"RP2040 fault: {motor_status.fault}")
            self.safety.update_prerequisites(
                connected=control_link_ok,
                homed=motor_status.homed,
                calibrated=calibrated,
                tracking_ok=tracking_ok,
                power_ok=power_status.safe_for_control,
                power_reason=power_status.reason,
            )
            running = self.safety.state is SafetyState.RUNNING
            if running:
                if not tracking_ok:
                    control = ControlOutput(
                        now,
                        0.0,
                        0.0,
                        0.0,
                        active=False,
                        reason="tracking is not fully reliable",
                    )
                    self._record_sample(now, state, control, tracking, motor_status)
                    self.safety.fault("tracking confidence lost during control")
                    running = False
                elif state.theta_rad is None or (
                    not self.controller.enabled
                    and abs(state.theta_rad)
                    > float(self.config["controller"]["fail_angle_rad"])
                ):
                    control = ControlOutput(
                        now,
                        0.0,
                        0.0,
                        0.0,
                        active=False,
                        reason="fail angle exceeded",
                    )
                    self._record_sample(now, state, control, tracking, motor_status)
                    self.safety.fault("pendulum exceeded configured fail angle")
                    running = False
                elif (
                    self.controller.enabled
                    and state.omega_rad_s is not None
                    and abs(state.omega_rad_s) > self.controller.max_omega
                ):
                    control = ControlOutput(
                        now,
                        0.0,
                        0.0,
                        0.0,
                        active=False,
                        reason="swing-up angular-speed limit exceeded",
                    )
                    self._record_sample(now, state, control, tracking, motor_status)
                    self.safety.fault("swing-up angular-speed limit exceeded")
                    running = False
                else:
                    with self._controller_lock:
                        control = self.controller.update(state, now)
                        controller_fault = self.controller.fault_reason
                        motion_bounds = (
                            (self.controller.cart_center - self.controller.max_excursion,
                             self.controller.cart_center + self.controller.max_excursion)
                            if self.controller.enabled else None
                        )
                    if controller_fault:
                        self._control_error = controller_fault
                        self._record_sample(now, state, control, tracking, motor_status)
                        self.safety.fault(controller_fault)
                    decision = self.safety.evaluate_velocity(
                        control.limited_velocity_mm_s, state, now,
                        motion_bounds_mm=motion_bounds,
                    )
                    control.limited_velocity_mm_s = decision.allowed_velocity_mm_s
                    with self._controller_lock:
                        self.controller.accept_applied_velocity(decision.allowed_velocity_mm_s)
                    if decision.stop_required:
                        running = False
                    else:
                        try:
                            self.motor.set_velocity(decision.allowed_velocity_mm_s)
                            self._last_motor_command = decision.allowed_velocity_mm_s
                            self._control_error = ""
                        except Exception as exc:
                            self._control_error = str(exc)
                            self.safety.fault(f"motor command failed: {exc}")
                            running = False
                    with self._lock:
                        self._latest_control = control
                        self._control_latency_ms = max(0.0, (now - state.timestamp_s) * 1000.0)
                    if running:
                        self._record_sample(now, state, control, tracking, motor_status)
            if not running:
                if was_running or (self._last_motor_command is not None and abs(self._last_motor_command) > 0.0):
                    try:
                        self.motor.stop()
                    except Exception:
                        pass
                    self._last_motor_command = 0.0
                with self._controller_lock:
                    # Keep internal swing-up faults visible until explicitly cleared.
                    if not self.controller.fault_reason:
                        self.controller.reset(now)
                with self._lock:
                    self._latest_control = ControlOutput(
                        now,
                        0.0,
                        0.0,
                        0.0,
                        active=False,
                        reason=self.safety.reason,
                    )
            was_running = running

    def _record_sample(
        self,
        now: float,
        state: CartPoleState,
        control: ControlOutput,
        tracking: TrackingFrame | None,
        motor_status: Any,
    ) -> None:
        confidence = 0.0
        if tracking and tracking.detections:
            required = [
                detection.confidence
                for detection in tracking.detections.values()
                if detection.valid
            ]
            confidence = min(required) if required else 0.0
        sample = {
            "timestamp_monotonic_s": now,
            "theta_rad": state.theta_rad,
            "omega_rad_s": state.omega_rad_s,
            "cart_position_mm": state.cart_position_mm,
            "cart_velocity_mm_s": state.cart_velocity_mm_s,
            "target_angle_rad": control.target_angle_rad,
            "requested_velocity_mm_s": control.requested_velocity_mm_s,
            "limited_velocity_mm_s": control.limited_velocity_mm_s,
            "angle_p": control.angle_p,
            "angle_i": control.angle_i,
            "angle_d": control.angle_d,
            "position_p": control.position_p,
            "position_i": control.position_i,
            "position_d": control.position_d,
            "output_saturated": control.saturated,
            "tracking_valid": bool(
                state.valid and tracking and tracking.status is TrackingStatus.OK
            ),
            "tracking_status": str(state.tracking_status),
            "tracking_confidence": confidence,
            "processing_ms": tracking.processing_ms if tracking else None,
            "fps": self._fps,
            "control_latency_ms": self._control_latency_ms,
            "motor_position_mm": motor_status.position_mm,
            "motor_velocity_mm_s": motor_status.velocity_mm_s,
            "safety_state": str(self.safety.state),
        }
        self.logger.append(sample)
        recent = dict(sample)
        active = self.logger.active
        recent["elapsed_s"] = (
            now - active.started_monotonic_s if active else now
        )
        with self._lock:
            self._recent.append(recent)

    # ----------------------------------------------------------- calibration
    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._latest_jpeg

    def latest_mask_jpeg(self, name: str) -> bytes | None:
        with self._lock:
            return self._latest_mask_jpeg.get(name)

    def sample_marker(
        self,
        name: str,
        role: MarkerRole,
        roi: tuple[int, int, int, int],
        required: bool = True,
    ) -> dict[str, Any]:
        with self._lock:
            packet = self._latest_raw_frame
        if packet is None:
            raise RuntimeError("no camera frame is available")
        marker = calibrate_marker_from_roi(
            packet.image, roi, name=name, role=role, required=required
        )
        with self._lock:
            self._staged_markers[name] = marker
        previews = self._staged_mask_previews(packet.image)
        with self._lock:
            self._latest_mask_jpeg = previews
        return self._marker_dict(marker)

    def _staged_mask_previews(self, frame: Any) -> dict[str, bytes]:
        with self._lock:
            markers = list(self._staged_markers.values())
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel_size = max(1, int(self.config["tracking"].get("morphology_kernel", 5)))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        previews: dict[str, bytes] = {}
        for marker in markers:
            mask = None
            for hsv_range in marker.hsv_ranges:
                current = cv2.inRange(
                    hsv,
                    tuple(int(value) for value in hsv_range.lower),
                    tuple(int(value) for value in hsv_range.upper),
                )
                mask = current if mask is None else cv2.bitwise_or(mask, current)
            if mask is not None:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                previews[marker.name] = encode_jpeg(mask, 80)
        return previews

    def set_marker_ranges(
        self, name: str, ranges: list[dict[str, list[int]]], min_area: float, max_area: float
    ) -> dict[str, Any]:
        if not ranges:
            raise ValueError("at least one HSV range is required")
        checked_ranges: list[HSVRange] = []
        for item in ranges:
            lower, upper = item.get("lower"), item.get("upper")
            if (
                not isinstance(lower, list)
                or not isinstance(upper, list)
                or len(lower) != 3
                or len(upper) != 3
            ):
                raise ValueError("each HSV range needs three lower and upper values")
            if any(not isinstance(value, int) for value in lower + upper):
                raise ValueError("HSV bounds must be integers")
            limits = (179, 255, 255)
            if any(
                not 0 <= lower[index] <= upper[index] <= limits[index]
                for index in range(3)
            ):
                raise ValueError("HSV bounds are out of range or reversed")
            checked_ranges.append(HSVRange(tuple(lower), tuple(upper)))
        if not 0.0 < min_area < max_area:
            raise ValueError("marker area requires 0 < min_area < max_area")
        packet: FramePacket | None = None
        with self._lock:
            marker = self._staged_markers.get(name)
            if marker is None:
                raise KeyError(name)
            marker.hsv_ranges = checked_ranges
            marker.min_area_px = float(min_area)
            marker.max_area_px = float(max_area)
            packet = self._latest_raw_frame
            result = self._marker_dict(marker)
        if packet is not None:
            previews = self._staged_mask_previews(packet.image)
            with self._lock:
                self._latest_mask_jpeg = previews
        return result

    @staticmethod
    def _marker_dict(marker: MarkerColorProfile) -> dict[str, Any]:
        return {
            "name": marker.name,
            "role": str(marker.role),
            "hsv_ranges": [asdict(item) for item in marker.hsv_ranges],
            "min_area_px": marker.min_area_px,
            "max_area_px": marker.max_area_px,
            "sample_hsv_median": marker.sample_hsv_median,
            "required": marker.required,
        }

    def staged_markers(self) -> list[dict[str, Any]]:
        with self._lock:
            return [self._marker_dict(marker) for marker in self._staged_markers.values()]

    def save_profile(
        self,
        name: str,
        coordinates: dict[str, Any],
        notes: str = "",
    ) -> dict[str, Any]:
        with self._lock:
            packet = self._latest_raw_frame
            markers = list(self._staged_markers.values())
        if packet is None:
            raise RuntimeError("no camera frame is available")
        if not markers:
            raise RuntimeError("calibrate at least one marker before saving")
        coordinate_calibration = CoordinateCalibration(**coordinates)
        if not coordinate_calibration.valid:
            raise ValueError("rail endpoints, length, direction, or homography are invalid")
        if coordinate_calibration.homography is not None:
            matrix = coordinate_calibration.homography
            if (
                len(matrix) != 3
                or any(len(row) != 3 for row in matrix)
                or any(
                    not math.isfinite(float(value))
                    for row in matrix
                    for value in row
                )
            ):
                raise ValueError("homography must be a finite 3x3 matrix")
        profile = new_profile(
            name,
            markers,
            coordinate_calibration,
            frame_brightness(
                packet.image, self.config["tracking"].get("brightness_roi_fraction")
            ),
            notes,
        )
        path = self.profile_store.save(profile)
        self._recalibration_count += 1
        self.load_profile(name)
        return {
            "path": str(path),
            "structurally_valid": profile.structurally_valid,
            "profile": profile.to_dict(),
        }

    def load_profile(self, name: str) -> None:
        profile = self.profile_store.load(name)
        tracker = ColorMarkerTracker(profile, self.config["tracking"])
        estimator = StateEstimator(profile.coordinates, self.config["estimator"])
        with self._lock:
            self.profile = profile
            self.tracker = tracker
            self.estimator = estimator
            self._staged_markers = {marker.name: marker for marker in profile.markers}
            self._calibration_verified = False
            self._verification_good_frames = 0
        self.safety.update_prerequisites(calibrated=False)

    # -------------------------------------------------------------- operators
    def home(self) -> None:
        if self.safety.state in {SafetyState.RUNNING, SafetyState.ARMED}:
            raise RuntimeError("stop/disarm before homing")
        self.motor.home()

    def arm(self) -> bool:
        with self._configuration_lock:
            return self.safety.arm()

    def begin_control(self) -> bool:
        with self._configuration_lock:
            output_mode = str(self.config["controller"].get("output_mode", "velocity"))
            if (
                output_mode == "acceleration"
                and (not math.isfinite(self.gains.kp_angle) or self.gains.kp_angle <= 9806.65)
            ):
                raise RuntimeError(
                    "acceleration-mode balance requires angle Kp above gravity "
                    "(9806.65 mm/s^2/rad)"
                )
            if self.controller.enabled:
                if output_mode != "acceleration":
                    raise RuntimeError("energy swing-up capture requires acceleration output mode")
                if any(
                    not math.isfinite(value) or value <= 0
                    for value in (self.gains.kp_angle, self.gains.kd_angle)
                ):
                    raise RuntimeError(
                        "energy swing-up requires approved nonzero angle Kp and Kd gains for capture"
                    )
                motor = self.motor.status
                if abs(motor.velocity_mm_s) > 0.05 or abs(motor.target_velocity_mm_s) > 0.05:
                    raise RuntimeError("stop the cart before starting energy swing-up")
            with self._controller_lock:
                self.controller.reset(time.monotonic())
                return self.safety.start()

    def operator_stop(self, reason: str = "operator stop") -> None:
        self.safety.stop(reason)
        try:
            self.motor.stop()
        except Exception:
            pass

    def configure_swing_up(self, parameters: dict[str, float]) -> dict[str, Any]:
        """Change only bounded energy parameters between stationary trials.

        This does not ARM/START, change PID gains, or change any speed, rail,
        acceleration, watchdog or tracking safety limit.
        """
        bounds = {
            "energy_gain": (0.1, 64.0),
            "energy_target_margin": (0.0, 0.05),
            "energy_rate_lag_s": (0.0, 0.06),
            "kick_acceleration_mm_s2": (0.0, self.controller.max_acceleration),
            "kick_duration_s": (0.0, 1.0),
            "max_duration_s": (0.5, 30.0),
            "cart_position_gain_s2": (0.0, 5.0),
            "cart_velocity_gain_s": (0.0, 5.0),
        }
        if not parameters or set(parameters) - set(bounds):
            raise ValueError("only bounded swing-up energy parameters may be changed")
        checked = {}
        for name, value in parameters.items():
            low, high = bounds[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a number")
            value = float(value)
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be in [{low:g}, {high:g}]")
            checked[name] = value
        with self._configuration_lock:
            if self.safety.state is not SafetyState.IDLE or self.logger.active is not None:
                raise RuntimeError("finish the experiment and stop/disarm before changing energy parameters")
            motor = self.motor.status
            if (not motor.connected or not motor.homed or motor.estop or motor.fault
                    or motor.left_limit or motor.right_limit
                    or abs(motor.velocity_mm_s) > 0.05
                    or abs(motor.target_velocity_mm_s) > 0.05
                    or not self._power_status.safe_for_control):
                raise RuntimeError("stationary, homed, fault-free motor and clean power required")
            controller_config = dict(self.config["controller"])
            swing_config = dict(controller_config.get("swing_up", {}))
            swing_config.update(checked)
            controller_config["swing_up"] = swing_config
            candidate = HybridSwingUpController(controller_config, self.gains)
            with self._controller_lock:
                self.controller = candidate
                self.config["controller"] = controller_config
                return self.controller.snapshot()

    def estop(self, reason: str = "operator emergency stop") -> None:
        self.safety.estop(reason)
        try:
            self.motor.estop()
        finally:
            self._last_motor_command = 0.0

    def clear_estop(self) -> bool:
        motor_status = self.motor.status
        clear = not motor_status.left_limit and not motor_status.right_limit
        if not clear:
            self.safety.clear_estop(False)
            return False
        self.motor.clear_estop()
        return self.safety.clear_estop(True)

    def clear_fault(self) -> bool:
        status = self.motor.status
        if status.connected:
            self.motor.clear_fault()
        cleared = self.safety.clear_fault()
        if cleared:
            with self._controller_lock:
                self.controller.reset(time.monotonic())
        return cleared

    # ------------------------------------------------------------ experiments
    def start_experiment(self, notes: str = "") -> dict[str, Any]:
        with self._configuration_lock:
            if self.safety.state not in {SafetyState.ARMED, SafetyState.RUNNING}:
                raise RuntimeError("arm the system before starting an experiment log")
            with self._controller_lock:
                self._experiment_swing_settings = self.controller.snapshot()
            active = self.logger.start(self.gains.as_dict(), notes)
            with self._lock:
                packet = self._latest_raw_frame
            if self.video_recorder.enabled and packet is not None:
                height, width = packet.image.shape[:2]
                recording_fps = (
                    self._fps
                    if 1.0 <= self._fps <= 120.0
                    else float(self.config["camera"].get("fps", 30))
                )
                try:
                    result = self.video_recorder.start(
                        active.experiment_id,
                        (width, height),
                        max(1.0, recording_fps),
                    )
                    self._video_error = result.error or ""
                except Exception as exc:
                    self._video_error = f"video recording could not start: {exc}"
                    self.video_recorder.mark_unavailable(self._video_error)
            elif self.video_recorder.enabled:
                self._video_error = "no camera frame was available when recording started"
                self.video_recorder.mark_unavailable(self._video_error)
            return asdict(active)

    def finish_experiment(self, stop_controller: bool = True) -> dict[str, Any]:
        with self._configuration_lock:
            if stop_controller:
                self.operator_stop("experiment complete")
            video = asdict(self.video_recorder.finish())
            if self._video_error and not video.get("error"):
                video["error"] = self._video_error
            summary = self.logger.finish(
                {
                    "calibration_profile": self.profile.name if self.profile else None,
                    "recalibration_count": self._recalibration_count,
                    "gain_source": self._gain_source,
                    "swing_up_settings": dict(self._experiment_swing_settings),
                    "video": video,
                    "safety_state": str(self.safety.state),
                    "safety_reason": self.safety.reason,
                    "camera_error": self._camera_error or None,
                    "control_error": self._control_error or None,
                }
            )
            self._last_summary = summary
            return summary

    def experiment_summaries(self, limit: int = 50) -> list[dict[str, Any]]:
        paths = sorted(
            (self.root / "experiments" / "summaries").glob("*.json"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )[:limit]
        return [json.loads(path.read_text(encoding="utf-8")) for path in paths]

    def get_summary(self, experiment_id: str) -> dict[str, Any]:
        if not experiment_id or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for c in experiment_id):
            raise ValueError("invalid experiment id")
        path = self.root / "experiments" / "summaries" / f"{experiment_id}.json"
        return json.loads(path.read_text(encoding="utf-8"))

    def latest_summary(self) -> dict[str, Any]:
        if self._last_summary:
            return self._last_summary
        summaries = self.experiment_summaries(limit=1)
        if not summaries:
            raise RuntimeError("at least one completed experiment is required")
        return summaries[0]

    # ---------------------------------------------------------------- status
    def recent_series(self, limit: int = 600) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._recent)[-max(1, min(limit, 1800)) :]

    def snapshot(self) -> dict[str, Any]:
        motor = self.motor.status
        with self._lock:
            state = self._latest_state
            tracking = self._latest_tracking
            control = self._latest_control
            profile = self.profile
            calibration = {
                "profile": profile.name if profile else None,
                "structurally_valid": profile.structurally_valid if profile else False,
                "live_verified": self._calibration_verified,
                "verification_frames": self._verification_good_frames,
                "available_profiles": self.profile_store.list_profiles(),
                "camera_error": self._camera_error,
            }
            tracking_data: dict[str, Any] = {
                "status": str(tracking.status) if tracking else "unavailable",
                "reason": tracking.reason if tracking else "no tracking result",
                "brightness": tracking.mean_brightness if tracking else None,
                "processing_ms": tracking.processing_ms if tracking else None,
                "markers": {
                    name: {
                        "role": str(detection.role),
                        "center_px": detection.center_px,
                        "confidence": detection.confidence,
                        "status": str(detection.status),
                    }
                    for name, detection in (tracking.detections.items() if tracking else [])
                },
            }
            experiment = {
                "active": asdict(self.logger.active) if self.logger.active else None,
                "dropped_rows": self.logger.dropped_rows,
                "latest_summary": (
                    {
                        key: value
                        for key, value in self._last_summary.items()
                        if key != "representative_series"
                    }
                    if self._last_summary
                    else None
                ),
                "video": asdict(self.video_recorder.status()),
            }
        snapshot = RuntimeSnapshot(
            timestamp_s=time.monotonic(),
            safety_state=str(self.safety.state),
            safety_reason=self.safety.reason,
            tracking=tracking_data,
            state=state.as_dict(),
            control=asdict(control),
            motor=motor.as_dict(),
            calibration=calibration,
            experiment=experiment,
            gain_source=self._gain_source,
            gains=self.gains.as_dict(),
            fps=self._fps,
            control_latency_ms=self._control_latency_ms,
        )
        result = asdict(snapshot)
        result["errors"] = {
            "camera": self._camera_error or None,
            "control": self._control_error or None,
        }
        result["system_health"] = self._power_status.as_dict()
        with self._controller_lock:
            result["swing_up"] = self.controller.snapshot()
        result["controller_config"] = {
            "output_mode": str(
                self.config["controller"].get("output_mode", "velocity")
            ),
            "max_velocity_mm_s": float(
                self.config["controller"]["max_velocity_mm_s"]
            ),
            "max_acceleration_mm_s2": float(
                self.config["controller"]["max_acceleration_mm_s2"]
            ),
        }
        return result

