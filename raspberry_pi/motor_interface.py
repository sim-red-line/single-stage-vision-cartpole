"""Thread-safe USB serial client for the RP2040 motion controller."""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Callable


@dataclass(slots=True)
class MotorStatus:
    connected: bool = False
    homed: bool = False
    estop: bool = False
    fault: str = ""
    position_mm: float | None = None
    velocity_mm_s: float = 0.0
    target_velocity_mm_s: float = 0.0
    left_limit: bool = False
    right_limit: bool = False
    watchdog_ok: bool = False
    rail_length_mm: float | None = None
    last_rx_s: float = 0.0
    raw: dict[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class MotorProtocolError(RuntimeError):
    pass


def parse_status_line(line: str) -> dict[str, Any] | None:
    text = line.strip()
    if text.startswith("STATUS:"):
        text = text[len("STATUS:") :]
    if not text.startswith("{"):
        return None
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as exc:
        raise MotorProtocolError(f"malformed status JSON: {exc}") from exc
    if not isinstance(parsed, dict):
        raise MotorProtocolError("status payload must be an object")
    return parsed


class MotorInterface:
    def __init__(
        self,
        config: dict[str, Any],
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.config = config
        self._serial_factory = serial_factory
        self._serial: Any | None = None
        self._status = MotorStatus()
        self._status_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_heartbeat_s = 0.0
        self._last_status_query_s = 0.0

    def connect(self) -> None:
        if self._serial is not None:
            return
        factory = self._serial_factory
        if factory is None:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("pyserial is required for hardware operation") from exc
            factory = serial.Serial
        serial_port = factory(
            self.config["serial_port"],
            int(self.config.get("baudrate", 115200)),
            timeout=float(self.config.get("serial_timeout_s", 0.1)),
        )
        # The RP2040 streams replies continuously.  Reopening the port can
        # otherwise begin in the middle of a STATUS JSON line and latch a host
        # safety fault even though the following complete status is healthy.
        reset_input = getattr(serial_port, "reset_input_buffer", None)
        if callable(reset_input):
            reset_input()
        self._serial = serial_port
        self._stop_event.clear()
        with self._status_lock:
            self._status.connected = True
            self._status.last_rx_s = time.monotonic()
        self._thread = threading.Thread(target=self._io_loop, name="rp2040-serial", daemon=True)
        self._thread.start()
        self.status_query()

    def close(self) -> None:
        self._stop_event.set()
        try:
            self.stop()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        serial_port, self._serial = self._serial, None
        if serial_port is not None:
            serial_port.close()
        with self._status_lock:
            self._status.connected = False

    def _write(self, command: str) -> None:
        serial_port = self._serial
        if serial_port is None:
            raise ConnectionError("RP2040 is not connected")
        payload = (command.rstrip() + "\n").encode("ascii", errors="strict")
        with self._write_lock:
            serial_port.write(payload)

    def _io_loop(self) -> None:
        heartbeat_period = float(self.config.get("heartbeat_period_s", 0.1))
        while not self._stop_event.is_set():
            now = time.monotonic()
            try:
                if now - self._last_heartbeat_s >= heartbeat_period:
                    self._write("HEARTBEAT")
                    self._last_heartbeat_s = now
                if now - self._last_status_query_s >= 0.25:
                    self._write("STATUS?")
                    self._last_status_query_s = now
                serial_port = self._serial
                raw = serial_port.readline() if serial_port is not None else b""
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    self._consume_line(line, now)
            except Exception as exc:
                with self._status_lock:
                    self._status.connected = False
                    self._status.fault = f"serial error: {exc}"
                self._stop_event.wait(0.05)

    def _consume_line(self, line: str, now_s: float | None = None) -> None:
        now_s = time.monotonic() if now_s is None else now_s
        parsed = parse_status_line(line)
        with self._status_lock:
            self._status.last_rx_s = now_s
            self._status.connected = True
            if parsed is None:
                if line.startswith("FAULT:"):
                    self._status.fault = line[6:].strip()
                return
            self._status.raw = parsed
            self._status.homed = bool(parsed.get("homed", self._status.homed))
            self._status.estop = bool(parsed.get("estop", self._status.estop))
            self._status.fault = str(parsed.get("fault", self._status.fault) or "")
            self._status.position_mm = _optional_float(parsed.get("position_mm"))
            self._status.velocity_mm_s = float(parsed.get("velocity_mm_s", 0.0))
            self._status.target_velocity_mm_s = float(
                parsed.get("target_velocity_mm_s", 0.0)
            )
            self._status.left_limit = bool(parsed.get("left_limit", False))
            self._status.right_limit = bool(parsed.get("right_limit", False))
            self._status.watchdog_ok = bool(parsed.get("watchdog_ok", False))
            self._status.rail_length_mm = _optional_float(parsed.get("rail_length_mm"))

    @property
    def status(self) -> MotorStatus:
        with self._status_lock:
            status = MotorStatus(**asdict(self._status))
        stale_after = max(0.5, float(self.config.get("watchdog_s", 0.4)) * 2.0)
        if status.connected and time.monotonic() - status.last_rx_s > stale_after:
            status.connected = False
            status.watchdog_ok = False
            status.fault = status.fault or "RP2040 status stale"
        return status

    def set_velocity(self, velocity_mm_s: float) -> None:
        if not math.isfinite(velocity_mm_s):
            raise ValueError("velocity must be finite")
        if abs(velocity_mm_s) > 600.0 + 1e-9:
            raise ValueError("normal velocity command exceeds 600 mm/s")
        self._write(f"VEL:{velocity_mm_s:.3f}")

    def stop(self) -> None:
        self._write("STOP")

    def estop(self) -> None:
        self._write("ESTOP")

    def clear_estop(self) -> None:
        self._write("CLEAR_ESTOP")

    def clear_fault(self) -> None:
        self._write("CLEAR_FAULT")

    def home(self) -> None:
        self._write("FULL_HOME")

    def status_query(self) -> None:
        self._write("STATUS?")


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    result = float(value)
    return result if math.isfinite(result) else None


class DisconnectedMotorInterface:
    """Safe default used when hardware.enabled is false; it never simulates success."""

    def __init__(self) -> None:
        self._status = MotorStatus(connected=False, fault="hardware disabled in config")

    @property
    def status(self) -> MotorStatus:
        return MotorStatus(**asdict(self._status))

    def connect(self) -> None:
        return None

    def close(self) -> None:
        return None

    def set_velocity(self, velocity_mm_s: float) -> None:
        raise ConnectionError("hardware is disabled")

    def stop(self) -> None:
        return None

    def estop(self) -> None:
        return None

    def clear_estop(self) -> None:
        raise ConnectionError("hardware is disabled")

    def clear_fault(self) -> None:
        raise ConnectionError("hardware is disabled")

    def home(self) -> None:
        raise ConnectionError("hardware is disabled")

    def status_query(self) -> None:
        return None

