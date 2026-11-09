"""Raspberry Pi power-health monitoring used by the safety boundary."""

from __future__ import annotations

import re
import subprocess
import time
from dataclasses import dataclass
from typing import Callable


THROTTLE_FLAGS = {
    0x1: "undervoltage detected",
    0x2: "arm frequency capped",
    0x4: "currently throttled",
    0x8: "soft temperature limit active",
    0x10000: "undervoltage occurred since boot",
    0x20000: "arm frequency capping occurred since boot",
    0x40000: "throttling occurred since boot",
    0x80000: "soft temperature limit occurred since boot",
}


@dataclass(frozen=True, slots=True)
class PiPowerStatus:
    timestamp_s: float
    available: bool
    required: bool
    clean: bool | None
    safe_for_control: bool
    raw_value: int | None
    reason: str

    def as_dict(self) -> dict[str, object]:
        return {
            "timestamp_s": self.timestamp_s,
            "available": self.available,
            "required": self.required,
            "clean": self.clean,
            "safe_for_control": self.safe_for_control,
            "raw_value": self.raw_value,
            "raw_hex": f"0x{self.raw_value:x}" if self.raw_value is not None else None,
            "reason": self.reason,
        }


class PiPowerMonitor:
    def __init__(
        self,
        *,
        required: bool,
        runner: Callable[[], str] | None = None,
    ) -> None:
        self.required = required
        self._runner = runner or self._run_vcgencmd
        self.status = PiPowerStatus(
            timestamp_s=time.monotonic(),
            available=False,
            required=required,
            clean=None,
            safe_for_control=not required,
            raw_value=None,
            reason="Pi power status has not been checked",
        )

    @staticmethod
    def _run_vcgencmd() -> str:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=1.0,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or f"vcgencmd exited {result.returncode}"
            raise RuntimeError(detail)
        return result.stdout.strip()

    def poll(self) -> PiPowerStatus:
        timestamp = time.monotonic()
        try:
            output = self._runner()
            match = re.search(r"throttled=(0x[0-9a-fA-F]+)", output)
            if match is None:
                raise ValueError(f"unexpected vcgencmd response: {output!r}")
            raw_value = int(match.group(1), 16)
            active = [name for mask, name in THROTTLE_FLAGS.items() if raw_value & mask]
            clean = raw_value == 0
            reason = "Pi power status clean" if clean else "; ".join(active)
            status = PiPowerStatus(
                timestamp_s=timestamp,
                available=True,
                required=self.required,
                clean=clean,
                safe_for_control=clean or not self.required,
                raw_value=raw_value,
                reason=reason,
            )
        except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
            status = PiPowerStatus(
                timestamp_s=timestamp,
                available=False,
                required=self.required,
                clean=None,
                safe_for_control=not self.required,
                raw_value=None,
                reason=f"Pi power status unavailable: {exc}",
            )
        self.status = status
        return status
