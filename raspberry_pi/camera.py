"""USB camera capture with monotonic acquisition timestamps."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(slots=True)
class FramePacket:
    image: np.ndarray
    timestamp_s: float
    sequence: int


class CameraError(RuntimeError):
    pass


class CameraCapture:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self._capture: cv2.VideoCapture | None = None
        self._lock = threading.Lock()
        self._sequence = 0
        self._driver_timestamps = False
        self._last_timestamp_s: float | None = None

    def open(self) -> None:
        backend = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY
        capture = cv2.VideoCapture(int(self.config.get("index", 0)), backend)
        if not capture.isOpened():
            capture.release()
            # Some platforms reject an explicit backend; retry with auto-detection.
            capture = cv2.VideoCapture(int(self.config.get("index", 0)))
        if not capture.isOpened():
            raise CameraError(f"unable to open camera index {self.config.get('index', 0)}")
        fourcc = str(self.config.get("fourcc", "")).strip().upper()
        if fourcc:
            if len(fourcc) != 4:
                capture.release()
                raise CameraError("camera fourcc must contain exactly four characters")
            capture.set(
                cv2.CAP_PROP_FOURCC,
                float(cv2.VideoWriter_fourcc(*fourcc)),
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.config.get("width", 1280)))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.config.get("height", 720)))
        capture.set(cv2.CAP_PROP_FPS, float(self.config.get("fps", 60)))
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.config.get("exposure") is not None:
            capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
            capture.set(cv2.CAP_PROP_EXPOSURE, float(self.config["exposure"]))
        if self.config.get("gain") is not None:
            capture.set(cv2.CAP_PROP_GAIN, float(self.config["gain"]))
        if self.config.get("white_balance") is not None:
            capture.set(cv2.CAP_PROP_AUTO_WB, 0.0)
            capture.set(
                cv2.CAP_PROP_WB_TEMPERATURE, float(self.config["white_balance"])
            )
        with self._lock:
            self._capture = capture
            self._sequence = 0
            self._driver_timestamps = capture.get(cv2.CAP_PROP_BACKEND) == cv2.CAP_V4L2
            self._last_timestamp_s = None

    def read(self) -> FramePacket:
        with self._lock:
            capture = self._capture
        if capture is None:
            raise CameraError("camera is not open")
        ok, image = capture.read()
        acquired = time.monotonic()
        if not ok or image is None:
            raise CameraError("camera returned an empty frame")
        if self._driver_timestamps:
            # V4L2 POS_MSEC is the dequeued buffer timestamp, before JPEG
            # decoding and Python scheduling. This device uses CLOCK_MONOTONIC;
            # reject incompatible clocks rather than inventing a frame period.
            captured = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if not math.isfinite(captured) or captured <= 0 or captured > acquired:
                raise CameraError("invalid V4L2 capture timestamp")
            if self._last_timestamp_s is None and acquired - captured > 5.0:
                raise CameraError("V4L2 capture clock is not recent monotonic time")
            acquired = captured
        with self._lock:
            if self._last_timestamp_s is not None and acquired <= self._last_timestamp_s:
                raise CameraError("camera capture timestamp did not advance")
            self._last_timestamp_s = acquired
            self._sequence += 1
            sequence = self._sequence
        return FramePacket(image=image, timestamp_s=acquired, sequence=sequence)

    def close(self) -> None:
        with self._lock:
            capture, self._capture = self._capture, None
        if capture is not None:
            capture.release()

    def properties(self) -> dict[str, float | str]:
        with self._lock:
            capture = self._capture
        if capture is None:
            return {}
        return {
            "timestamp_source": "v4l2_capture" if self._driver_timestamps else "delivery",
            "width": capture.get(cv2.CAP_PROP_FRAME_WIDTH),
            "height": capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
            "fps": capture.get(cv2.CAP_PROP_FPS),
            "fourcc": capture.get(cv2.CAP_PROP_FOURCC),
            "exposure": capture.get(cv2.CAP_PROP_EXPOSURE),
            "gain": capture.get(cv2.CAP_PROP_GAIN),
            "white_balance": capture.get(cv2.CAP_PROP_WB_TEMPERATURE),
        }


def encode_jpeg(image: np.ndarray, quality: int = 78) -> bytes:
    ok, encoded = cv2.imencode(
        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise CameraError("JPEG encoding failed")
    return encoded.tobytes()

