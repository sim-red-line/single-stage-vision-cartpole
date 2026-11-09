"""Best-effort local experiment video recording off the camera/control threads."""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np


@dataclass(slots=True)
class VideoRecordingResult:
    enabled: bool
    path: str | None
    frames_written: int
    frames_dropped: int
    error: str | None = None


class ExperimentVideoRecorder:
    def __init__(self, directory: str | Path, config: dict[str, Any]) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.enabled = bool(config.get("record_video", True))
        self.codec = str(config.get("video_codec", "MJPG"))
        if len(self.codec) != 4:
            raise ValueError("video_codec must contain exactly four FourCC characters")
        self.queue: queue.Queue[np.ndarray] = queue.Queue(
            maxsize=int(config.get("video_queue_size", 24))
        )
        self._writer: cv2.VideoWriter | None = None
        self._path: Path | None = None
        self._active = False
        self._frames_written = 0
        self._frames_dropped = 0
        self._error: str | None = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._worker = threading.Thread(
            target=self._writer_loop, name="experiment-video-writer", daemon=True
        )
        self._worker.start()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def start(
        self, experiment_id: str, frame_size: tuple[int, int], fps: float
    ) -> VideoRecordingResult:
        if not self.enabled:
            return VideoRecordingResult(False, None, 0, 0)
        with self._lock:
            if self._active or self._writer is not None:
                raise RuntimeError("a video recording is already active")
            self._path = None
            self._frames_written = 0
            self._frames_dropped = 0
            self._error = None
            width, height = frame_size
            if width <= 0 or height <= 0 or fps <= 0:
                raise ValueError("video frame size and FPS must be positive")
            path = self.directory / f"{experiment_id}.avi"
            writer = cv2.VideoWriter(
                str(path),
                cv2.VideoWriter_fourcc(*self.codec),
                float(fps),
                (int(width), int(height)),
            )
            if not writer.isOpened():
                writer.release()
                self._error = f"unable to open video writer with codec {self.codec}"
                return VideoRecordingResult(True, None, 0, 0, self._error)
            self._writer = writer
            self._path = path
            self._active = True
            return VideoRecordingResult(True, str(path), 0, 0)

    def mark_unavailable(self, error: str) -> VideoRecordingResult:
        with self._lock:
            if self._active or self._writer is not None:
                raise RuntimeError("cannot mark video unavailable while recording")
            self._path = None
            self._frames_written = 0
            self._frames_dropped = 0
            self._error = error
            return self.status()

    def append(self, frame_bgr: np.ndarray) -> None:
        with self._lock:
            if not self._active:
                return
        frame = frame_bgr.copy()
        with self._lock:
            if not self._active:
                return
            try:
                # Queue insertion and the active check are atomic with finish().
                self.queue.put_nowait(frame)
            except queue.Full:
                self._frames_dropped += 1

    def status(self) -> VideoRecordingResult:
        with self._lock:
            return VideoRecordingResult(
                enabled=self.enabled,
                path=str(self._path) if self._path else None,
                frames_written=self._frames_written,
                frames_dropped=self._frames_dropped,
                error=self._error,
            )

    def finish(self) -> VideoRecordingResult:
        with self._lock:
            self._active = False
            path = str(self._path) if self._path else None
        self.queue.join()
        with self._lock:
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            if path and self._frames_written > 0:
                output = Path(path)
                if not output.exists() or output.stat().st_size == 0:
                    self._error = self._error or "video writer produced no file data"
            result = VideoRecordingResult(
                enabled=self.enabled,
                path=path,
                frames_written=self._frames_written,
                frames_dropped=self._frames_dropped,
                error=self._error,
            )
            self._path = None
            return result

    def _writer_loop(self) -> None:
        while not self._stop.is_set() or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                with self._lock:
                    writer = self._writer
                if writer is not None:
                    writer.write(frame)
                    with self._lock:
                        self._frames_written += 1
            except Exception as exc:
                with self._lock:
                    self._error = f"video write failed: {exc}"
                    self._frames_dropped += 1
            finally:
                self.queue.task_done()

    def close(self) -> None:
        if self.active or self._writer is not None:
            self.finish()
        self._stop.set()
        self._worker.join(timeout=2.0)

