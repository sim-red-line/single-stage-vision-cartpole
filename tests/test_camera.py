import cv2
import numpy as np
import pytest

from raspberry_pi.camera import CameraCapture, CameraError


class FakeCapture:
    def __init__(self) -> None:
        self.settings: list[tuple[int, float]] = []
        self.released = False

    def isOpened(self) -> bool:
        return True

    def set(self, prop: int, value: float) -> bool:
        self.settings.append((prop, value))
        return True

    def get(self, _prop: int) -> float:
        return 0.0

    def release(self) -> None:
        self.released = True


def test_camera_applies_fourcc_before_resolution(monkeypatch):
    fake = FakeCapture()
    monkeypatch.setattr(cv2, "VideoCapture", lambda *_args: fake)

    camera = CameraCapture(
        {"index": 0, "fourcc": "MJPG", "width": 1280, "height": 720, "fps": 60}
    )
    camera.open()

    expected = float(cv2.VideoWriter_fourcc(*"MJPG"))
    assert fake.settings[0] == (cv2.CAP_PROP_FOURCC, expected)
    assert fake.settings[1][0] == cv2.CAP_PROP_FRAME_WIDTH


def test_camera_rejects_invalid_fourcc(monkeypatch):
    fake = FakeCapture()
    monkeypatch.setattr(cv2, "VideoCapture", lambda *_args: fake)

    camera = CameraCapture({"index": 0, "fourcc": "MJPEG"})
    with pytest.raises(CameraError, match="exactly four"):
        camera.open()

    assert fake.released


def timestamp_camera(monkeypatch, timestamps, deliveries, backend=cv2.CAP_V4L2):
    fake = FakeCapture()
    source = iter(timestamps)
    received = iter(deliveries)
    fake.read = lambda: (True, np.zeros((2, 2, 3), dtype=np.uint8))
    fake.get = lambda prop: (float(backend) if prop == cv2.CAP_PROP_BACKEND
                             else next(source) * 1000.0)
    monkeypatch.setattr(cv2, "VideoCapture", lambda *_args: fake)
    monkeypatch.setattr("raspberry_pi.camera.time.monotonic", lambda: next(received))
    camera = CameraCapture({})
    camera.open()
    return camera


def test_capture_intervals_exclude_delivery_jitter_and_preserve_dropped_frames(monkeypatch):
    camera = timestamp_camera(monkeypatch, [100, 100.033, 100.100],
                              [100.010, 100.060, 100.106])
    packets = [camera.read() for _ in range(3)]
    assert packets[1].timestamp_s - packets[0].timestamp_s == pytest.approx(.033)
    assert packets[2].timestamp_s - packets[1].timestamp_s == pytest.approx(.067)
    assert [p.sequence for p in packets] == [1, 2, 3]


def test_buffered_capture_retains_age_for_stale_frame_guard(monkeypatch):
    camera = timestamp_camera(monkeypatch, [100, 100.033], [100.010, 100.333])
    camera.read()
    assert 100.333 - camera.read().timestamp_s == pytest.approx(.300)


@pytest.mark.parametrize("source", [0, float("nan"), float("inf"), 101, 1])
def test_invalid_driver_clock_rejects_frame(monkeypatch, source):
    camera = timestamp_camera(monkeypatch, [source], [100.010])
    with pytest.raises(CameraError, match="V4L2 capture"):
        camera.read()


@pytest.mark.parametrize("source", [100, 99.99])
def test_repeated_or_backward_capture_timestamp_rejects_frame(monkeypatch, source):
    camera = timestamp_camera(monkeypatch, [100, source], [100.010, 100.040])
    camera.read()
    with pytest.raises(CameraError, match="did not advance"):
        camera.read()


def test_other_camera_backend_keeps_monotonic_delivery_clock(monkeypatch):
    camera = timestamp_camera(monkeypatch, [], [100.010], backend=cv2.CAP_FFMPEG)
    assert camera.read().timestamp_s == 100.010
