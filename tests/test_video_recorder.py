from pathlib import Path

import numpy as np
import pytest

from raspberry_pi.video_recorder import ExperimentVideoRecorder


def test_video_is_written_locally_and_reports_frame_counts(tmp_path):
    recorder = ExperimentVideoRecorder(
        tmp_path, {"record_video": True, "video_codec": "MJPG", "video_queue_size": 20}
    )
    started = recorder.start("experiment", (64, 48), 20.0)
    if started.error:
        recorder.close()
        pytest.skip(started.error)
    for index in range(6):
        frame = np.full((48, 64, 3), index * 20, dtype=np.uint8)
        recorder.append(frame)
    result = recorder.finish()
    recorder.close()
    assert result.frames_written == 6
    assert result.frames_dropped == 0
    assert result.path is not None
    assert Path(result.path).exists()
    assert Path(result.path).stat().st_size > 0

