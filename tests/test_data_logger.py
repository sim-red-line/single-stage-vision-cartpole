import csv
import json

from raspberry_pi.data_logger import ExperimentLogger


def test_logger_writes_csv_and_measured_json_summary(tmp_path):
    logger = ExperimentLogger(
        tmp_path / "logs",
        tmp_path / "summaries",
        {"queue_size": 1000, "flush_period_s": 0.01},
    )
    active = logger.start(
        {
            "kp_angle": 1.0,
            "ki_angle": 0.0,
            "kd_angle": 0.1,
            "kp_position": 0.0,
            "ki_position": 0.0,
            "kd_position": 0.0,
        },
        "test run",
    )
    for index in range(61):
        logger.append(
            {
                "timestamp_monotonic_s": active.started_monotonic_s + index * 0.1,
                "theta_rad": 0.01,
                "omega_rad_s": 0.0,
                "cart_position_mm": 250.0,
                "cart_velocity_mm_s": 0.0,
                "requested_velocity_mm_s": 1.0,
                "limited_velocity_mm_s": 1.0,
                "tracking_valid": True,
                "tracking_status": "ok",
                "tracking_confidence": 0.9,
                "output_saturated": index < 10,
                "processing_ms": 4.0,
                "control_latency_ms": 12.0,
                "fps": 10.0,
            }
        )
    summary = logger.finish({"gain_source": "config"})
    logger.close()

    with active.csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 61
    assert summary["metrics"]["sample_count"] == 61
    assert summary["metrics"]["average_control_latency_ms"] == 12.0
    assert summary["metrics"]["output_saturation_time_s"] > 0.0
    saved = json.loads(
        (tmp_path / "summaries" / f"{active.experiment_id}.json").read_text(
            encoding="utf-8"
        )
    )
    assert saved["context"]["gain_source"] == "config"

