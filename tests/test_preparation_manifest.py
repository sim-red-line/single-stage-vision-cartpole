import hashlib
import json

import pytest

from experiments.prepare_baseline_start import BASELINE_HASH, verify_frozen_files, validate_start_target


def frozen_files(tmp_path):
    config = tmp_path / "config.local.yaml"
    source = tmp_path / "controller.py"
    config.write_text("capture_angle_rad: 0.22\n")
    source.write_text("controller fixture\n")
    digest = hashlib.sha256(config.read_bytes()).hexdigest()
    data = {"files_sha256": {
        config.name: digest,
        source.name: hashlib.sha256(source.read_bytes()).hexdigest(),
    }}
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps(data))
    return config, source, manifest, digest


def test_new_cohort_requires_explicit_matching_configuration_digest(tmp_path):
    _, _, manifest, digest = frozen_files(tmp_path)
    with pytest.raises(RuntimeError, match="explicit configuration digest"):
        verify_frozen_files(tmp_path, manifest, BASELINE_HASH)
    assert verify_frozen_files(tmp_path, manifest, digest)["files_sha256"]["config.local.yaml"] == digest


def test_configuration_changed_after_review_is_rejected(tmp_path):
    config, _, manifest, digest = frozen_files(tmp_path)
    config.write_text("capture_angle_rad: 0.3\n")
    with pytest.raises(RuntimeError, match="configuration changed"):
        verify_frozen_files(tmp_path, manifest, digest)


def test_control_source_changed_after_review_is_rejected(tmp_path):
    _, source, manifest, digest = frozen_files(tmp_path)
    source.write_text("changed controller\n")
    with pytest.raises(RuntimeError, match="frozen baseline source"):
        verify_frozen_files(tmp_path, manifest, digest)


def test_manifest_must_include_configuration_digest(tmp_path):
    _, _, manifest, digest = frozen_files(tmp_path)
    manifest.write_text(json.dumps({"files_sha256": {}}))
    with pytest.raises(RuntimeError, match="explicit configuration digest"):
        verify_frozen_files(tmp_path, manifest, digest)


@pytest.mark.parametrize("target", [235.0, 277.0, float("nan"), float("inf")])
def test_start_pose_adjustment_cannot_escape_existing_central_band(target):
    with pytest.raises(ValueError, match="within20mm"):
        validate_start_target(target, 256.369)


def test_recorded_start_can_be_matched_after_rehome_without_changing_coordinates():
    assert validate_start_target(244.35, 256.369) == 244.35
    assert validate_start_target(246.30, 256.369) == 246.30
