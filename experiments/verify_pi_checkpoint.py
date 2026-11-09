"""Check the saved Pi snapshot after git lfs pull; no network or motor access."""
import hashlib
import json
from pathlib import Path


root = Path(__file__).resolve().parents[1]
manifest_path = root/"backups/pi-before-double-20261101/SNAPSHOT_MANIFEST.json"
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
failures = []
for entry in manifest["files"]:
    target = (root/entry["snapshot"]).resolve()
    assert target.is_relative_to(root/"backups/pi-before-double-20261101")
    if not target.is_file() or hashlib.sha256(target.read_bytes()).hexdigest() != entry["sha256"]:
        failures.append(entry["snapshot"])
print(json.dumps({"verified_files":len(manifest["files"])-len(failures),
                  "total_files":len(manifest["files"]),"failures":failures,
                  "hint":"Run git lfs pull if media files are still LFS pointers."},indent=2))
raise SystemExit(1 if failures else 0)
