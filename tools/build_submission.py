"""Build only source files; no staging directory or recursive deletion."""
from __future__ import annotations

import hashlib
import json
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src/inspection_mission"))

from tools.preflight import MAX_SRC_ZIP_BYTES, run_preflight
from inspection_mission.validation import validate_configuration


def build():
    report = run_preflight(ROOT)
    if not report.ok:
        raise ValueError("preflight: " + "; ".join(report.errors))
    dist = ROOT / "dist"
    dist.mkdir(exist_ok=True)
    archive = dist / "inspection_src.zip"
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as package:
        for path in sorted((ROOT / "src").rglob("*")):
            if not path.is_file() or "__pycache__" in path.parts or path.suffix in (".pyc", ".pyo"):
                continue
            package.write(path, path.relative_to(ROOT).as_posix())
    if archive.stat().st_size > MAX_SRC_ZIP_BYTES:
        raise ValueError("archive exceeds 20 MB")
    script = dist / "run_inspection.sh"
    script.write_text((ROOT / "scripts/run_inspection.sh").read_text(encoding="utf-8").replace("\r\n", "\n"), encoding="utf-8", newline="\n")
    script.chmod(0o755)
    hashes = "\n".join(hashlib.sha256(path.read_bytes()).hexdigest() + "  " + path.name for path in (archive, script)) + "\n"
    (dist / "SHA256SUMS.txt").write_text(hashes, encoding="ascii")
    try:
        validate_configuration(ROOT / "src/inspection_mission/config/inspection.yaml")
        configuration_ready, reason = True, "configuration passes local validation; official execution still pending"
    except (ValueError, KeyError, TypeError, OSError) as exc:
        configuration_ready, reason = False, str(exc)
    readiness = {"kind": "development_build", "configuration_ready": configuration_ready,
                 "platform_run_verified": False, "reason": reason,
                 "note": "Do not spend a formal upload attempt on this unbound development configuration."}
    (dist / "readiness.json").write_text(json.dumps(readiness, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"archive": str(archive), "bytes": archive.stat().st_size, **readiness}, ensure_ascii=False))
    return archive


if __name__ == "__main__":
    build()
