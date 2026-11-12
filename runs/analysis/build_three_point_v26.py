"""Build a new practice archive while preserving the officially scored v25 base."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path("src/inspection_mission/inspection_mission")
BASE_SHA = "6c3f39642065ee14b07775a37fa1b9eac6dce91a37977382b337ad27858bbf1e"
FROZEN = {
    "ground_observation_route.py": "5ab108c4128db8b32c18cee084fce636e8ce80700f18dd37e468006a2a58b945",
    "ground_single_point.py": "902424ca5ba3ec208a0028645d377f4ce92ff77e5ee56205c87fac95a9b349cb",
    "ground_matcher_worker.py": "d50ae59a24f11d80ca566193cf63e254eb5c55846eff278def24905b96735ec8",
    "ground_plane_matcher.py": "1b9c9b3cd658f5192307161dbbe21954968e9680c98424250d2d0836e1dc710d",
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(report_url, variant="v26"):
    if (not report_url.startswith("https://")
            or any(char in report_url for char in '\r\n"$`\\')
            or variant not in {"v26", "v26b", "v27"}):
        raise ValueError("explicit callback HTTPS URL and version required")
    base = ROOT / "dist/inspection_single_point_v25_src.zip"
    if sha(base) != BASE_SHA:
        raise ValueError("officially scored v25 base changed")
    additions = [ROOT / PACKAGE / name for name in ("ground_three_point.py", "ground_three_point_plan.py")]
    archive = ROOT / f"dist/inspection_three_point_{variant}_src.zip"
    script = ROOT / f"dist/run_three_point_{variant}.sh"
    manifest_path = ROOT / f"runs/evidence/20261112/three_point_{variant}_artifacts.json"
    if any(path.exists() for path in (archive, script, manifest_path)):
        raise ValueError("version already exists; preserve previous artifacts")
    for name, expected in FROZEN.items():
        if sha(ROOT / PACKAGE / name) != expected:
            raise ValueError("frozen production module changed: " + name)
    source = (ROOT / "scripts/run_ground_three_point.sh").read_text(encoding="utf-8").replace("\r\n", "\n")
    source = source.replace("export PYTHONUNBUFFERED=1\n", "export PYTHONUNBUFFERED=1\nexport INSPECTION_DIAGNOSTIC_URL=\"" + report_url + "\"\n", 1)
    if (not source.startswith("#!/usr/bin/env bash\n")
            or "python3 -u -m inspection_mission.ground_three_point --duration 1200" not in source):
        raise ValueError("unexpected practice launcher")
    replacements = {path.relative_to(ROOT).as_posix(): path for path in additions}
    with zipfile.ZipFile(base) as original:
        for name, expected in FROZEN.items():
            if hashlib.sha256(original.read((PACKAGE / name).as_posix())).hexdigest() != expected:
                raise ValueError("frozen base source mismatch")
        with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED, compresslevel=9) as out:
            for entry in original.infolist():
                if entry.filename not in replacements:
                    out.writestr(entry, original.read(entry.filename))
            for name, path in replacements.items():
                out.write(path, name)
    with script.open("x", encoding="utf-8", newline="\n") as handle:
        handle.write(source)
    if archive.stat().st_size > 20 * 1024 * 1024:
        raise ValueError("archive exceeds platform practice size limit")
    with zipfile.ZipFile(archive) as result:
        if result.testzip() is not None:
            raise ValueError("corrupt practice archive")
        for name, path in replacements.items():
            if result.read(name) != path.read_bytes():
                raise ValueError("source bytes changed during build")
    manifest = {"version": "three-point-" + variant, "mode": "practice", "official_task_success": False,
        "full_mission_complete": False, "basis": "frozen scored v25 plus independently reviewed continuous ground modules",
        "source_files_sha256": {path.relative_to(ROOT).as_posix(): sha(path) for path in additions},
        "frozen_base_sha256": BASE_SHA,
        "artifacts": [{"path": path.relative_to(ROOT).as_posix(), "bytes": path.stat().st_size,
                       "sha256": sha(path)} for path in (archive, script)]}
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--report-url", required=True)
    parser.add_argument("--variant", default="v26")
    args = parser.parse_args()
    print(json.dumps(build(args.report_url, args.variant), ensure_ascii=True))
