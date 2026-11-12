"""Package the reviewed finite vertical experiment without modifying scored v25."""
from pathlib import Path
import getpass
import hashlib
import json
import re
import urllib.error
import urllib.parse
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "dist/inspection_single_point_v25_src.zip"
MODULE = ROOT / "src/inspection_mission/inspection_mission/bounded_vertical_response.py"
LAUNCHER = ROOT / "scripts/run_bounded_vertical_response.sh"
EXPECTED = {
    BASE: "6c3f39642065ee14b07775a37fa1b9eac6dce91a37977382b337ad27858bbf1e",
    MODULE: "1ee83ca7f52a5e1e60de63f778e7caae939b9d4645f510d064bfc57f3016fd78",
    LAUNCHER: "e161324e54bd411e6d14e0051c28bb1d4f4a8454270d63fb64cd4adf262e3baf",
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    archive = ROOT / "dist/inspection_vertical_v27_src.zip"
    script = ROOT / "dist/run_vertical_v27.sh"
    manifest_path = ROOT / "runs/evidence/20261112/vertical_v27_artifacts.json"
    if any(p.exists() for p in (archive, script, manifest_path)):
        raise ValueError("preserve existing experiment artifacts")
    for path, expected in EXPECTED.items():
        if sha(path) != expected:
            raise ValueError("frozen source mismatch: " + path.name)
    receiver = json.loads(getpass.getpass("Receiver metadata (hidden): "))
    local = urllib.parse.urlsplit(receiver["local_url"])
    if local.scheme != "http" or local.hostname != "127.0.0.1" or local.port != 63974:
        raise ValueError("unexpected isolated loopback receiver")
    old = (ROOT / "dist/run_single_point_v25.sh").read_text(encoding="utf-8")
    match = re.search(r'^export INSPECTION_DIAGNOSTIC_URL="([^"]+)"$', old, re.MULTILINE)
    if match is None:
        raise ValueError("existing practice tunnel absent")
    host = urllib.parse.urlsplit(match.group(1))
    if host.scheme != "https" or not host.hostname or host.username or host.password:
        raise ValueError("invalid existing practice tunnel")
    url = urllib.parse.urlunsplit(("https", host.netloc, local.path, "", ""))
    if any(c in url for c in '\r\n"$`\\'):
        raise ValueError("invalid callback URL")
    try:
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(url, timeout=15) as response:
            status = response.status
    except urllib.error.HTTPError as error:
        status = error.code
    if status != 405:
        raise ValueError("isolated callback preflight failed, HTTP " + str(status))
    source = LAUNCHER.read_text(encoding="utf-8").replace("\r\n", "\n")
    if not source.startswith("#!/usr/bin/env bash\n") or "python3 -u -m inspection_mission.bounded_vertical_response\n" not in source:
        raise ValueError("unexpected reviewed launcher")
    source = source.replace("export PYTHONUNBUFFERED=1\n", "export PYTHONUNBUFFERED=1\nexport INSPECTION_DIAGNOSTIC_URL=\"" + url + "\"\n", 1)
    target = MODULE.relative_to(ROOT).as_posix()
    with zipfile.ZipFile(BASE) as original:
        names = set(original.namelist())
        for dependency in ("bounded_ground_probe.py", "telemetry.py"):
            if "src/inspection_mission/inspection_mission/" + dependency not in names:
                raise ValueError("missing experiment dependency: " + dependency)
        with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED, compresslevel=9) as out:
            for entry in original.infolist():
                if entry.filename != target:
                    out.writestr(entry, original.read(entry.filename))
            out.write(MODULE, target)
    with script.open("x", encoding="utf-8", newline="\n") as handle:
        handle.write(source)
    with zipfile.ZipFile(archive) as result:
        if result.testzip() or result.read(target) != MODULE.read_bytes():
            raise ValueError("experiment archive integrity failed")
    if archive.stat().st_size > 20 * 1024 * 1024:
        raise ValueError("practice archive exceeds size cap")
    manifest = {"version": "bounded-vertical-v27", "mode": "practice",
        "official_task_success": False, "full_mission_complete": False,
        "source_sha256": EXPECTED[MODULE], "base_sha256": EXPECTED[BASE],
        "receiver_get_status": status, "test_event_written": False,
        "artifacts": [{"path": p.relative_to(ROOT).as_posix(), "bytes": p.stat().st_size, "sha256": sha(p)} for p in (archive, script)]}
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=True), flush=True)


if __name__ == "__main__":
    main()
