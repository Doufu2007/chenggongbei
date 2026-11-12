"""Build isolated practice artifacts using explicitly reviewed source digests."""
import argparse
import getpass
import hashlib
import json
import re
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE_SHA = "6c3f39642065ee14b07775a37fa1b9eac6dce91a37977382b337ad27858bbf1e"
MODES = {
    "direction_v28": ("ground_direction_response", "run_ground_direction_response"),
    "trim_v29": ("bounded_vertical_trim", "run_bounded_vertical_trim"),
    "reverse_v30": ("ground_reverse_response", "run_ground_reverse_response"),
    "axis_v32": ("ground_three_point_axis", "run_ground_three_point_axis"),
    "height_v31": ("bounded_height_hold", "run_bounded_height_hold"),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=MODES)
    parser.add_argument("--source-sha", required=True)
    parser.add_argument("--launcher-sha", required=True)
    args = parser.parse_args()
    module, launcher = MODES[args.mode]
    source = ROOT / "src/inspection_mission/inspection_mission" / (module+".py")
    shell = ROOT / "scripts" / (launcher+".sh")
    base = ROOT / "dist/inspection_single_point_v25_src.zip"
    for path, digest in ((source, args.source_sha), (shell, args.launcher_sha), (base, BASE_SHA)):
        if not re.fullmatch("[0-9a-f]{64}", digest) or sha(path) != digest:
            raise ValueError("reviewed artifact mismatch: "+path.name)
    archive = ROOT / ("dist/inspection_"+args.mode+"_src.zip")
    script = ROOT / ("dist/run_"+args.mode+".sh")
    manifest_path = ROOT / ("runs/evidence/20261112/"+args.mode+"_artifacts.json")
    if any(p.exists() for p in (archive, script, manifest_path)):
        raise ValueError("existing practice artifact must be preserved")
    metadata = json.loads(getpass.getpass("Receiver metadata (hidden): "))
    local = urllib.parse.urlsplit(metadata["local_url"])
    if local.scheme != "http" or local.hostname != "127.0.0.1" or local.port != 63974:
        raise ValueError("unexpected isolated receiver")
    old = (ROOT / "dist/run_single_point_v25.sh").read_text(encoding="utf-8")
    match = re.search(r'^export INSPECTION_DIAGNOSTIC_URL="([^"]+)"$', old, re.MULTILINE)
    if match is None:
        raise ValueError("existing practice tunnel absent")
    host = urllib.parse.urlsplit(match.group(1))
    if host.scheme != "https" or not host.hostname or host.username or host.password:
        raise ValueError("invalid existing tunnel")
    url = urllib.parse.urlunsplit(("https", host.netloc, local.path, "", ""))
    if any(c in url for c in '\r\n"$`\\'):
        raise ValueError("invalid callback URL")
    try:
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(url, timeout=15) as response:
            status = response.status
    except urllib.error.HTTPError as error:
        status = error.code
    if status != 405:
        raise ValueError("callback preflight failed, HTTP "+str(status))
    text = shell.read_text(encoding="utf-8").replace("\r\n", "\n")
    if not text.startswith("#!/usr/bin/env bash\n") or "python3 -u -m inspection_mission."+module not in text:
        raise ValueError("unexpected reviewed practice launcher")
    if text.count("export PYTHONUNBUFFERED=1\n") != 1:
        raise ValueError("unexpected launcher environment marker")
    text = text.replace("export PYTHONUNBUFFERED=1\n", "export PYTHONUNBUFFERED=1\nexport INSPECTION_DIAGNOSTIC_URL=\""+url+"\"\n")
    additions = [source]
    if args.mode in ("trim_v29", "height_v31"):
        dependency = ROOT / "src/inspection_mission/inspection_mission/bounded_vertical_response.py"
        if sha(dependency) != "1ee83ca7f52a5e1e60de63f778e7caae939b9d4645f510d064bfc57f3016fd78":
            raise ValueError("frozen vertical dependency changed")
        additions.append(dependency)
    if args.mode == "reverse_v30":
        dependency = ROOT / "src/inspection_mission/inspection_mission/ground_direction_response.py"
        if sha(dependency) != "aeecaa1a50b585cfdd13218a94d56283f0ea93f03a71151475512e852bb21946":
            raise ValueError("frozen direction dependency changed")
        additions.append(dependency)
    if args.mode == "axis_v32":
        for filename, digest in (
            ("ground_three_point.py", "6f7a7bb24f9e5269c72bafc05929085d111599a66a135f2dd3782bef5728abc2"),
            ("ground_three_point_plan.py", "d4d4ade05388de5f7435399f2092824aa291c2f1440c6b3fa3adc0941bfa6c65"),
            ("ground_three_point_axis_plan.py", "e1bb65499f24eba7389b645dd29c10539bd7942fa51675d57912d95454deba8e"),
        ):
            dependency = ROOT / "src/inspection_mission/inspection_mission" / filename
            if sha(dependency) != digest:
                raise ValueError("frozen axis dependency changed: "+filename)
            additions.append(dependency)
    replace = {p.relative_to(ROOT).as_posix(): p for p in additions}
    with zipfile.ZipFile(base) as original, zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED, compresslevel=9) as out:
        for entry in original.infolist():
            if entry.filename not in replace:
                out.writestr(entry, original.read(entry.filename))
        for name, path in replace.items():
            out.write(path, name)
    with script.open("x", encoding="utf-8", newline="\n") as handle:
        handle.write(text)
    with zipfile.ZipFile(archive) as out:
        if out.testzip() or any(out.read(name) != p.read_bytes() for name, p in replace.items()):
            raise ValueError("practice archive integrity failed")
    if archive.stat().st_size > 20*1024*1024:
        raise ValueError("practice archive exceeds size cap")
    result = {"version": args.mode, "mode": "practice", "official_task_success": False,
        "full_mission_complete": False, "receiver_get_status": status, "test_event_written": False,
        "source_files_sha256": {name: sha(p) for name, p in replace.items()},
        "reviewed_launcher_sha256": args.launcher_sha, "base_sha256": BASE_SHA,
        "artifacts": [{"path": p.relative_to(ROOT).as_posix(), "bytes": p.stat().st_size, "sha256": sha(p)} for p in (archive, script)]}
    manifest_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=True), flush=True)


if __name__ == "__main__":
    main()
