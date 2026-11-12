"""Exercise the actual bounded-motion shell entry point, without ROS."""
import json
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BASH = Path(r"C:/Program Files/Git/bin/bash.exe") if os.name == "nt" else Path(shutil.which("bash") or "/missing-bash")


def shell_path(path):
    value = Path(path).resolve().as_posix()
    return "/" + value[0].lower() + value[2:] if os.name == "nt" else value


@pytest.mark.skipif(not BASH.is_file(), reason="bash unavailable")
@pytest.mark.parametrize("child_exit,setup_exit", [(0, 0), (23, 0), (0, 31)])
def test_motion_launcher_runs_uploaded_package_and_preserves_real_failure(tmp_path, child_exit, setup_exit):
    workspace = tmp_path / "uploaded motion workspace"
    package = workspace / "src/inspection_mission/inspection_mission"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "bounded_ground_probe.py").write_text(
        "import json,os,sys\n"
        "assert os.environ['FAKE_ROS_READY']=='1'\n"
        "print('MOTION_ARGS '+json.dumps(sys.argv[1:]))\n"
        f"sys.exit({child_exit})\n", encoding="utf-8")
    launcher = workspace / "scripts/run_ground_control_probe.sh"
    launcher.parent.mkdir()
    shutil.copyfile(ROOT / "scripts/run_ground_control_probe.sh", launcher)
    setup = tmp_path / "fake ros setup.sh"
    setup.write_text(f'python3() {{ {shlex.quote(shell_path(sys.executable))} "$@"; }}\n'
                     + (f"return {setup_exit}\n" if setup_exit else "export FAKE_ROS_READY=1\n"), encoding="utf-8")
    env = dict(os.environ, INSPECTION_ROS_SETUP=shell_path(setup))
    env.pop("INSPECTION_DIAGNOSTIC_URL", None)
    result = subprocess.run([str(BASH), str(launcher)], cwd=tmp_path, env=env,
                            capture_output=True, text=True, encoding="utf-8", timeout=10)
    assert result.returncode == (setup_exit or child_exit), result.stdout + result.stderr
    assert f"GROUND_CONTROL_PROBE launch_exit exit_code={setup_exit or child_exit}" in result.stderr
    if not setup_exit:
        line = next(line for line in result.stdout.splitlines() if line.startswith("MOTION_ARGS "))
        args = json.loads(line.removeprefix("MOTION_ARGS "))
        assert args == ["--duration", "45", "--body-radius", "0.447", "--scan-offset-bound", "0.114"]
    else:
        assert "MOTION_ARGS" not in result.stdout
