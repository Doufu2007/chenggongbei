import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BASH = r"C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash") or ""


def shell_path(path):
    value = Path(path).resolve().as_posix()
    return "/" + value[0].lower() + value[2:] if os.name == "nt" else value


def make_workspace(root, exit_code=0):
    package = root / "src" / "inspection_mission" / "inspection_mission"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "probe.py").write_text(
        "import os, sys\n"
        "print('PROBE_SOURCE_STARTED', flush=True)\n"
        "if os.environ.get('FAKE_ROS_READY') != '1': sys.exit(91)\n"
        f"sys.exit({exit_code})\n",
        encoding="utf-8",
    )


def make_ros_setup(path, source="export FAKE_ROS_READY=1\n"):
    path.parent.mkdir(parents=True, exist_ok=True)
    # The local Windows python3 alias opens the Store; keep the launcher real
    # while mapping its external Python command to the active test interpreter.
    interpreter = shlex.quote(shell_path(sys.executable))
    path.write_text(f'python3() {{ {interpreter} "$@"; }}\n' + source, encoding="utf-8")


def run_launcher(script, cwd, setup, workspace=None):
    env = dict(os.environ)
    env["INSPECTION_ROS_SETUP"] = shell_path(setup)
    env.pop("INSPECTION_WORKSPACE", None)
    if workspace is not None:
        env["INSPECTION_WORKSPACE"] = shell_path(workspace)
    return subprocess.run(
        [BASH, str(script)], cwd=cwd, env=env, capture_output=True,
        text=True, encoding="utf-8", timeout=10,
    )


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_finds_uploaded_source_from_its_own_directory_with_spaces(tmp_path):
    workspace = tmp_path / "uploaded source workspace"
    make_workspace(workspace)
    script = workspace / "scripts" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros setup.sh"
    make_ros_setup(setup)

    result = run_launcher(script, tmp_path, setup)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "INTERFACE_PROBE launch_start" in result.stdout
    assert "PROBE_SOURCE_STARTED" in result.stdout
    assert "INTERFACE_PROBE launch_exit exit_code=0" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_finds_source_from_working_directory_when_script_is_elsewhere(tmp_path):
    workspace = tmp_path / "workspace"
    make_workspace(workspace)
    script = tmp_path / "separate launcher" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup)

    result = run_launcher(script, workspace, setup)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PROBE_SOURCE_STARTED" in result.stdout


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_uses_explicit_workspace_after_script_and_cwd_candidates(tmp_path):
    workspace = tmp_path / "explicit workspace"
    make_workspace(workspace)
    script = tmp_path / "launcher" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup)

    result = run_launcher(script, tmp_path, setup, workspace=workspace)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PROBE_SOURCE_STARTED" in result.stdout


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
@pytest.mark.parametrize(
    ("setup_source", "probe_exit", "expected_exit"),
    [("exit 31\n", 0, 31), ("export FAKE_ROS_READY=1\n", 23, 23)],
)
def test_launcher_reports_and_preserves_setup_and_user_program_exit_codes(
    tmp_path, setup_source, probe_exit, expected_exit
):
    workspace = tmp_path / "workspace"
    make_workspace(workspace, exit_code=probe_exit)
    script = workspace / "scripts" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup, setup_source)

    result = run_launcher(script, tmp_path, setup)

    assert result.returncode == expected_exit, result.stdout + result.stderr
    assert f"INTERFACE_PROBE launch_exit exit_code={expected_exit}" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_fails_clearly_when_uploaded_source_is_missing(tmp_path):
    script = tmp_path / "launcher" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup)

    result = run_launcher(script, tmp_path, setup)

    assert result.returncode == 2
    assert "inspection_mission/inspection_mission/probe.py" in result.stderr
    assert "INTERFACE_PROBE launch_exit exit_code=2" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_prefers_its_uploaded_source_over_an_explicit_unrelated_workspace(tmp_path):
    workspace = tmp_path / "uploaded workspace"
    make_workspace(workspace)
    unrelated_workspace = tmp_path / "unrelated workspace"
    make_workspace(unrelated_workspace, exit_code=89)
    script = workspace / "scripts" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup)

    result = run_launcher(script, tmp_path, setup, workspace=unrelated_workspace)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PROBE_SOURCE_STARTED" in result.stdout


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_tolerates_optional_unset_ros_setup_variables(tmp_path):
    workspace = tmp_path / "workspace"
    make_workspace(workspace)
    script = workspace / "scripts" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup, 'if [ -z "$PROBE_TEST_UNSET_OPTION" ]; then export FAKE_ROS_READY=1; fi\n')

    result = run_launcher(script, tmp_path, setup)

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PROBE_SOURCE_STARTED" in result.stdout


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_preserves_failure_from_the_source_workspaces_overlay(tmp_path):
    workspace = tmp_path / "workspace"
    make_workspace(workspace)
    overlay = workspace / "install" / "setup.bash"
    overlay.parent.mkdir()
    overlay.write_text("return 37\n", encoding="utf-8")
    script = workspace / "scripts" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup)

    result = run_launcher(script, tmp_path, setup)

    assert result.returncode == 37, result.stdout + result.stderr
    assert "PROBE_SOURCE_STARTED" not in result.stdout
    assert "INTERFACE_PROBE launch_exit exit_code=37" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_reports_missing_ros_setup_before_attempting_the_probe(tmp_path):
    script = tmp_path / "run_interface_probe.sh"
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)

    result = run_launcher(script, tmp_path, tmp_path / "missing ROS setup.sh")

    assert result.returncode == 2
    assert "INTERFACE_PROBE launch_start" in result.stdout
    assert "ros_setup_missing" in result.stderr
    assert "INTERFACE_PROBE launch_exit exit_code=2" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_can_run_an_installed_probe_when_no_uploaded_source_is_present(tmp_path):
    workspace = tmp_path / "installed workspace"
    overlay = workspace / "install" / "setup.bash"
    overlay.parent.mkdir(parents=True)
    overlay.write_text(
        'ros2() {\n'
        '  if [[ "$*" != "run inspection_mission interface_probe --duration 30" ]]; then return 92; fi\n'
        '  printf "INSTALLED_PROBE_STARTED\\n"\n'
        '  return 26\n'
        '}\n',
        encoding="utf-8",
    )
    script = workspace / "scripts" / "run_interface_probe.sh"
    script.parent.mkdir()
    shutil.copy2(ROOT / "scripts" / "run_interface_probe.sh", script)
    setup = tmp_path / "ros.sh"
    make_ros_setup(setup)

    result = run_launcher(script, tmp_path, setup)

    assert result.returncode == 26, result.stdout + result.stderr
    assert "INSTALLED_PROBE_STARTED" in result.stdout
    assert "INTERFACE_PROBE launch_exit exit_code=26" in result.stderr
