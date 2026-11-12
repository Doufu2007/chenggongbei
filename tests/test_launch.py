"""Run the real shell launcher with a tiny transport executable boundary."""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BASH = r"C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash") or ""


def shell_path(path):
    value = path.as_posix()
    return "/" + value[0].lower() + value[2:] if os.name == "nt" else value


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_uses_installed_share_and_propagates_child_failure(tmp_path):
    (tmp_path / "bin").mkdir()
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    (share / "config/inspection.yaml").write_text("{}", encoding="utf-8")
    setup = tmp_path / "setup.sh"
    setup.write_text('export PATH="$FAKE_BIN:$PATH"\n', encoding="utf-8")
    ros2 = tmp_path / "bin/ros2"
    ros2.write_text('''#!/usr/bin/env bash
if [[ "$1" == pkg ]]; then echo "$FAKE_SHARE"; exit 0; fi
if [[ "$3" == validate_config ]]; then exit 0; fi
if [[ "$3" == mission_node ]]; then exit 17; fi
if [[ "$3" == vision_node ]]; then exit 17; fi
''', encoding="utf-8")
    ros2.chmod(0o755)
    env = dict(os.environ, INSPECTION_ROS_SETUP=setup.as_posix(),
               FAKE_BIN=shell_path(tmp_path / "bin"), FAKE_SHARE=shell_path(share))
    result = subprocess.run([BASH, (ROOT / "scripts/run_inspection.sh").as_posix()],
                            env=env, cwd=tmp_path, capture_output=True, text=True, encoding="utf-8", timeout=10)
    assert result.returncode == 17, result.stdout + result.stderr
    assert "INSPECTION_MISSION launch_start" in result.stdout
    assert "INSPECTION_MISSION launch_exit exit_code=17" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_reports_missing_installed_package_before_starting_nodes(tmp_path):
    (tmp_path / "bin").mkdir()
    setup = tmp_path / "setup.sh"
    setup.write_text('export PATH="$FAKE_BIN:$PATH"\n', encoding="utf-8")
    ros2 = tmp_path / "bin/ros2"
    ros2.write_text("#!/usr/bin/env bash\nexit 41\n", encoding="utf-8")
    ros2.chmod(0o755)
    env = dict(os.environ, INSPECTION_ROS_SETUP=setup.as_posix(),
               FAKE_BIN=shell_path(tmp_path / "bin"))

    result = subprocess.run([BASH, (ROOT / "scripts/run_inspection.sh").as_posix()],
                            env=env, cwd=tmp_path, capture_output=True, text=True, encoding="utf-8", timeout=10)

    assert result.returncode == 41
    assert "inspection_mission package lookup failed" in result.stderr
    assert "INSPECTION_MISSION launch_exit exit_code=41" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_refuses_missing_ros_setup_before_launching(tmp_path):
    result = subprocess.run([BASH, (ROOT / "scripts/run_inspection.sh").as_posix()],
                            env=dict(os.environ, INSPECTION_ROS_SETUP=(tmp_path / "missing.sh").as_posix()),
                            capture_output=True, text=True, encoding="utf-8", timeout=10)
    assert result.returncode != 0
    assert "ROS setup" in result.stderr


@pytest.mark.parametrize(("vision_exit", "expected_exit"), [(0, 1), (23, 23)])
@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_rejects_vision_node_exiting_before_mission(tmp_path, vision_exit, expected_exit):
    (tmp_path / "bin").mkdir()
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    (share / "config/inspection.yaml").write_text("{}", encoding="utf-8")
    setup = tmp_path / "setup.sh"
    setup.write_text('export PATH="$FAKE_BIN:$PATH"\n', encoding="utf-8")
    ros2 = tmp_path / "bin/ros2"
    ros2.write_text(f'''#!/usr/bin/env bash
if [[ "$1" == pkg ]]; then echo "$FAKE_SHARE"; exit 0; fi
if [[ "$3" == validate_config ]]; then exit 0; fi
if [[ "$3" == mission_node ]]; then exec python -c 'import time; time.sleep(0.2)'; fi
if [[ "$3" == vision_node ]]; then exit {vision_exit}; fi
''', encoding="utf-8")
    ros2.chmod(0o755)
    setsid = tmp_path / "bin/setsid"
    setsid.write_text('#!/usr/bin/env bash\nexec "$@"\n', encoding="utf-8")
    setsid.chmod(0o755)
    env = dict(os.environ, INSPECTION_ROS_SETUP=setup.as_posix(),
               FAKE_BIN=shell_path(tmp_path / "bin"), FAKE_SHARE=shell_path(share))

    result = subprocess.run([BASH, (ROOT / "scripts/run_inspection.sh").as_posix()],
                            env=env, cwd=tmp_path, capture_output=True, text=True, encoding="utf-8", timeout=10)

    assert result.returncode == expected_exit, result.stdout + result.stderr
    assert "vision_node exited before mission_node" in result.stderr


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_accepts_mission_success_and_cleans_up_vision(tmp_path):
    (tmp_path / "bin").mkdir()
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    (share / "config/inspection.yaml").write_text("{}", encoding="utf-8")
    setup = tmp_path / "setup.sh"
    setup.write_text('export PATH="$FAKE_BIN:$PATH"\n', encoding="utf-8")
    ros2 = tmp_path / "bin/ros2"
    ros2.write_text('''#!/usr/bin/env bash
if [[ "$1" == pkg ]]; then echo "$FAKE_SHARE"; exit 0; fi
if [[ "$3" == validate_config ]]; then exit 0; fi
if [[ "$3" == mission_node ]]; then exit 0; fi
if [[ "$3" == vision_node ]]; then exec python -c 'import time; time.sleep(30)'; fi
''', encoding="utf-8")
    ros2.chmod(0o755)
    setsid = tmp_path / "bin/setsid"
    setsid.write_text('#!/usr/bin/env bash\nexec "$@"\n', encoding="utf-8")
    setsid.chmod(0o755)
    env = dict(os.environ, INSPECTION_ROS_SETUP=setup.as_posix(),
               FAKE_BIN=shell_path(tmp_path / "bin"), FAKE_SHARE=shell_path(share))

    result = subprocess.run([BASH, (ROOT / "scripts/run_inspection.sh").as_posix()],
                            env=env, cwd=tmp_path, capture_output=True, text=True, encoding="utf-8", timeout=10)

    assert result.returncode == 0, result.stdout + result.stderr


def make_overlay_workspace(root, *, has_package=True, setup_source=None):
    overlay = root / "install" / "setup.bash"
    overlay.parent.mkdir(parents=True)
    share = root / "share"
    if has_package:
        (share / "config").mkdir(parents=True)
        (share / "config/inspection.yaml").write_text("{}", encoding="utf-8")
    if setup_source is None:
        value = shell_path(share) if has_package else ""
        setup_source = f'export FAKE_SHARE="{value}"\n'
    overlay.write_text(setup_source, encoding="utf-8")
    return share


def run_overlay_launcher(tmp_path, script_workspace, cwd, *, explicit_workspace=None):
    script = script_workspace / "scripts/run_inspection.sh"
    script.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / "scripts/run_inspection.sh", script)
    fake_bin = tmp_path / "fake_bin"
    fake_bin.mkdir()
    ros2 = fake_bin / "ros2"
    ros2.write_text('''#!/usr/bin/env bash
if [[ "$1" == pkg ]]; then
  if [[ -z "${FAKE_SHARE:-}" ]]; then exit 41; fi
  printf '%s\\n' "$FAKE_SHARE"
  exit 0
fi
if [[ "$3" == validate_config ]]; then
  printf 'VALIDATED_CONFIG=%s\\n' "$5"
  exit 19
fi
exit 93
''', encoding="utf-8")
    ros2.chmod(0o755)
    setup = tmp_path / "ros_setup.sh"
    setup.write_text('export PATH="$FAKE_BIN:$PATH"\nunset FAKE_SHARE\n', encoding="utf-8")
    env = dict(os.environ, INSPECTION_ROS_SETUP=shell_path(setup), FAKE_BIN=shell_path(fake_bin))
    env.pop("INSPECTION_WORKSPACE", None)
    if explicit_workspace is not None:
        env["INSPECTION_WORKSPACE"] = shell_path(explicit_workspace)
    return subprocess.run(
        [BASH, script.as_posix()], cwd=cwd, env=env, capture_output=True,
        text=True, encoding="utf-8", timeout=10,
    )


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_prefers_its_workspace_over_an_unrelated_cwd_overlay(tmp_path):
    workspace = tmp_path / "script workspace"
    share = make_overlay_workspace(workspace)
    unrelated = tmp_path / "unrelated cwd"
    make_overlay_workspace(unrelated)

    result = run_overlay_launcher(tmp_path, workspace, unrelated)

    assert result.returncode == 19, result.stdout + result.stderr
    assert f"VALIDATED_CONFIG={shell_path(share)}/config/inspection.yaml" in result.stdout


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_prefers_an_explicit_workspace_over_its_own_and_cwd(tmp_path):
    workspace = tmp_path / "script workspace"
    make_overlay_workspace(workspace)
    cwd = tmp_path / "cwd workspace"
    make_overlay_workspace(cwd)
    explicit = tmp_path / "explicit workspace"
    share = make_overlay_workspace(explicit)

    result = run_overlay_launcher(tmp_path, workspace, cwd, explicit_workspace=explicit)

    assert result.returncode == 19, result.stdout + result.stderr
    assert f"VALIDATED_CONFIG={shell_path(share)}/config/inspection.yaml" in result.stdout


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_keeps_cwd_overlay_as_a_fallback(tmp_path):
    workspace = tmp_path / "separate launcher"
    cwd = tmp_path / "cwd workspace"
    share = make_overlay_workspace(cwd)

    result = run_overlay_launcher(tmp_path, workspace, cwd)

    assert result.returncode == 19, result.stdout + result.stderr
    assert f"VALIDATED_CONFIG={shell_path(share)}/config/inspection.yaml" in result.stdout


@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_continues_after_an_overlay_without_the_mission_package(tmp_path):
    workspace = tmp_path / "script workspace"
    share = make_overlay_workspace(workspace)
    unrelated = tmp_path / "unrelated explicit workspace"
    make_overlay_workspace(unrelated, has_package=False)

    result = run_overlay_launcher(tmp_path, workspace, tmp_path, explicit_workspace=unrelated)

    assert result.returncode == 19, result.stdout + result.stderr
    assert f"VALIDATED_CONFIG={shell_path(share)}/config/inspection.yaml" in result.stdout


@pytest.mark.parametrize("setup_source,expected_exit", [("return 37\n", 37), ("exit 39\n", 39)])
@pytest.mark.skipif(not Path(BASH).exists(), reason="bash unavailable")
def test_launcher_preserves_overlay_source_failure(tmp_path, setup_source, expected_exit):
    workspace = tmp_path / "script workspace"
    make_overlay_workspace(workspace, setup_source=setup_source)
    cwd = tmp_path / "cwd workspace"
    make_overlay_workspace(cwd)

    result = run_overlay_launcher(tmp_path, workspace, cwd)

    assert result.returncode == expected_exit, result.stdout + result.stderr
    assert "VALIDATED_CONFIG=" not in result.stdout
    assert f"INSPECTION_MISSION launch_exit exit_code={expected_exit}" in result.stderr
