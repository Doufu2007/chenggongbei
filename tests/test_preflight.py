from pathlib import Path

from tools.preflight import run_preflight


ROOT = Path(__file__).resolve().parents[1]


def test_preflight_accepts_clean_submission_tree():
    report = run_preflight(ROOT)
    assert report.ok, report.errors


def test_preflight_rejects_forbidden_script(tmp_path):
    (tmp_path / "src" / "inspection_mission").mkdir(parents=True)
    (tmp_path / "src" / "inspection_mission" / "package.xml").write_text(
        "<package format='3'><name>inspection_mission</name></package>", encoding="utf-8"
    )
    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "run_inspection.sh").write_text("ros2 launch ican_inspection_sim sim.launch.py\n", encoding="utf-8")
    report = run_preflight(tmp_path)
    assert not report.ok
    assert any("forbidden" in error for error in report.errors)


def test_preflight_allows_reject_list_but_rejects_actual_hidden_subscription(tmp_path):
    import shutil
    shutil.copytree(ROOT / "src", tmp_path / "src", ignore=shutil.ignore_patterns("__pycache__"))
    shutil.copytree(ROOT / "scripts", tmp_path / "scripts")
    probe = tmp_path / "src/inspection_mission/inspection_mission/probe.py"
    probe.write_text('def forbidden(topic):\n    return topic.startswith("/gazebo/")\n', encoding="utf-8")
    assert run_preflight(tmp_path).ok
    probe.write_text('node.create_subscription(T, "/gazebo/model_states", callback, 10)\n', encoding="utf-8")
    report = run_preflight(tmp_path)
    assert not report.ok
    assert any("forbidden" in item for item in report.errors)

