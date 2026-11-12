"""Evidence and side-effect boundaries for submission materials."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "create_submission_media.py"


def test_import_does_not_write_materials_or_run_artificial_scenario(tmp_path):
    code = """
import runpy
import sys
import types
trap = types.ModuleType('tools.mission_scenario')
def forbidden(*args, **kwargs):
    raise RuntimeError('artificial scenario ran during import')
trap.demonstration_plan = trap.run_scenario = forbidden
sys.modules['tools.mission_scenario'] = trap
runpy.run_path(sys.argv[1], run_name='media_import')
"""
    result = subprocess.run(
        [sys.executable, "-c", code, str(SCRIPT)], cwd=tmp_path,
        capture_output=True, text=True, encoding="utf-8", timeout=30,
        env={**os.environ, "PYTHONUTF8": "1"},
    )
    assert result.returncode == 0, result.stderr
    assert not (tmp_path / "submission").exists()


def run_cli(*args):
    return subprocess.run(
        [sys.executable, str(SCRIPT), *map(str, args)], cwd=ROOT,
        capture_output=True, text=True, encoding="utf-8", timeout=60,
        env={**os.environ, "PYTHONUTF8": "1"},
    )


def test_draft_pdf_cli_reads_junit_without_generating_video(tmp_path):
    import fitz
    report = tmp_path / "checks.xml"
    report.write_text(
        '<testsuites><testsuite tests="3" failures="1" errors="0" skipped="1">'
        '<testcase name="passed"/><testcase name="failed"><failure>example</failure>'
        '</testcase><testcase name="skipped"><skipped/></testcase>'
        '</testsuite></testsuites>', encoding="utf-8",
    )
    output = tmp_path / "draft.pdf"
    result = run_cli("--pdf", output, "--junit", report)
    assert result.returncode == 0, result.stderr
    assert output.is_file()
    assert not list(tmp_path.glob("*.mp4"))
    with fitz.open(output) as document:
        text = "".join(page.get_text() for page in document)
    assert "技术方案草稿" in text
    assert "总计 3；通过 1；失败 1；错误 0；跳过 1" in text
    assert "/referee/ground_tasks" in text
    assert "std_msgs/msg/String" in text
    assert "/uav/laser_fire" in text
    assert "固定下降距离" in text
    assert "UUID" in text
    assert "官方任务完成率：未知" in text
    assert "官方完整运行录像：尚未取得" in text


def test_official_cli_requires_evidence_before_writing(tmp_path):
    output = tmp_path / "official.pdf"
    result = run_cli("--mode", "official", "--pdf", output)
    assert result.returncode != 0
    assert "--run-record" in result.stderr
    assert "--video-input" in result.stderr
    assert not output.exists()


def test_validation_reader_counts_cases_and_rejects_inconsistent_junit(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("submission_media_test", SCRIPT)
    media = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(media)
    report = tmp_path / "invalid.xml"
    report.write_text('<testsuite tests="5"><testcase/></testsuite>', encoding="utf-8")
    with pytest.raises(ValueError, match="JUnit"):
        media.read_junit(report)


def test_probe_evidence_in_materials_preserves_diagnostic_boundary(tmp_path):
    import hashlib
    import importlib.util
    import json
    spec = importlib.util.spec_from_file_location("submission_probe_evidence", SCRIPT)
    media = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(media)
    callback = tmp_path / "diagnostic.jsonl"
    rows = [
        {"event": "launcher_started", "data": {"read_only": True}},
        {"event": "probe_started", "data": {"read_only": True}},
        {"event": "sample", "data": {"topic": "/odom", "message": {"x": 0.0}}},
        {"event": "launcher_exit", "data": {"exit_code": 0}},
    ]
    callback.write_text("\n".join(json.dumps({"payload": row, "source": "synthetic unit-test callback"})
                                  for row in rows), encoding="utf-8")
    result = media.collect_validation(probe_evidence=callback, probe_output=tmp_path / "analysis")
    probe = result["probe_runtime"]
    assert probe["status"] == "partial"
    assert probe["startup"] == {"launcher_started": True, "probe_started": True}
    assert probe["sample_topics"] == ["/odom"]
    assert probe["launcher_exit_codes"] == [0]
    assert probe["official_scoring"] == "unverified"
    assert "score" not in probe and "full_mission_completed" not in probe
    assert any(item["sha256"] == hashlib.sha256(callback.read_bytes()).hexdigest()
               for item in result["inputs"])


def test_materials_reject_invalid_probe_evidence(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("submission_invalid_probe", SCRIPT)
    media = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(media)
    callback = tmp_path / "invalid.jsonl"
    callback.write_text("{broken", encoding="utf-8")
    with pytest.raises(ValueError, match="probe evidence"):
        media.collect_validation(probe_evidence=callback, probe_output=tmp_path / "analysis")


def probe_material_fixture(tmp_path):
    """Synthetic snapshot tests rendering claims; never represents platform imagery."""
    import json
    from tools.analyze_probe_evidence import analyze_evidence
    from test_run_records import scored_record
    callback = tmp_path / "synthetic-probe.jsonl"
    samples = {
        "/referee/ground_tasks": {"message": {"data": "ground_1,ground_4,ground_0"}},
        "/camera/image": {"message": {"width": 800, "height": 800, "encoding": "rgb8",
                                     "header": {"frame_id": "camera_link"}}},
        "/camera/camera_info": {"message": {"k": [476.7, 0, 400.5, 0, 476.7, 400.5, 0, 0, 1]}},
        "/uav/laser_state": {"message": {"data": False}},
    }
    for topic, sample in samples.items():
        sample["topic"] = topic
    snapshot = {"read_only": True, "official_scoring": "unverified", "elapsed_s": 60,
                "topics": [], "subscriptions": {}, "samples": samples,
                "sample_counts": {topic: 1 for topic in samples}, "errors": {}, "public_endpoints": {}}
    rows = [{"event": "launcher_started", "data": {"read_only": True}},
            {"event": "probe_snapshot", "data": snapshot},
            {"event": "launcher_exit", "data": {"exit_code": 0}}]
    callback.write_text("\n".join(json.dumps({"payload": row, "source": "synthetic unit-test callback"})
                                  for row in rows), encoding="utf-8")
    analysis = tmp_path / "analysis-input"
    analyze_evidence(callback, analysis)
    record = scored_record("synthetic-probe", status="failed")
    record.update(kind="interface_probe", full_mission_completed=None, duration_s=112.172,
                  draw_order=["ground_1", "ground_4", "ground_0"], evidence_paths=[str(callback)])
    record["scores"] = {"items": dict.fromkeys(record["scores"]["items"]), "total": None, "manual": None}
    path = tmp_path / "probe-record.json"
    path.write_text(json.dumps(record), encoding="utf-8")
    return callback, analysis / "summary.json", path


def test_snapshot_pdf_reports_received_data_without_inventing_start_event_or_score(tmp_path):
    import fitz
    import hashlib
    import json
    callback, summary, record = probe_material_fixture(tmp_path)
    output = tmp_path / "snapshot-draft.pdf"
    result = run_cli("--pdf", output, "--probe-evidence", callback,
                     "--probe-summary", summary, "--probe-run-record", record)
    assert result.returncode == 0, result.stderr
    with fitz.open(output) as document:
        text = "".join(page.get_text() for page in document)
    compact = "".join(text.split())
    assert "probe_started 事件未收到" in text
    assert "完整诊断快照与真实消息" in compact
    assert "ground_1,ground_4,ground_0" in text
    assert "本轮公开抽签" in text
    assert "800×800" in text and "rgb8" in text
    assert "/uav/laser_state" in text and "false" in text
    assert "平台记录：failed；成绩=null；平台耗时=112.172秒" in compact
    assert "实际回调未验证" not in text
    assert "官方完整运行录像：尚未取得" in text
    manifest = json.loads(output.with_suffix(".manifest.json").read_text(encoding="utf-8"))
    assert manifest["mode"] == "draft" and manifest["official"] is None
    for path in (callback, summary, record):
        assert any(item["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
                   for item in manifest["validation"]["inputs"])


def test_supplied_probe_summary_must_match_raw_callback(tmp_path):
    import importlib.util
    import json
    callback, summary, _ = probe_material_fixture(tmp_path)
    value = json.loads(summary.read_text(encoding="utf-8"))
    value["source_evidence"]["sha256"] = "0" * 64
    summary.write_text(json.dumps(value), encoding="utf-8")
    spec = importlib.util.spec_from_file_location("submission_summary_join", SCRIPT)
    media = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(media)
    with pytest.raises(ValueError, match="summary.*match|match.*summary"):
        media.collect_validation(probe_evidence=callback, probe_summary=summary,
                                 probe_output=tmp_path / "analysis-output")


def test_valid_probe_record_cannot_open_official_materials_mode(tmp_path):
    import importlib.util
    _, _, record = probe_material_fixture(tmp_path)
    spec = importlib.util.spec_from_file_location("submission_probe_official_gate", SCRIPT)
    media = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(media)
    with pytest.raises(ValueError, match="diagnostic"):
        media.validate_official_inputs([record], tmp_path / "not-a-platform-video.mp4")


def test_probe_record_must_reference_the_same_callback(tmp_path):
    import importlib.util
    import json
    callback, _, record = probe_material_fixture(tmp_path)
    value = json.loads(record.read_text(encoding="utf-8"))
    value["evidence_paths"] = []
    record.write_text(json.dumps(value), encoding="utf-8")
    spec = importlib.util.spec_from_file_location("submission_probe_record_join", SCRIPT)
    media = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(media)
    with pytest.raises(ValueError, match="same callback"):
        media.collect_validation(probe_evidence=callback, probe_run_record=record,
                                 probe_output=tmp_path / "analysis-output")


def test_official_cli_rejects_offline_record_and_missing_video(tmp_path):
    import json
    record = tmp_path / "offline.json"
    record.write_text(json.dumps({"kind": "offline"}), encoding="utf-8")
    output = tmp_path / "official.pdf"
    result = run_cli("--mode", "official", "--pdf", output,
                     "--run-record", record, "--video-input", tmp_path / "missing.mp4")
    assert result.returncode != 0
    assert not output.exists()


def test_official_video_must_belong_to_the_same_valid_settled_complete_run(tmp_path, monkeypatch):
    import hashlib
    import importlib.util
    import json
    from test_run_records import scored_record
    spec = importlib.util.spec_from_file_location("submission_media_gate", SCRIPT)
    media = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(media)
    artifact = tmp_path / "source.zip"
    artifact.write_bytes(b"synthetic unit-test source")
    evidence = tmp_path / "settlement.json"
    evidence.write_text("synthetic unit-test evidence", encoding="utf-8")
    video = tmp_path / "synthetic.mp4"
    video.write_bytes(b"synthetic unit-test container")
    digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
    complete = scored_record("synthetic-complete")
    failed = scored_record("synthetic-failed", status="failed")
    failed["scores"] = {"items": dict.fromkeys(failed["scores"]["items"]), "total": None, "manual": None}
    for item in (complete, failed):
        item["source"] = item["script"] = {"path": str(artifact), "sha256": digest}
    complete["evidence_paths"] = [str(evidence)]
    failed["evidence_paths"] = [str(video)]
    paths = []
    for name, item in (("complete", complete), ("failed", failed)):
        path = tmp_path / (name + ".json")
        path.write_text(json.dumps(item), encoding="utf-8")
        paths.append(path)
    # Codec behavior is external; the reviewed bug is the cross-record evidence join.
    monkeypatch.setattr(media, "probe_video", lambda path: media.file_evidence(path))
    with pytest.raises(ValueError, match="same|settled|valid"):
        media.validate_official_inputs(paths, video)
