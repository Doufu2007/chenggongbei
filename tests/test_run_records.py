"""Synthetic unit inputs are never saved as real platform run records."""

import copy
import importlib
import json
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "run_records.py"
ITEMS = {
    "ground_1": 10, "ground_2": 10, "ground_3": 10,
    "window_1": 10, "aerial_1": 15, "aerial_2": 15,
    "window_2": 10, "landing": 10,
}


def api():
    assert TOOL.is_file(), "offline run record CLI has not been implemented"
    return importlib.import_module("tools.run_records")


def synthetic_record(run_id="synthetic-unit-run", **changes):
    record = {
        "schema_version": 1,
        "record_id": run_id or "synthetic-missing-id",
        "runId": run_id,
        "kind": "official_task",
        "evaluation_mode": "practice",
        "condition_id": "synthetic-condition-1",
        "variant": "baseline",
        "status": "failed",
        "full_mission_completed": None,
        "source": {"path": None, "sha256": None},
        "script": {"path": None, "sha256": None},
        "draw_order": None,
        "scores": {"items": dict.fromkeys(ITEMS), "total": None, "manual": None},
        "duration_s": 43.072,
        "failure_reason": "no_score_extracted",
        "evidence_paths": ["synthetic-unit-evidence.json"],
    }
    record.update(changes)
    return record


def scored_record(run_id="synthetic-scored", total=90, **changes):
    record = synthetic_record(run_id, status="completed", full_mission_completed=True,
                              failure_reason=None)
    record["scores"] = {"items": copy.deepcopy(ITEMS), "total": total, "manual": None}
    if total != 90:
        record["scores"]["items"] = dict.fromkeys(ITEMS)
    record.update(changes)
    return record


def test_scoreless_failed_attempt_stays_in_completion_denominator():
    result = api().summarize_records([synthetic_record(), scored_record()])
    assert result["task_attempts"] == 2
    assert result["completed_runs"] == 1
    assert result["completion_rate"] == 0.5
    assert result["minimum_valid_total"] == 90
    assert result["failure_reasons"] == {"no_score_extracted": 1}


def test_completed_status_without_score_is_neither_settlement_nor_completion():
    record = synthetic_record(status="completed", full_mission_completed=True, failure_reason=None)
    result = api().summarize_records([record])
    assert result["completed_runs"] == 0
    assert result["valid_settled_runs"] == 0
    assert result["minimum_valid_total"] is None
    assert result["completion_rate"] == 0.0


def test_valid_score_does_not_prove_full_mission_completion():
    result = api().summarize_records([scored_record(full_mission_completed=None)])
    assert result["valid_settled_runs"] == 1
    assert result["completed_runs"] == 0
    assert result["unknown_completion"] == 1


def test_duplicate_run_id_is_counted_once_and_conflict_is_rejected():
    record = scored_record()
    result = api().summarize_records([record, copy.deepcopy(record)])
    assert result["task_attempts"] == 1
    assert result["duplicate_count"] == 1
    conflict = copy.deepcopy(record)
    conflict["duration_s"] = 99
    with pytest.raises(ValueError, match="conflicting.*runId"):
        api().summarize_records([record, conflict])


def test_missing_run_id_failure_stays_in_denominator_and_cannot_complete():
    result = api().summarize_records([scored_record(run_id=None)])
    assert result["task_attempts"] == 1
    assert result["completed_runs"] == 0
    assert result["missing_run_id"] == 1
    assert result["failure_reasons"]["missing_run_id"] == 1


def test_offline_and_probe_do_not_pollute_task_metrics():
    probe = synthetic_record("synthetic-probe", kind="interface_probe")
    offline = synthetic_record("synthetic-offline", kind="offline", evaluation_mode=None)
    result = api().summarize_records([probe, offline])
    assert result["task_attempts"] == 0
    assert result["completion_rate"] is None
    assert result["minimum_valid_total"] is None
    assert result["excluded_counts"] == {"interface_probe": 1, "offline": 1}
    assert result["data_status"] == "no_task_attempts"


def test_pending_tasks_are_reported_without_pretending_a_final_result():
    result = api().summarize_records([synthetic_record(status="running", failure_reason=None)])
    assert result["pending_tasks"] == 1
    assert result["task_attempts"] == 0
    assert result["completion_rate"] is None


def test_manual_score_is_separate_and_unknown_items_are_preserved():
    record = scored_record(total=75, full_mission_completed=None)
    record["scores"]["manual"] = 30
    checked = api().validate_record(record)
    assert checked["scores"]["items"]["ground_1"] is None
    result = api().summarize_records([checked])
    assert result["minimum_valid_total"] == 75
    assert result["manual_scores"] == {"known_count": 1, "minimum": 30, "maximum": 30}


def test_known_zero_score_is_not_changed_to_unknown_or_inferred_as_full_score():
    record = scored_record(total=0, full_mission_completed=False)
    result = api().summarize_records([record])
    assert result["valid_settled_runs"] == 1
    assert result["minimum_valid_total"] == 0
    assert result["completed_runs"] == 0


@pytest.mark.parametrize("field,value", [
    ("total", 90.01), ("total", -1), ("total", True),
    ("total", float("nan")), ("manual", 30.01), ("manual", -1),
])
def test_score_bounds_and_finite_numbers_are_enforced(field, value):
    record = synthetic_record()
    record["scores"][field] = value
    with pytest.raises(ValueError, match="scores"):
        api().validate_record(record)


def test_item_bounds_and_total_consistency_are_checked():
    record = scored_record()
    record["scores"]["items"]["aerial_1"] = 16
    with pytest.raises(ValueError, match="aerial_1"):
        api().validate_record(record)
    record = scored_record()
    record["scores"]["total"] = 89
    with pytest.raises(ValueError, match="sum"):
        api().validate_record(record)
    record["scores"]["items"]["landing"] = None
    api().validate_record(record)  # A partial breakdown cannot imply a missing item value.


def test_missing_total_is_never_computed_from_item_scores():
    record = scored_record()
    record["scores"]["total"] = None
    checked = api().validate_record(record)
    assert checked["scores"]["total"] is None
    assert api().summarize_records([checked])["minimum_valid_total"] is None


def test_optional_physical_ground_ids_preserve_ground_zero_without_relabelling_or_double_count():
    record = scored_record(total=10, full_mission_completed=False)
    record['scores']['items']['ground_1'] = 10
    physical = {'ground_0':10,'ground_1':None,'ground_2':None,'ground_3':None,'ground_4':None}
    record['scores']['ground_points'] = copy.deepcopy(physical)
    checked = api().validate_record(record)
    assert checked['scores']['ground_points'] == physical
    assert checked['scores']['items']['ground_1'] == 10
    assert checked['scores']['total'] == 10
    assert api().summarize_records([checked])['completed_runs'] == 0
    assert 'ground_points'not in api().validate_record(scored_record())['scores']


@pytest.mark.parametrize('physical',[
    {'ground_5':10},{'ground_zero':10},[],None,{'ground_0':10.01},
    {'ground_0':-1},{'ground_0':True},{'ground_0':float('nan')},{'ground_0':float('inf')},
])
def test_physical_ground_ids_reject_unknown_point_and_invalid_score(physical):
    record = scored_record(total=10)
    record['scores']['ground_points'] = physical
    with pytest.raises(ValueError,match='scores.ground_points'):
        api().validate_record(record)


def test_physical_ground_breakdown_neither_invents_total_nor_contradicts_known_total():
    record = scored_record(total=10)
    record['scores']['ground_points'] = {'ground_0':10,'ground_4':10}
    with pytest.raises(ValueError,match='scores.total'):
        api().validate_record(record)
    record['scores']['total'] = None
    checked = api().validate_record(record)
    assert checked['scores']['total']is None
    assert checked['scores']['items']['ground_1']is None


def test_partial_items_cannot_contradict_the_total_upper_bound():
    record = scored_record()
    record["scores"]["items"] = dict.fromkeys(ITEMS)
    record["scores"]["items"]["ground_1"] = 0
    with pytest.raises(ValueError, match="scores.total"):
        api().validate_record(record)


def test_missing_evidence_prevents_verified_completion_but_preserves_attempt():
    result = api().summarize_records([scored_record(evidence_paths=[])])
    assert result["task_attempts"] == 1
    assert result["completed_runs"] == 0
    assert result["failure_reasons"] == {"evidence_missing": 1}


def test_duration_distribution_only_uses_valid_settlements_and_reports_missing():
    records = [scored_record("synthetic-a", duration_s=10),
               scored_record("synthetic-b", duration_s=30),
               scored_record("synthetic-c", duration_s=None),
               synthetic_record("synthetic-failure", duration_s=1000)]
    distribution = api().summarize_records(records)["valid_duration_s"]
    assert distribution == {"n": 2, "missing": 1, "min": 10, "median": 20,
                            "p95": 30, "max": 30}


def test_compare_pairs_only_matching_conditions_and_excludes_diagnostics():
    records = [scored_record("synthetic-a", total=70, duration_s=40),
               scored_record("synthetic-b", total=80, duration_s=35, variant="candidate"),
               scored_record("synthetic-unmatched", total=90, variant="candidate",
                             condition_id="synthetic-other")]
    records.append(synthetic_record("synthetic-probe", kind="interface_probe", variant="candidate"))
    result = api().compare_records(records, "baseline", "candidate")
    assert result["paired_conditions"] == 1
    assert result["pairs"][0]["score_delta"] == 10
    assert result["pairs"][0]["duration_delta_s"] == -5
    assert result["unmatched_conditions"] == ["synthetic-other"]
    assert result["data_status"] == "insufficient_pairs"


def test_compare_retains_failed_partner_without_fabricating_zero_delta():
    records = [scored_record(), synthetic_record("synthetic-b", variant="candidate")]
    result = api().compare_records(records, "baseline", "candidate")
    assert result["paired_conditions"] == 1
    assert result["pairs"][0]["score_delta"] is None
    assert result["pairs"][0]["candidate_completed"] is False
    assert result["score_pairs"] == 0


def test_compare_rejects_self_comparison_and_does_not_arbitrarily_pair_repeats():
    records = [scored_record("synthetic-a"), scored_record("synthetic-b"),
               scored_record("synthetic-c", variant="candidate")]
    with pytest.raises(ValueError, match="different"):
        api().compare_records(records, "baseline", "baseline")
    result = api().compare_records(records, "baseline", "candidate")
    assert result["paired_conditions"] == 0
    assert result["ambiguous_conditions"] == ["synthetic-condition-1"]


@pytest.mark.parametrize("changes", [
    {"evaluation_mode": "competition"}, {"draw_order": ["public-point-b", "public-point-a"]},
])
def test_compare_does_not_pair_different_modes_or_observed_draw_orders(changes):
    records = [scored_record("synthetic-a", draw_order=["public-point-a", "public-point-b"]),
               scored_record("synthetic-b", variant="candidate", **changes)]
    result = api().compare_records(records, "baseline", "candidate")
    assert result["paired_conditions"] == 0
    assert result["incompatible_conditions"] == ["synthetic-condition-1"]


@pytest.mark.parametrize("mutation", [
    {"schema_version": 2}, {"schema_version": True}, {"duration_s": -1},
    {"kind": "practice"}, {"evaluation_mode": None},
    {"source": {"path": "package.zip", "sha256": "not-a-sha256"}},
    {"evidence_paths": ["https://example.test/?token=secret"]},
    {"password": "must-not-be-stored"},
    {"evaluation_mode": []},
])
def test_invalid_schema_and_secret_bearing_payloads_are_rejected(mutation):
    with pytest.raises(ValueError):
        api().validate_record(synthetic_record(**mutation))


def test_real_probe_import_is_explicit_and_preserves_unknown_values():
    path = ROOT / "docs" / "platform_probe_result_20261106.json"
    before = path.read_bytes()
    record = api().import_probe(path, evidence_path="docs/platform_probe_result_20261106.json")
    assert record["runId"] == "run_20261106_005323_885f0a05"
    assert record["kind"] == "interface_probe"
    assert record["evaluation_mode"] == "practice"
    assert record["duration_s"] == 43.072
    assert record["scores"]["total"] is None
    assert record["draw_order"] is None
    assert record["source"]["sha256"] is None
    assert record["full_mission_completed"] is None
    assert "tabLogs" not in record
    assert api().summarize_records([record])["task_attempts"] == 0
    assert path.read_bytes() == before


def cli(*args):
    return subprocess.run([sys.executable, str(TOOL), *map(str, args)], cwd=ROOT,
                          text=True, encoding="utf-8", capture_output=True, check=False)


def test_cli_validation_summary_compare_and_hash_are_offline(tmp_path):
    api()
    path = tmp_path / "synthetic-only.json"
    path.write_text(json.dumps([scored_record(), synthetic_record("synthetic-b", variant="candidate")]),
                    encoding="utf-8")
    result = cli("validate", path)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["record_count"] == 2
    result = cli("summary", path)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["completion_rate"] == 0.5
    result = cli("compare", path, "--baseline", "baseline", "--candidate", "candidate")
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["paired_conditions"] == 1
    artifact = tmp_path / "artifact.txt"
    artifact.write_bytes(b"abc")
    result = cli("hash", artifact)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["sha256"] == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"


def test_cli_rejects_invalid_json_and_conflicting_records_without_partial_summary(tmp_path):
    api()
    path = tmp_path / "bad.json"
    path.write_text("{not-json", encoding="utf-8")
    result = cli("summary", path)
    assert result.returncode == 2
    assert not result.stdout
    conflict = scored_record(duration_s=1)
    path.write_text(json.dumps([scored_record(), conflict]), encoding="utf-8")
    result = cli("summary", path)
    assert result.returncode == 2
    assert "conflicting" in result.stderr
    assert not result.stdout


def test_cli_probe_import_writes_only_explicit_output_and_refuses_overwrite(tmp_path):
    output = tmp_path / "real-probe.json"
    result = cli("import-probe", ROOT / "docs" / "platform_probe_result_20261106.json",
                 "--output", output, "--evidence-path", "docs/platform_probe_result_20261106.json")
    assert result.returncode == 0, result.stderr
    assert json.loads(output.read_text(encoding="utf-8"))["scores"]["total"] is None
    before = output.read_bytes()
    result = cli("import-probe", ROOT / "docs" / "platform_probe_result_20261106.json", "--output", output)
    assert result.returncode == 2
    assert result.stderr and result.stderr.startswith("run_records:")
    assert output.read_bytes() == before


def test_file_verification_catches_wrong_artifact_hash(tmp_path):
    api()
    artifact = tmp_path / "source.zip"
    artifact.write_bytes(b"abc")
    evidence = tmp_path / "evidence.json"
    evidence.write_text("{}", encoding="utf-8")
    record = synthetic_record(source={"path": str(artifact), "sha256": "0" * 64},
                              evidence_paths=[str(evidence)])
    path = tmp_path / "synthetic.json"
    path.write_text(json.dumps(record), encoding="utf-8")
    result = cli("validate", path, "--verify-files")
    assert result.returncode == 2
    assert "sha256" in result.stderr
