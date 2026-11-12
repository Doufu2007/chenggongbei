"""Independent transport and source-binding review; never operates the platform."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src" / "inspection_mission"), str(ROOT / "tests")]

import pytest

from inspection_mission import ground_single_point as production
from inspection_mission.telemetry import DiagnosticReporter
from test_ground_single_point import SingleRun, fixture_jobs
from test_bounded_ground_probe import install_ros_transport


def actual_source_results(*, encoding="bgr8", padding=0):
    run = SingleRun().until(lambda item: item.guard.state == "RECOGNIZE_WAIT")
    _, results = fixture_jobs(run)
    channels = 1 if encoding == "mono8" else 3
    for index, frame in enumerate(run.guard.capture_frames):
        msg = deepcopy(frame["msg"])
        msg.width = msg.height = 800
        msg.encoding = encoding
        msg.step = 800 * channels + padding
        color = bytes([37 + index]) if channels == 1 else bytes([37 + index, 83, 151])
        msg.data = (color * 800 + b"\x5a" * padding) * 800
        frame["msg"] = msg
    jobs = production.prepare_capture_jobs(run.guard.capture_frames,
        session_id=run.guard.session_id, first_id=run.guard.target_id,
        epoch=run.guard.stationary_epoch, first_seq=100)
    assert run.guard.register_prepared_jobs(jobs, epoch=run.guard.stationary_epoch)
    spec = next(item for item in production.load_ground_templates() if item.class_id == "trapped_people")
    for result, job in zip(results, jobs):
        result["job"] = {key: deepcopy(value) for key, value in job.items() if key != "source_jpeg"}
        result["matcher"]["candidates"][0].update(
            template_sha256=spec.expected_sha256, provenance=deepcopy(spec.provenance))
    return run, jobs, results


def test_all_real_hook_events_fit_bounded_emitter_with_full_public_provenance_and_status():
    run, jobs, results = actual_source_results()
    for result in results:
        assert run.guard.accept_match_result(result, wall_s=run.wall)
    for index in range(20):
        run.guard.observe_public("/referee/recognition_status",
            {"data": str(index)+"中"*10000}, wall_s=run.wall)
    events = run.guard.additional_events()
    failures = []
    sizes = []
    for event, data in events:
        try:
            payload = DiagnosticReporter._encode(event, data)
            sizes.append({"section": data.get("section"), "bytes": len(payload)})
        except ValueError as error:
            failures.append({"section": data.get("section"), "error": str(error),
                "data_json_bytes": len(json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8"))})
    sha = hashlib.sha256(Path(production.__file__).read_bytes()).hexdigest()
    output = Path(__file__).with_name("real_emitter_sizes_"+sha[:12]+".json")
    output.write_text(json.dumps({"sizes": sizes, "failures": failures}, ensure_ascii=False, indent=2), encoding="utf-8")
    assert not failures


def test_real_report_final_does_not_reintroduce_oversized_embedded_duplicate(monkeypatch):
    install_ros_transport(monkeypatch)
    run, jobs, results = actual_source_results()
    for result in results:
        assert run.guard.accept_match_result(result, wall_s=run.wall)
    for index in range(20):
        run.guard.observe_public("/referee/recognition_status",
            {"data": str(index)+"中"*10000}, wall_s=run.wall)
    events = []
    class Sink:
        def send_now(self, event, data, timeout_s):
            DiagnosticReporter._encode(event, data)
            events.append((event, deepcopy(data)))
            return True
    node = production.create_single_point_node(Sink(), wall_clock=lambda: run.wall)
    node.guard = run.guard
    node.report_final()
    errors = [error for _, data in events if data.get("section") == "evidence_errors"
              for error in data.get("errors", []) if error.get("section") == "survey"
              and error.get("topic") == "approach"]
    node.close_watchdog()
    assert not errors


@pytest.mark.parametrize("encoding,padding", [("bgr8", 0), ("rgb8", 8), ("mono8", 8)])
def test_original_source_metadata_is_unchanged_in_submitted_job(encoding, padding):
    run, jobs, _ = actual_source_results(encoding=encoding, padding=padding)
    for job, frame in zip(jobs, run.guard.capture_frames):
        assert job["image_stamp_s"] == frame["record"]["stamp_s"]
        assert job["frame_id"] == frame["record"]["frame_id"]
        assert job["topic"] == frame["record"]["topic"]
        assert job["camera_info"] == frame["camera_info"]
        assert job["pose"] == frame["record"]["odom"]
        assert job["source_ros_encoding"] == encoding
        assert job["source_ros_stride"] == frame["msg"].step
        assert job["source_ros_sha256"] == hashlib.sha256(frame["msg"].data).hexdigest()
        assert job["source_sha256"] == hashlib.sha256(job["source_jpeg"]).hexdigest()


def test_emitted_success_evidence_retains_original_job_metadata_for_independent_rebinding():
    run, jobs, results = actual_source_results()
    for result in results:
        assert run.guard.accept_match_result(result, wall_s=run.wall)
    core = run.guard.additional_events()[0][1]
    serialized = json.dumps(core, ensure_ascii=False)
    # These are factual source fields already supplied to the real worker.
    # A durable record must retain them, not only the derived JPEG SHA.
    for job in jobs:
        assert job["source_ros_sha256"] in serialized
    assert '"camera_info"' in serialized and '"source_ros_stride"' in serialized


def node_at_ready_state(monkeypatch):
    install_ros_transport(monkeypatch)
    wall = [0.]
    node = production.create_single_point_node(wall_clock=lambda: wall[0])
    run, jobs, results = actual_source_results()
    for result in results:
        assert run.guard.accept_match_result(result, wall_s=run.wall)
    node.guard = run.guard
    wall[0] = run.wall
    return node, run, wall


def test_no_subscriber_preflight_is_not_labeled_as_actual_publisher_invocation(monkeypatch):
    node, run, wall = node_at_ready_state(monkeypatch)
    calls = []
    node.publisher_by_topic["/inspection/detections"] = SimpleNamespace(
        get_subscription_count=lambda: 0, publish=lambda message: calls.append(message.data))
    node.tick()
    endpoint = node.command_endpoints["/inspection/detections"]
    assert calls == [] and run.guard.attempts["recognition"]["attempted"] is True
    assert run.guard.attempts["recognition"]["sent"] is False
    assert endpoint["actual_publish_attempt_count"] == len(calls)
    node.close_watchdog()


def test_successful_ros_string_transport_is_exactly_once_and_counted(monkeypatch):
    node, run, wall = node_at_ready_state(monkeypatch)
    calls = []
    node.publisher_by_topic["/inspection/detections"] = SimpleNamespace(
        get_subscription_count=lambda: 1, publish=lambda message: calls.append(message.data))
    for _ in range(3):
        node.tick()
    endpoint = node.command_endpoints["/inspection/detections"]
    assert calls == [production.recognition_phrase("ground", "被困人员")]
    assert endpoint["actual_publish_attempt_count"] == endpoint["actual_publish_return_count"] == 1
    assert not run.guard.attempts["recognition"]["official_acceptance"]
    node.close_watchdog()


def test_raising_publisher_call_counts_one_invocation_and_zero_returns(monkeypatch):
    node, run, wall = node_at_ready_state(monkeypatch)
    calls = []
    def broken(message):
        calls.append(message.data)
        raise RuntimeError("publication uncertain after invocation")
    node.publisher_by_topic["/inspection/detections"] = SimpleNamespace(
        get_subscription_count=lambda: 1, publish=broken)
    for _ in range(3):
        node.tick()
    endpoint = node.command_endpoints["/inspection/detections"]
    assert len(calls) == 1
    assert endpoint["actual_publish_attempt_count"] == 1
    assert endpoint["actual_publish_return_count"] == 0
    assert run.guard.state == "STOP" and run.guard.attempts["recognition"]["sent"] is False
    node.close_watchdog()


def test_matched_context_worker_failure_remains_bound_in_final_evidence():
    run, jobs, _ = actual_source_results()
    original = {key: deepcopy(value) for key, value in jobs[0].items() if key != "source_jpeg"}
    result = {"matches_current_context": True, "job": original, "job_was_dispatched": True,
        "job_was_executed": None, "error": "worker_process_exited: -1"}
    class Worker:
        def cancel_context(self): pass
        def set_context(self, *args): pass
        def poll(self): return [deepcopy(result)]
    coordinator = production.GroundSinglePointCoordinator(run.guard, Worker(), prepare=lambda *args, **kwargs: [])
    coordinator.poll_once(run.wall)
    assert run.guard.state == "STOP" and "single_point_matcher_failed:" in run.guard.reason
    core = run.guard.additional_events()[0][1]
    serialized = json.dumps(core, ensure_ascii=False)
    assert original["source_sha256"] in serialized
    assert '"job_was_executed": null' in serialized

