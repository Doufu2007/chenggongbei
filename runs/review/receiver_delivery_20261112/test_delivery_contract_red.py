"""Proposed delivery ledger contracts: intentionally RED on current receiver.

These do not bind sockets, send HTTP, or change production implementation.
"""
import hashlib
from pathlib import Path
import sys
import pytest

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from offline_receiver_probe import FIXTURE_TOKEN,execute,make_request


@pytest.mark.parametrize('case,wire,status,reason',[
    ('wrong_path',make_request(path='/wrong-fixture-path'),404,'path_mismatch'),
    ('method',make_request(method='GET'),405,'method_not_allowed'),
    ('schema',make_request(raw=b'{broken'),400,'event_invalid'),
    ('large',make_request(headers=[('Content-Type','application/json'),('Content-Length','65537')],raw=b''),413,'body_too_large'),
])
def test_rejection_has_separate_sanitized_delivery_ledger(tmp_path,case,wire,status,reason):
    result=execute(tmp_path,case,wire)
    assert result['http_status']==status
    assert result['accepted']==[] and result['events_count']==0
    assert len(result['deliveries'])==1, 'request outcome is invisible after rejection'
    row=result['deliveries'][0]
    assert row['http_status']==status and row['reason']==reason
    assert row['evidence_appended'] is False
    assert row['started_at'] and row['finished_at']
    assert FIXTURE_TOKEN not in str(row)
    assert '/' + FIXTURE_TOKEN not in str(row)
    assert len(row['path_fingerprint'])==16


def test_accepted_payload_keeps_original_schema_and_separate_success_delivery(tmp_path):
    result=execute(tmp_path,'accepted',make_request())
    assert result['http_status']==202 and result['accepted_count']==1
    assert set(result['accepted'][0])=={'received_at','payload','source'}
    assert len(result['deliveries'])==1, 'accepted event lacks a delivery outcome'
    row=result['deliveries'][0]
    assert row['http_status']==202 and row['evidence_appended'] is True
    assert row['reason']=='accepted' and row['response_completed'] is True


def test_response_failure_is_distinct_from_successful_evidence_append(tmp_path):
    result=execute(tmp_path,'broken_reply',make_request(),reply_error=True)
    assert result['accepted_count']==1 and result['events_count']==1
    assert result['uncaught_error_type']=='BrokenPipeError'
    assert len(result['deliveries'])==1, 'client disconnect after persistence is invisible'
    row=result['deliveries'][0]
    assert row['evidence_appended'] is True and row['response_completed'] is False
    assert row['reason']=='response_write_error' and row['exception_type']=='BrokenPipeError'


def test_sink_failure_has_no_evidence_and_explicit_delivery_reason(tmp_path):
    result=execute(tmp_path,'sink_failure',make_request(),sink_error=True)
    assert result['http_status']==500 and result['accepted']==[]
    assert len(result['deliveries'])==1, 'evidence IO error is invisible'
    assert result['deliveries'][0]['reason']=='evidence_io_error'
