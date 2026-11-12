"""Current production behavior on byte streams; no network or socket binding."""
import json
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from offline_receiver_probe import cases


def test_current_http_outcomes_and_evidence_separation(tmp_path):
    rows=cases(tmp_path)
    assert {r['case']:r['http_status'] for r in rows}=={
        'accepted_launcher':202,'wrong_path':404,'exact_token_query_rejected':404,
        'get_exact_path':405,'invalid_json':400,'content_type':400,'duplicate_length':400,
        'too_large':413,'short_body':400,'body_timeout':400,'sink_error':500,
        'reply_write_error':None,'unsupported_method':501}
    for row in rows:
        count=1 if row['case'] in ('accepted_launcher','reply_write_error') else 0
        assert row['accepted_count']==row['events_count']==count
        assert row['accepted_event_names']==(['launcher_started'] if count else [])
        for accepted in row['accepted']:
            assert set(accepted)=={'received_at','payload','source'}
            assert accepted['source']=='unverified_callback'
    assert next(r for r in rows if r['case']=='reply_write_error')['uncaught_error_type']=='BrokenPipeError'
