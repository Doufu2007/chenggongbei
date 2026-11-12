import json
from email import policy
from email.parser import BytesParser

import pytest

from tools import platform_session


def test_practice_upload_preserves_binary_and_unicode_files(tmp_path):
    source = tmp_path / "源码.zip"
    script = tmp_path / "run.sh"
    source.write_bytes(b"PK\x03\x04\x00\xff\r\n\x00")
    script.write_bytes(b"#!/bin/bash\nprintf 'test\\n'\n")
    data, content_type = platform_session.practice_upload_body(
        {"scenarioKey": "inspection", "note": "练习，不是正式提交"},
        {"file": str(source), "scriptFile": str(script)}, tmp_path)
    message = BytesParser(policy=policy.default).parsebytes(
        b"Content-Type: " + content_type.encode() + b"\r\n\r\n" + data)
    parts = {part.get_param("name", header="content-disposition"): part
             for part in message.iter_parts()}
    assert parts["file"].get_payload(decode=True) == source.read_bytes()
    assert parts["file"].get_filename() == "源码.zip"
    assert parts["scriptFile"].get_payload(decode=True) == script.read_bytes()
    assert parts["note"].get_payload(decode=True).decode() == "练习，不是正式提交"


def test_practice_upload_refuses_outside_workspace_and_unknown_fields(tmp_path):
    source = tmp_path / "src.zip"
    script = tmp_path / "run.sh"
    source.write_bytes(b"PK")
    script.write_text("#!/bin/bash\n")
    files = {"file": str(source), "scriptFile": str(script)}
    with pytest.raises(ValueError, match="workspace"):
        platform_session.practice_upload_body({"scenarioKey": "inspection"}, files, tmp_path / "child")
    with pytest.raises(ValueError, match="fields"):
        platform_session.practice_upload_body({"scenarioKey": "inspection", "teamId": "x"}, files, tmp_path)


def test_practice_upload_requires_inspection_source_and_script(tmp_path):
    with pytest.raises(ValueError, match="inspection"):
        platform_session.practice_upload_body({"scenarioKey": "delivery"}, {}, tmp_path)
    with pytest.raises(ValueError, match="two"):
        platform_session.practice_upload_body({"scenarioKey": "inspection"}, {}, tmp_path)


def test_evidence_does_not_persist_auth_fields(tmp_path):
    result = platform_session.redact({"token": "auth", "nested": {"password": "pw"}, "score": None})
    assert "auth" not in json.dumps(result)
    assert result["score"] is None
