"""Interactive first-party HTTP session; credentials remain only in memory.

This support utility uses the login contract published by the platform web UI.
It does not read browser storage and never writes authentication responses.
"""
from __future__ import annotations

import getpass
import http.cookiejar
import json
import mimetypes
import sys
import urllib.error
import urllib.request
import uuid
from pathlib import Path


BASE = "https://www.splenach.com"


def practice_upload_body(fields, files, workspace):
    """Encode the website's two-file practice form without changing file bytes."""
    if fields.get("scenarioKey") != "inspection":
        raise ValueError("Only the authorized inspection practice scenario is supported")
    if set(fields) - {"scenarioKey", "note"}:
        raise ValueError("Unknown practice form fields")
    if set(files) != {"file", "scriptFile"}:
        raise ValueError("The practice form requires exactly two files")
    workspace = Path(workspace).resolve()
    boundary = "inspection-practice-" + uuid.uuid4().hex
    parts = []
    for name, value in fields.items():
        if not isinstance(value, str):
            raise ValueError("Practice form fields must be strings")
        parts.append((f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n'
                      + value).encode("utf-8") + b"\r\n")
    for field, raw_path in files.items():
        path = Path(raw_path).resolve()
        if not path.is_relative_to(workspace):
            raise ValueError("Upload files must be inside the project workspace")
        if any(char in path.name for char in ('"', '\r', '\n')):
            raise ValueError("Invalid upload filename")
        if field == "scriptFile" and path.suffix != ".sh":
            raise ValueError("Startup script must be a .sh file")
        if field == "file" and not path.name.endswith((".zip", ".tar.gz", ".tgz", ".tar")):
            raise ValueError("Source file must be a workspace archive")
        if path.stat().st_size > 20 * 1024 * 1024:
            raise ValueError("Upload file exceeds the current 20 MB practice limit")
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        header = (f'--{boundary}\r\nContent-Disposition: form-data; name="{field}"; '
                  f'filename="{path.name}"\r\nContent-Type: {content_type}\r\n\r\n')
        parts.append(header.encode("utf-8") + path.read_bytes() + b"\r\n")
    parts.append(f"--{boundary}--\r\n".encode())
    return b"".join(parts), "multipart/form-data; boundary=" + boundary


def save_evidence(target_name, result):
    if not target_name:
        return
    target = Path(target_name).resolve()
    workspace = Path(__file__).resolve().parents[1]
    if not target.is_relative_to(workspace):
        raise ValueError("Evidence output must be inside the project workspace")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(redact(result), ensure_ascii=False, indent=2), encoding="utf-8")


def redact(value):
    if isinstance(value, dict):
        return {key: ("<redacted>" if any(word in key.lower() for word in
                ("password", "token", "cookie", "secret")) else redact(item))
                for key, item in value.items()}
    if isinstance(value, list):
        return [redact(item) for item in value]
    return value


def main():
    cookies = http.cookiejar.CookieJar()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
                                        urllib.request.HTTPCookieProcessor(cookies))
    phone = getpass.getpass("Platform phone (hidden): ")
    password = getpass.getpass("Platform password (hidden): ")
    payload = json.dumps({"phone": phone, "password": password,
                          "role": "student", "platform": "ros"}).encode()
    try:
        request = urllib.request.Request(BASE + "/api/auth/login", data=payload,
                  headers={"Content-Type": "application/json", "User-Agent": "Mozilla/5.0"})
        with opener.open(request, timeout=30) as response:
            login = json.load(response)
    except urllib.error.HTTPError as error:
        print(json.dumps({"authenticated": False, "status": error.code,
                          "error": error.read().decode("utf-8", "replace")[:500]}), flush=True)
        return 1
    finally:
        del phone, password, payload
    token = login.get("token")
    if not login.get("success") or not token:
        print(json.dumps({"authenticated": False, "response": redact(login)}, ensure_ascii=True), flush=True)
        return 1
    del login
    print('{"authenticated": true, "credentials_persisted": false}', flush=True)
    for line in sys.stdin:
        try:
            command = json.loads(line)
            if command.get("action") == "exit":
                break
            path = command["path"]
            if not path.startswith("/api/") or "://" in path:
                raise ValueError("Only this platform's observed API paths are accepted")
            body = command.get("body")
            content_type = "application/json"
            encoded = json.dumps(body, ensure_ascii=False).encode() if body is not None else None
            if command.get("files"):
                if path != "/api/ros-simulation/evaluation" or command.get("method") != "POST":
                    raise ValueError("File upload is restricted to the observed practice evaluation form")
                encoded, content_type = practice_upload_body(body or {}, command["files"],
                                                            Path(__file__).resolve().parents[1])
            headers = {"Authorization": "Bearer " + token, "X-Auth-Role": "student",
                       "Content-Type": content_type, "User-Agent": "Mozilla/5.0"}
            request = urllib.request.Request(BASE + path, data=encoded,
                      headers=headers, method=command.get("method", "GET"))
            with opener.open(request, timeout=120 if command.get("files") else 45) as response:
                result = json.load(response)
                status = response.status
            cleaned = redact(result)
            save_evidence(command.get("save"), cleaned)
            print(json.dumps({"status": status, "path": path, "response": cleaned}, ensure_ascii=True)[:command.get("limit", 12000)], flush=True)
        except urllib.error.HTTPError as error:
            raw_error = error.read().decode("utf-8", "replace")
            try:
                failed = {"status": error.code, "response": redact(json.loads(raw_error))}
            except ValueError:
                failed = {"status": error.code, "error": raw_error[:1000]}
            save_evidence(command.get("save"), failed)
            print(json.dumps(failed, ensure_ascii=True), flush=True)
        except Exception as error:
            print(json.dumps({"error": str(error)}, ensure_ascii=True), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
