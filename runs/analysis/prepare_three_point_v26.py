"""Reuse the existing tunnel host with a new isolated loopback receiver path."""
from pathlib import Path
import argparse
import getpass
import json
import re
import sys
import urllib.error
import urllib.parse
import urllib.request

from build_three_point_v26 import ROOT, build


parser = argparse.ArgumentParser()
parser.add_argument("--build", action="store_true")
args = parser.parse_args()
receiver = json.loads(getpass.getpass("Receiver metadata (hidden): "))
local = urllib.parse.urlsplit(receiver["local_url"])
if local.scheme != "http" or local.hostname != "127.0.0.1" or local.port != 63974:
    raise ValueError("unexpected isolated receiver")
old = (ROOT / "dist/run_single_point_v25.sh").read_text(encoding="utf-8")
match = re.search(r'^export INSPECTION_DIAGNOSTIC_URL="([^"]+)"$', old, re.MULTILINE)
if match is None:
    raise ValueError("frozen practice callback host absent")
prior = urllib.parse.urlsplit(match.group(1))
if prior.scheme != "https" or not prior.hostname or prior.username or prior.password:
    raise ValueError("unexpected practice tunnel host")
url = urllib.parse.urlunsplit(("https", prior.netloc, local.path, "", ""))
try:
    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(url, timeout=15) as response:
        status = response.status
except urllib.error.HTTPError as error:
    status = error.code
if status != 405:
    raise ValueError("new receiver public reachability failed, HTTP " + str(status))
print(json.dumps({"receiver_get_status": status, "test_event_written": False}), flush=True)
if args.build:
    print(json.dumps(build(url), ensure_ascii=True), flush=True)
