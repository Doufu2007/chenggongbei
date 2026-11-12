"""Package the full task entry point and freeze every uploaded source byte."""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'inspection_mission'))


def build():
    from inspection_mission.ground_template_catalog import load_complete_ground_templates
    from inspection_mission.full_flight_inspection import create_flight_inspection_node
    from inspection_mission.ground_three_point_return import create_return_three_point_node
    assert callable(create_flight_inspection_node) and callable(create_return_three_point_node)
    assert len(load_complete_ground_templates()) == 10
    src = ROOT / 'src' / 'inspection_mission'
    paths = [p for p in sorted(src.rglob('*')) if p.is_file()
             and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo')]
    for p in paths:
        if p.suffix == '.py':
            ast.parse(p.read_text(encoding='utf-8'), filename=str(p))
    script_text = (ROOT / 'scripts' / 'run_full_inspection.sh').read_text(encoding='utf-8').replace('\r\n', '\n')
    if 'python3 -u -m inspection_mission.full_inspection_runtime --ground-duration 1800 --flight-duration 900' not in script_text:
        raise ValueError('script must launch the complete pipeline')
    for forbidden in ('gzserver', 'gzclient', 'ros2 launch ican_', 'apt install', 'apt-get'):
        if forbidden in script_text:
            raise ValueError('prohibited environment startup: ' + forbidden)
    dist = ROOT / 'dist'
    dist.mkdir(exist_ok=True)
    archive = dist / 'inspection_full_mission_v41_src.zip'
    manifest = []
    with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for p in paths:
            raw = p.read_bytes()
            name = p.relative_to(ROOT).as_posix()
            z.writestr(name, raw)
            manifest.append({'path': name, 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()})
    if archive.stat().st_size > 50*1024*1024:
        raise ValueError('source archive exceeds platform 50MB limit')
    script = dist / 'run_full_mission_v41.sh'
    script.write_text(script_text, encoding='utf-8', newline='\n')
    result = {'kind': 'full_inspection_practice_candidate', 'full_mission_platform_verified': False,
              'source': {'path': str(archive.relative_to(ROOT)), 'bytes': archive.stat().st_size,
                         'sha256': hashlib.sha256(archive.read_bytes()).hexdigest()},
              'script': {'path': str(script.relative_to(ROOT)), 'sha256': hashlib.sha256(script.read_bytes()).hexdigest()},
              'files': manifest}
    target = dist / 'full_mission_v41_manifest.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k != 'files'}, ensure_ascii=False))
    return result


if __name__ == '__main__':
    build()
