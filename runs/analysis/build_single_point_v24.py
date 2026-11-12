"""Build the reviewed single-point practice package over the frozen v23 archive."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = Path('src/inspection_mission/inspection_mission')
FROZEN_ROUTE_SHA = '5ab108c4128db8b32c18cee084fce636e8ce80700f18dd37e468006a2a58b945'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--report-url', required=True)
    parser.add_argument('--variant', choices=('v24', 'v24b', 'v25'), default='v24')
    args = parser.parse_args()
    if not args.report_url.startswith('https://') or any(c in args.report_url for c in '\r\n"$`\\'):
        raise ValueError('explicit HTTPS callback URL required')
    frozen = ROOT / 'dist/inspection_ground_route_v23_src.zip'
    route = ROOT / PACKAGE / 'ground_observation_route.py'
    if sha(route) != FROZEN_ROUTE_SHA:
        raise ValueError('the reviewed observation route changed')
    additions = [ROOT / PACKAGE / name for name in (
        'ground_single_point.py', 'ground_matcher_worker.py', 'ground_plane_matcher.py')]
    additions += sorted((ROOT / PACKAGE / 'assets/ground_templates').iterdir())
    if len(additions) != 12 or any(not p.is_file() for p in additions):
        raise ValueError('three modules, eight original images and one manifest required')
    archive = ROOT / ('dist/inspection_single_point_'+args.variant+'_src.zip')
    script = ROOT / ('dist/run_single_point_'+args.variant+'.sh')
    if archive.exists() or script.exists():
        raise ValueError('versioned artifacts already exist; preserve them')
    with zipfile.ZipFile(frozen) as base, zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=9) as out:
        for item in base.infolist():
            if item.filename not in {p.relative_to(ROOT).as_posix() for p in additions}:
                out.writestr(item, base.read(item.filename))
        for path in additions:
            out.write(path, path.relative_to(ROOT).as_posix())
    source = (ROOT / 'scripts/run_ground_single_point.sh').read_text(encoding='utf-8').replace('\r\n', '\n')
    source = source.replace('export PYTHONUNBUFFERED=1\n', 'export PYTHONUNBUFFERED=1\nexport INSPECTION_DIAGNOSTIC_URL="'+args.report_url+'"\n', 1)
    with script.open('x', encoding='utf-8', newline='\n') as handle:
        handle.write(source)
    if archive.stat().st_size > 20*1024*1024 or not source.startswith('#!/usr/bin/env bash\n'):
        raise ValueError('practice upload contract mismatch')
    manifest = {'version':'single-point-'+args.variant, 'mode':'practice',
        'official_task_success':False, 'revision':'ground_single_point_v1',
        'basis':'frozen v23 archive plus independently reviewed single-point, worker, matcher and public image assets',
        'source_files_sha256':{p.relative_to(ROOT).as_posix():sha(p) for p in [route,*additions]},
        'artifacts':[{'path':p.relative_to(ROOT).as_posix(),'bytes':p.stat().st_size,'sha256':sha(p)} for p in (archive,script)]}
    (ROOT / ('runs/evidence/20261112/single_point_'+args.variant+'_artifacts.json')).write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(manifest, ensure_ascii=True))


if __name__ == '__main__':
    main()
