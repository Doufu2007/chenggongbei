"""Complete public ground references, including two recovered download failures.

Old experimental catalogs stay frozen. This catalog supplies no run-specific
class or point mapping; each runtime exposure still has to pass the matcher.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from .ground_single_point import GROUND_LABELS, TEMPLATE_COMMIT, TEMPLATE_REPOSITORY, load_ground_templates


RECOVERED = {
    'collapsed_ruins_01.png': ('collapsed_ruins', 'e5ed0aaf0b9b4432c262bec0255da90dbccb30bbf05479b70df1a98dafc169b2',
                              'ec6da6a8bcd3e3428334996b7dce28b34f7ef264', 3270491),
    'damaged_vehicle_02.jpg': ('damaged_vehicle', 'dbb9a48f44f4b63e27ebdba53343af120cae8d4ef27d1cc82a5bf4407d954026',
                              'fd9486e0bfe7d6a8af61d4de8dfa71c640effd9b', 276074),
}


def additional_asset_root():
    return Path(__file__).resolve().parent / 'assets' / 'ground_templates_recovered'


def load_complete_ground_templates(asset_dir=None, *, additional_dir=None):
    from .ground_plane_matcher import TemplateSpec
    specs = load_ground_templates(asset_dir)
    root = (Path(additional_dir) if additional_dir is not None else additional_asset_root()).resolve()
    try:
        manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
    except (FileNotFoundError, json.JSONDecodeError) as exc:
        raise ValueError('missing or invalid recovered template manifest') from exc
    entries = manifest.get('templates', [])
    if (manifest.get('repository') != TEMPLATE_REPOSITORY or manifest.get('commit') != TEMPLATE_COMMIT
            or len(entries) != 2 or {r.get('file') for r in entries} != set(RECOVERED)):
        raise ValueError('explicit recovered two-template manifest required')
    for row in entries:
        name = row['file']
        category, sha256, blob, size = RECOVERED[name]
        path = (root / name).resolve()
        source_path = 'src/ican_inspection_examples/ican_inspection_examples/templates/' + name
        expected_url = 'https://raw.githubusercontent.com/' + TEMPLATE_REPOSITORY + '/' + TEMPLATE_COMMIT + '/' + source_path
        if (path.parent != root or row.get('class_id') != category or row.get('sha256') != sha256
                or row.get('git_blob_sha') != blob or row.get('size_bytes') != size
                or row.get('source_url') != expected_url):
            raise ValueError('recovered template source/class/hash mismatch: ' + name)
        try:
            raw = path.read_bytes()
        except FileNotFoundError as exc:
            raise ValueError('missing recovered template: ' + name) from exc
        actual_blob = hashlib.sha1(b'blob ' + str(len(raw)).encode('ascii') + b'\0' + raw).hexdigest()
        if len(raw) != size or hashlib.sha256(raw).hexdigest() != sha256 or actual_blob != blob:
            raise ValueError('recovered template hash mismatch: ' + name)
        specs.append(TemplateSpec(path, category, GROUND_LABELS[category],
            {'repository': TEMPLATE_REPOSITORY, 'commit': TEMPLATE_COMMIT,
             'repository_path': source_path, 'source_url': expected_url,
             'git_blob_sha': blob, 'license_status': 'public_repository_provenance_preserved'}, sha256))
    return specs
