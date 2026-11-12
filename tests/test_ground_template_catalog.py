from pathlib import Path
import shutil

import pytest


def test_complete_catalog_restores_two_download_failures_without_changing_frozen_eight():
    from inspection_mission.ground_template_catalog import load_complete_ground_templates
    from inspection_mission.ground_single_point import load_ground_templates
    old = load_ground_templates()
    specs = load_complete_ground_templates()
    assert len(specs) == 10
    by_name = {Path(s.path).name: s for s in specs}
    assert len(by_name) == 10
    for spec in old:
        assert by_name[Path(spec.path).name] == spec
    assert by_name['damaged_vehicle_02.jpg'].class_id == 'damaged_vehicle'
    assert by_name['collapsed_ruins_01.png'].class_id == 'collapsed_ruins'
    assert all(s.expected_sha256 for s in specs)


def test_corrupted_recovered_template_is_rejected(tmp_path):
    from inspection_mission.ground_template_catalog import load_complete_ground_templates, additional_asset_root
    root = tmp_path / 'additional'
    shutil.copytree(additional_asset_root(), root)
    (root / 'damaged_vehicle_02.jpg').write_bytes(b'incomplete-download')
    with pytest.raises(ValueError, match='hash'):
        load_complete_ground_templates(additional_dir=root)


def test_missing_recovered_template_is_rejected(tmp_path):
    from inspection_mission.ground_template_catalog import load_complete_ground_templates, additional_asset_root
    root = tmp_path / 'additional'
    shutil.copytree(additional_asset_root(), root)
    (root / 'collapsed_ruins_01.png').unlink()
    with pytest.raises(ValueError, match='missing'):
        load_complete_ground_templates(additional_dir=root)
