"""Offline image-neighbor diagnostics must preserve provenance and raw scores."""
import importlib
import importlib.util
import json
import hashlib

import numpy as np
import pytest
from PIL import Image


def api():
    name = "tools.diagnose_clip_image_neighbors"
    assert importlib.util.find_spec(name) is not None, "independent image-feature diagnostic is missing"
    return importlib.import_module(name)


def test_normalized_image_cosines_keep_every_reference_and_rank_without_acceptance():
    report = api().cosine_neighbors([[3., 0.], [0., 2.]], [[1., 1.], [1., 0.], [0., -1.]],
                                    ["diagonal", "right", "down"])
    assert np.asarray(report["cosine_matrix"]) == pytest.approx(np.asarray([[2**-.5, 1., 0.], [2**-.5, 0., -1.]]))
    assert [item["reference_id"] for item in report["rankings"][0]] == ["right", "diagonal", "down"]
    assert [item["reference_id"] for item in report["rankings"][1]] == ["diagonal", "right", "down"]
    assert report["class_decision"] is None and report["threshold"] is None


@pytest.mark.parametrize("bad", [[[0., 0.]], [[float("nan"), 1.]], [[float("inf"), 1.]]])
def test_invalid_feature_vectors_do_not_produce_rankings(bad):
    with pytest.raises(ValueError):
        api().cosine_neighbors(bad, [[1., 0.]], ["ref"])


def make_projection(tmp_path):
    original, query = tmp_path / "original.png", tmp_path / "ground4_candidate_rectified_world_axes.png"
    Image.new("RGB", (10, 10), "gray").save(original)
    Image.new("RGB", (8, 12), "blue").save(query)
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    projection = tmp_path / "projection.json"
    projection.write_text(json.dumps({"source_image": {"path": str(original), "sha256": digest(original)},
        "products": [{"path": str(query), "sha256": digest(query)}], "mapping_status": "candidate_unverified"}), encoding="utf-8")
    return projection, original, query


def test_query_load_verifies_original_and_derived_bytes_and_keeps_mode_assumptions(tmp_path):
    projection, _, query = make_projection(tmp_path)
    path, record = api().load_query_record(projection, "ground4_candidate_rectified_world_axes.png")
    assert path == query and record["mapping_status"] == "candidate_unverified"
    assert record["query_size_wh"] == [8, 12]


@pytest.mark.parametrize("which", ["original", "query"])
def test_query_provenance_rejects_any_image_byte_change(tmp_path, which):
    projection, original, query = make_projection(tmp_path)
    (original if which == "original" else query).write_bytes(b"changed")
    with pytest.raises(ValueError, match="digest"):
        api().load_query_record(projection, "ground4_candidate_rectified_world_axes.png")
