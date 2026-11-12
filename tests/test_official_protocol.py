"""Official table 5 phrases; no image identity is inferred by this module."""
import pytest


@pytest.mark.parametrize("target", ["坍塌废墟", "滑坡塌方", "被困人员", "受损车辆", "救援营地"])
def test_ground_phrase_matches_official_table_five(target):
    from inspection_mission.official_protocol import recognition_phrase
    assert recognition_phrase("ground", target) == f"发现{target}，地面巡航任务点确认，将巡航信息发送回指挥中心。"


@pytest.mark.parametrize("target", ["坍塌废墟", "滑坡塌方", "被困人员", "受损车辆", "救援营地"])
def test_aerial_phrase_matches_official_table_five(target):
    from inspection_mission.official_protocol import recognition_phrase
    assert recognition_phrase("aerial", target) == f"发现空中巡航任务点，识别目标为{target}，执行灯光提示，将巡航信息发送回指挥中心。"


@pytest.mark.parametrize("target", ["救援无人机", "飞鸟"])
def test_window_phrase_matches_official_table_five(target):
    from inspection_mission.official_protocol import recognition_phrase
    assert recognition_phrase("window", target) == f"发现窗户上方目标为{target}，执行下降避让，穿越窗户。"


@pytest.mark.parametrize("kind,target", [
    ("ground", "飞鸟"), ("window", "被困人员"), ("aerial", "person"),
    ("aerial", "坍塌废墟,被困人员"), ("ground", "*"), ("ground", ""),
    ("ground", None), ("ground", ["坍塌废墟", "被困人员"]),
    ("guess", "坍塌废墟"),
])
def test_unknown_or_ambiguous_observed_category_is_rejected(kind, target):
    from inspection_mission.official_protocol import recognition_phrase
    with pytest.raises(ValueError):
        recognition_phrase(kind, target)
