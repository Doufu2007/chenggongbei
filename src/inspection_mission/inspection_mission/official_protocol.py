"""巡检识别话术，逐字对应官方判分细则 V1.0 第 5 章表 5。

Source: https://www.splenach.com/api/courses/129d6e75-f19a-4c97-9ec6-0853695a4c48
Saved evidence: docs/referee_rules_v1.html (retrieved 2026-11-06).

This module maps one camera-observed Chinese category to its official phrase.
It does not identify images, choose a category, probe a referee, or acknowledge
recognition. Callers must supply a fresh, unambiguous visual observation.
"""
from __future__ import annotations


_GROUND_CATEGORIES = frozenset(("坍塌废墟", "滑坡塌方", "被困人员", "受损车辆", "救援营地"))
_WINDOW_CATEGORIES = frozenset(("救援无人机", "飞鸟"))


def recognition_phrase(kind: str, target_id: str) -> str:
    """Return exactly one table-5 phrase for a measured category.

    ``kind`` is ``ground``, ``aerial`` or ``window``.  Unknown categories,
    decoder aliases and combined alternatives are refused rather than guessed.
    The controller owns observation freshness, confidence and association.
    """
    if not isinstance(kind, str) or kind not in ("ground", "aerial", "window"):
        raise ValueError("unsupported official recognition kind")
    allowed = _WINDOW_CATEGORIES if kind == "window" else _GROUND_CATEGORIES
    if not isinstance(target_id, str) or target_id not in allowed:
        raise ValueError("recognition must contain exactly one observed official category")
    if kind == "ground":
        return f"发现{target_id}，地面巡航任务点确认，将巡航信息发送回指挥中心。"
    if kind == "aerial":
        return f"发现空中巡航任务点，识别目标为{target_id}，执行灯光提示，将巡航信息发送回指挥中心。"
    return f"发现窗户上方目标为{target_id}，执行下降避让，穿越窗户。"
