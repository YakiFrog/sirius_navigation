"""Test the shared SAM3 class registry loader and alias helpers."""

import textwrap
from pathlib import Path

from sirius_navigation.sam3_classes import (
    load_sam3_classes,
    normalize_classes,
    prompt_from_classes,
    unknown_prompts,
)

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "sam3_classes.yaml"


def test_normalize_classes_rejects_reserved_and_invalid_ids():
    raw = {
        "grass": {"id": 3, "color": [0, 255, 0]},
        "wall": {"id": 1, "color": [0, 0, 0]},
        "broken": {"id": 999, "color": [1, 2, 3]},
        "bad_color": {"id": 4, "color": [300, 0, 0]},
        "short_color": {"id": 4, "color": [255, 255]},
    }

    classes = normalize_classes(raw)

    assert set(classes) == {"grass", "short_color"}
    assert classes["grass"]["id"] == 3
    # 2要素の色は0で埋めて3要素に正規化する。
    assert classes["short_color"]["color"] == [255, 255, 0]


def test_prompt_from_classes_preserves_order():
    classes = {"grass": {"id": 3}, "tactile paving": {"id": 4}}

    assert prompt_from_classes(classes) == "grass, tactile paving"


def test_unknown_prompts_reports_unregistered_phrases():
    classes = {"grass": {"id": 3}, "tactile paving": {"id": 4}}

    assert unknown_prompts("grass, line type tactile paving", classes) == [
        "line type tactile paving"
    ]
    assert unknown_prompts("grass, tactile paving", classes) == []


def test_load_sam3_classes_reads_env_override(tmp_path, monkeypatch):
    config = tmp_path / "sam3_classes.yaml"
    config.write_text(
        textwrap.dedent(
            """
            prompt: grass, lawnturf
            classes:
              grass: {id: 3, color: [0, 255, 0]}
              lawnturf: {id: 3, color: [0, 255, 0]}
            class_thresholds:
              grass: 0.4
            """
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("SIRIUS_SAM3_CLASSES", str(config))

    cfg, path = load_sam3_classes()

    assert path == str(config)
    assert cfg["prompt"] == "grass, lawnturf"
    # エイリアスは同じIDへ統合される。
    assert cfg["classes"]["lawnturf"]["id"] == cfg["classes"]["grass"]["id"] == 3
    assert cfg["class_thresholds"]["grass"] == 0.4


def test_shared_config_merges_line_type_tactile_paving():
    """同梱の登録簿で line-type 点字ブロックが tactile paving と同一IDか確認."""
    import yaml

    data = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8")) or {}
    classes = normalize_classes(data.get("classes"))

    assert classes["tactile paving"]["id"] == 4
    for alias in ("line-type tactile paving", "line type tactile paving"):
        assert classes[alias]["id"] == classes["tactile paving"]["id"]


def test_config_file_exists_in_source_tree():
    assert CONFIG_PATH.exists()
