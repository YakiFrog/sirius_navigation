"""Shared helper to load the SAM3 semantic class registry.

ZED オフライン(run_offline_mapping.sh) と THETA(theta_sam3_perspective_node /
theta_sam3_bev_node / theta_sam3_preview) の両方がこのモジュールを使い、
config/sam3_classes.yaml の内容を SAM3 サーバの /class_registry と
/class_thresholds へ適用する。

登録簿は「検出プロンプト文字列」→ {"id": int, "color": [r, g, b]} の対応。
同じ id を複数のプロンプトに割り当てることで、言い回し違い（エイリアス）を
地図上の同一クラスへ統合できる。
"""

import os

import yaml

DEFAULT_FILENAME = "sam3_classes.yaml"

# 予約ID（0=unknown, 1=wall, 2=floor）。クラスIDは 3〜255。
RESERVED_IDS = (0, 1, 2)
MIN_CLASS_ID = 3
MAX_CLASS_ID = 255

# ファイルが無いときの後方互換フォールバック。
DEFAULT_CLASSES = {
    "grass": {"id": 3, "color": [0, 255, 0]},
    "tactile paving": {"id": 4, "color": [255, 255, 0]},
    "line-type tactile paving": {"id": 4, "color": [255, 255, 0]},
    "line type tactile paving": {"id": 4, "color": [255, 255, 0]},
    "roadway": {"id": 5, "color": [0, 0, 255]},
    "sidewalk": {"id": 6, "color": [128, 128, 128]},
}
DEFAULT_PROMPT = ", ".join(DEFAULT_CLASSES.keys())


def _candidate_paths():
    """設定ファイルの候補パスを優先順に返す."""
    env_path = os.environ.get("SIRIUS_SAM3_CLASSES")
    if env_path:
        yield os.path.expanduser(env_path)

    # ワークスペースの params ディレクトリ（手編集しやすい）
    yield os.path.expanduser("~/sirius_jazzy_ws/params/sam3_classes.yaml")

    # インストール済みパッケージの share/config
    try:
        from ament_index_python.packages import get_package_share_directory

        yield os.path.join(
            get_package_share_directory("sirius_navigation"),
            "config",
            DEFAULT_FILENAME,
        )
    except Exception:
        pass

    # ソースツリーの config（スクリプト直接実行時）
    here = os.path.dirname(os.path.abspath(__file__))
    yield os.path.join(os.path.dirname(here), "config", DEFAULT_FILENAME)

    # 開発中（colcon build 前）でもソースの設定を拾えるようにする。
    yield os.path.expanduser(
        "~/sirius_jazzy_ws/src/sirius/sirius_navigation/config/" + DEFAULT_FILENAME
    )


def normalize_classes(raw_classes):
    """クラス登録簿を検証・正規化する.

    {name: {"id": int, "color": [r, g, b]}} を返す。不正な行は除外する。
    """
    classes = {}
    if not isinstance(raw_classes, dict):
        return classes
    for name, spec in raw_classes.items():
        if not isinstance(spec, dict):
            continue
        clean_name = str(name).strip()
        if not clean_name:
            continue
        try:
            class_id = int(spec.get("id", 0))
        except (TypeError, ValueError):
            continue
        if not MIN_CLASS_ID <= class_id <= MAX_CLASS_ID:
            continue
        color = spec.get("color", [255, 255, 255])
        try:
            color = [int(value) for value in color][:3]
        except (TypeError, ValueError):
            continue
        while len(color) < 3:
            color.append(0)
        if any(not 0 <= value <= 255 for value in color):
            continue
        classes[clean_name] = {"id": class_id, "color": color}
    return classes


def load_sam3_classes():
    """(config, path) を返す.

    config は {"prompt": str, "classes": {name: {"id", "color"}},
    "class_thresholds": {name: float}}。見つからない場合は既定値を返し、
    path は None になる。
    """
    for path in _candidate_paths():
        if not path or not os.path.exists(path):
            continue
        try:
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:  # pragma: no cover - defensive
            print(f"Warning: failed to read SAM3 classes from {path}: {exc}")
            continue

        classes = normalize_classes(data.get("classes")) or dict(DEFAULT_CLASSES)
        raw_thresholds = data.get("class_thresholds") or {}
        thresholds = {}
        if isinstance(raw_thresholds, dict):
            for name, value in raw_thresholds.items():
                try:
                    thresholds[str(name).strip()] = float(value)
                except (TypeError, ValueError):
                    continue
        prompt = str(data.get("prompt") or "").strip() or prompt_from_classes(classes)
        return {
            "prompt": prompt,
            "classes": classes,
            "class_thresholds": thresholds,
        }, path

    return {
        "prompt": DEFAULT_PROMPT,
        "classes": dict(DEFAULT_CLASSES),
        "class_thresholds": {},
    }, None


def prompt_from_classes(classes):
    """登録簿のキーから検出プロンプト文字列を生成する（重複は保持）."""
    return ", ".join(str(name) for name in classes)


def unknown_prompts(prompt, classes):
    """プロンプト内で登録簿に無いフレーズを返す（ID0落ちの検出用）."""
    if not prompt:
        return []
    known = {str(name).strip() for name in classes}
    unknown = []
    for phrase in str(prompt).split(","):
        clean = phrase.strip()
        if clean and clean not in known:
            unknown.append(clean)
    return unknown
