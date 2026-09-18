"""Shared helper to load semantic class cost configuration.

路面マッピングの生成側（sam3_map_colorizer / sam3_indexed_map_node /
theta_indexed_map_node）と、消費側（sam3_colored_map_loader）の両方から使う。

コストは OccupancyGrid 値 (0-100) で表現する:
  global_cost : グローバルコストマップ用。100 = LETHAL（壁相当）。
  local_cost  : ローカルコストマップ用のソフトコスト。
"""

import os

import yaml

DEFAULT_FILENAME = "semantic_costs.yaml"
DEFAULT_GLOBAL_COST = 0
DEFAULT_LOCAL_COST = 0


def _candidate_paths():
    """設定ファイルの候補パスを優先順に返す."""
    env_path = os.environ.get("SIRIUS_SEMANTIC_COSTS")
    if env_path:
        yield os.path.expanduser(env_path)

    # ワークスペースの params ディレクトリ（手編集しやすい）
    yield os.path.expanduser("~/sirius_jazzy_ws/params/semantic_costs.yaml")

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


def load_semantic_costs():
    """(costs, path) を返す.

    costs は {class_name_lower: {"global_cost": int, "local_cost": int}}。
    見つからない場合は ({}, None)。
    """
    for path in _candidate_paths():
        if not path or not os.path.exists(path):
            continue
        try:
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:  # pragma: no cover - defensive
            print(f"Warning: failed to read semantic costs from {path}: {exc}")
            continue

        costs = {}
        if isinstance(data, dict):
            for name, info in data.items():
                if not isinstance(info, dict):
                    continue
                costs[str(name).strip().lower()] = {
                    "global_cost": int(info.get("global_cost", DEFAULT_GLOBAL_COST)),
                    "local_cost": int(info.get("local_cost", DEFAULT_LOCAL_COST)),
                }
        return costs, path

    return {}, None


def costs_for(costs, name):
    """クラス名に対応するコスト辞書を返す（無ければ 0/0）."""
    return costs.get(
        str(name).strip().lower(),
        {"global_cost": DEFAULT_GLOBAL_COST, "local_cost": DEFAULT_LOCAL_COST},
    )
