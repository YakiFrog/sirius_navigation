# Sirius Navigation - ターゲット追跡・追従システム (Target Tracking & Following System)

本システムは、LiDAR（2Dレーザースキャン）データから特定のターゲット（人間・NPCなど）をリアルタイムに検出し、一定の距離を保ちながら自動で追従（フォロー）するための機能を提供します。

---

## システム構成

本システムは、**検出ノード (`target_detector`)** と **追従ノード (`target_follower`)** の2つのROS 2ノードから構成されており、これらは管理しやすいように専用のサブパッケージフォルダ `sirius_navigation/target_following/` にまとめられています。

```mermaid
graph TD
    %% センサー入力
    subgraph Sensors [センサー入力]
        Hokuyo["Hokuyo LiDAR<br/>(足元 180° /hokuyo_scan)"]
        Scan3["Scan3 LiDAR<br/>(胴体 270° /scan3)"]
    end

    %% 検出ノード
    subgraph DetectorNode [target_following/target_detector.py]
        Legs["脚クラスタ検出"]
        Torso["胴体クラスタ検出"]
        Fusion["データフュージョン<br/>(脚+胴体関連付け)"]
        MTT["マルチターゲットトラッカー<br/>(複数ターゲットの管理)"]
        KF["個別カルマンフィルター<br/>(複数のTrackを同時追跡)"]
    end

    %% 追従ノード
    subgraph FollowerNode [target_following/target_follower.py]
        OdomSub["ターゲットオドメトリ購読<br/>(/npc/odom)"]
        TF["自己位置ルックアップ<br/>(map ↔ base_footprint)"]
        Calc["追従目標座標計算<br/>(維持距離・姿勢)"]
    end

    %% 出力・アクション
    subgraph Nav2Stack [Nav2 スタック]
        Nav2["NavigateToPose アクション"]
    end

    %% 接続関係
    Hokuyo --> Legs
    Scan3 --> Torso
    Legs --> Fusion
    Torso --> Fusion
    Fusion --> MTT
    MTT -->|新規/既存Track割り当て| KF
    KF -->|/target_detector/target_markers| RViz2[RViz2 可視化]
    KF -->|プライマリターゲット情報 /npc/odom| OdomSub
    OdomSub --> Calc
    TF --> Calc
    Calc -->|NavigateToPose| Nav2
```

---

## 1. 複数ターゲット検出・追跡ノード (`target_detector.py`)

足元の LiDAR（脚）と腰の高さの LiDAR（胴体）のデータを統合（フュージョン）し、**複数のターゲット（周囲にいるすべての人型オブジェクト）をマルチターゲット・トラッキング (MTT)** します。

### トラッキングアルゴリズムの仕組み

1. **個別トラック管理 (`Track`クラス)**:
   検出されたすべてのターゲットは、個別の ID、位置・速度状態（カルマンフィルター）、共分散、およびサイズ特徴量を保持する `Track` インスタンスとして管理されます。
2. **ゲート付き最近傍関連付け (Gated Nearest Neighbor: GNN)**:
   毎フレーム、既存の全トラックの予測位置と新しく検出された候補位置との間の「統合コスト（距離差 ＋ 特長量の類似度）」を計算し、最もマッチする組み合わせを決定します。
3. **新規ターゲット登録**:
   どの既存トラックにも関連付けられなかった検出候補が、ロボット前方のロックオンエリア（前方1.0m、左右±80cm）内で検知され、数フレーム安定して検出された場合、新しいユニークIDを持つ新規トラックとして登録されます。
4. **プライマリターゲット（追従対象）の選定**:
   * ロボットが追従して移動するための「プライマリターゲット（`/npc/odom` へ配信）」は、ロックオンされたトラックから選ばれます。
   * 現在のプライマリターゲットがロストした場合、**自動的に現在検知されている中で最も近い他のアクティブなトラックに引き継がれ**、誰もいない場合のみキャリブレーション待ちに戻ります。

### 主要パラメータ (`params/` 内で設定可能)

| パラメータ名 | デフォルト値 | 説明 |
| :--- | :--- | :--- |
| `lockon_max_range` | **`1.0m`** | キャリブレーション時に新規ロックオンを受け入れる前方の最大距離。 |
| `lockon_max_lateral` | **`0.8m`** | キャリブレーション時に受け入れる左右の最大ズレ幅（±80cm）。 |
| `calib_miss_tolerance` | **`10`** | キャリブレーション中の検出漏れ許容数（10フレーム/約1秒）。超えるとカウントリセット。 |
| `gating_distance` | **`0.6m`** | 追従中のカルマンフィルター関連付けゲート。静的オブジェクト（柱など）への吸い付きを防止。 |
| `active_fov_deg` | **`270.0`** | 追従中の有効視野角。270度に広げることで、ロボットの真横や斜め後方の回り込みに対応。 |

---

## 2. ターゲット追従ノード (`target_follower.py`)

検出ノードが選定したプライマリターゲットのオドメトリ情報（`/npc/odom`）を元に、ロボットが一定の距離を維持して追尾するためのゴール指令を Nav2 に送信します。

### 主要パラメータ

| パラメータ名 | デフォルト値 | 説明 |
| :--- | :--- | :--- |
| `follow_distance` | `0.8m` | ターゲットとロボットが維持する目標距離。 |
| `deadband` | `0.1m` | 停止判定用の不感帯（維持距離±10cm）。これより近づいた場合は自動停止。 |
| `min_update_distance` | `0.15m` | ターゲットがこの距離以上移動した場合のみゴールを更新。 |

---

## 起動とRViz2での可視化

ナビゲーションスタックが立ち上がっている状態で、一括起動スクリプトを実行してください。

```bash
# 1. ワークスペースのセットアップ
cd ~/sirius_jazzy_ws
source install/setup.bash

# 2. 追従制御メニューの起動
./bash/startup_bash/target_follow.sh
```

### RViz2での確認
* トラッキングされている全ての人型オブジェクトは、トピック `/target_detector/target_markers` (MarkerArray) として配信されます。
* RViz2に「MarkerArray」ディスプレイを追加し、トピックに `/target_detector/target_markers` を設定することで、追跡されているすべての人がシリンダーで表示され、頭の上に `ID:X` または `FOLLOW ID:X` (追従対象) とテキストが表示されます。
