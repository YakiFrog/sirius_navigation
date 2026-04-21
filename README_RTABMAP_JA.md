# RTAB-Map 3D カラーマッピング

このドキュメントでは、Siriusロボットで **RTAB-Map** を使用して環境の 3D マップを作成する方法を説明します。

## 概要

`sam3_rtabmap.launch.py` は、以下の3つのコンポーネントを統合して高品質な 3D カラーマップを作成します。

1.  **SAM3 (Segmentation and Mapping)**: ZEDカメラ等から取得した点群データを ROS 2 に配信します。
2.  **SlamToolbox**: 2D レーザースキャンデータから堅牢な自己位置推定（オドメトリ）を提供します。
3.  **RTAB-Map (Real-Time Appearance-Based Mapping)**: グラフベースの SLAM で、3D データの蓄積、ループクロージャ検出（以前訪れた場所の認識）、そして最終的な 3D マップの生成を担います。

## 主な特徴

-   **3D カラーマップ生成**: SAM3 から提供される色付き点群を使用して、視覚的に分かりやすいマップを作成します。
-   **ループクロージャ検出**: ロボットが以前の場所に戻った際に誤差を修正し、地図の整合性を保ちます。
-   **セグメンテーションとの統合**: SAM3 によって特定の物体や領域が抽出された点群のみを使用することで、動的な物体を除去した高品質な地図作成が可能です。

## 使用方法

### 起動コマンド

```bash
ros2 launch sirius_navigation sam3_rtabmap.launch.py
```

### 主要な引数

-   `use_sim_time`: シミュレーション環境で使用する場合は `true`（デフォルト）に設定します。

## アーキテクチャ

RTAB-Map ノードは以下のトピックを購読・配信します。

### 購読トピック (Subscribed Topics)

-   `/sam3/obstacles` (sensor_msgs/PointCloud2): SAM3 から配信される 3D 点群データ。
-   `tf`: ロボットの座標系情報。

### 配信トピック (Published Topics)

-   `/rtabmap/mapData`: 作成されたマップのグラフ構造とデータ。
-   `/rtabmap/cloud_map`: 蓄積された 3D 点群マップ。
-   `/rtabmap/grid_map`: RTAB-Map が生成した 2D 占有格子地図。

## 設定の詳細

RTAB-Map のパラメータは `sam3_rtabmap.launch.py` 内で以下のように最適化されています：

-   `RGBD/AngularUpdate`: `0.01` (少しの回転でも地図を更新)
-   `RGBD/LinearUpdate`: `0.01` (少しの移動でも地図を更新)
-   `Grid/FromDepth`: `true` (3D点群からグリッドマップを生成)
-   `Reg/Strategy`: `0` (Visual Registration を使用)

## 注意事項

-   RTAB-Map はメモリ負荷が高いため、長時間のマッピングを行う際はメモリ使用量に注意してください。
-   高品質なマップを作成するには、ロボットをゆっくり動かすことが推奨されます。
