# sirius_navigation パッケージ

Siriusロボットのナビゲーション機能を提供するROS2パッケージです。

## 主な機能

### 1. センサーフュージョン（EKF）
- IMUとホイールオドメトリを融合
- 高精度な自己位置推定
- robot_localizationパッケージのEKFを使用

### 2. HWT905 IMU対応
- Witmotion HWT905高精度AHRSセンサー対応
- 姿勢精度0.05°（静止時）
- 最大200Hz更新周波数
- プラグアンドプレイ対応

### 3. アシストテレオペレーション
- 障害物回避機能付きテレオペレーション
- 安全な手動操作をサポート

### 4. 3Dカラーマッピング（RTAB-Map）
- SAM3の点群を使用した高品質な3Dマップ構築
- ループクロージャ検出による自己位置修正
- 2D SLAM（SlamToolbox）との統合

### 5. ナビゲーションツール
- EKF初期位置設定ツール
- ナビゲーションキャンセル機能

## ドキュメント

### クイックスタート
- **[QUICKSTART_HWT905.md](./QUICKSTART_HWT905.md)** - HWT905 IMUの最速セットアップ

### 詳細ガイド
- **[README_HWT905.md](./README_HWT905.md)** - HWT905の詳細設定とトラブルシューティング
- **[HWT905_COORDINATE_TRANSFORM.md](./HWT905_COORDINATE_TRANSFORM.md)** - HWT905座標系変換の詳細説明
- **[SENSOR_FUSION_README.md](./SENSOR_FUSION_README.md)** - センサーフュージョンの概要と設定
- **[EKF_INITIALPOSE_GUIDE.md](./EKF_INITIALPOSE_GUIDE.md)** - EKF初期位置設定ガイド
- **[ASSISTED_TELEOP_README.md](./ASSISTED_TELEOP_README.md)** - アシストテレオペレーション
- **[README_RTABMAP_JA.md](./README_RTABMAP_JA.md)** - RTAB-Map 3Dカラーマッピング (詳細ガイド)
- **[TARGET_FOLLOW_README.md](./TARGET_FOLLOW_README.md)** - ターゲット追跡・追従システム（LiDARによる人追跡とフォロー）

### エラー対応
- **[ERROR_FIX_GUIDE.md](./ERROR_FIX_GUIDE.md)** - エラー対処ガイド
- **[FIX_SUMMARY.md](./FIX_SUMMARY.md)** - 修正の要約

## ローンチファイル

### 1. witmotion_hwt905.launch.py
HWT905 IMUセンサーを起動します。

```bash
# 基本的な使用方法
ros2 launch sirius_navigation witmotion_hwt905.launch.py

# カスタムポート指定
ros2 launch sirius_navigation witmotion_hwt905.launch.py \
  port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  frequency:=100.0
```

**引数:**
- `port`: シリアルポート（デフォルト: `/dev/ttyUSB0`）
- `baud_rate`: ボーレート（デフォルト: `115200`）
- `frequency`: 更新周波数 [Hz]（デフォルト: `100.0`）
- `frame_id`: フレームID（デフォルト: `imu_link`）
- `imu_topic`: 出力トピック名（デフォルト: `/imu`）
- `log_level`: ログレベル（デフォルト: `info`）

### 2. sensor_fusion.launch.py
IMUとオドメトリを融合するEKFノードを起動します。

```bash
# シミュレーションIMU使用
ros2 launch sirius_navigation sensor_fusion.launch.py

# HWT905を自動起動
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true

# カスタムポートでHWT905を起動
ros2 launch sirius_navigation sensor_fusion.launch.py \
  start_hwt905:=true \
  hwt905_port:=/dev/ttyACM0 \
  hwt905_baud:=115200
```

**引数:**
- `start_hwt905`: HWT905を自動起動（デフォルト: `false`）
- `hwt905_port`: HWT905のシリアルポート（デフォルト: `/dev/ttyUSB0`）
- `hwt905_baud`: HWT905のボーレート（デフォルト: `115200`）

### 3. assisted_teleop.launch.py
障害物回避機能付きテレオペレーションを起動します。

```bash
ros2 launch sirius_navigation assisted_teleop.launch.py
```

### 4. sam3_rtabmap.launch.py
RTAB-Mapを使用した3Dカラーマッピングを起動します。

```bash
ros2 launch sirius_navigation sam3_rtabmap.launch.py
```

## スクリプト

### start_hwt905.sh
HWT905を簡単に起動するためのスクリプトです。

```bash
# デフォルトポート（/dev/ttyUSB0）で起動
./start_hwt905.sh

# カスタムポートで起動
./start_hwt905.sh /dev/ttyACM0
```

### start_navigation_with_fusion.sh
センサーフュージョンとナビゲーションを起動します。

```bash
./start_navigation_with_fusion.sh
```

### start_assisted_teleop.sh
アシストテレオペレーションを起動します。

```bash
./start_assisted_teleop.sh
```

## Pythonノード

### 1. ekf_pose_initializer.py
RViz2の`/initialpose`をEKFに転送するノードです。

**購読トピック:**
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped)

**配信トピック:**
- `/set_pose` (geometry_msgs/PoseWithCovarianceStamped)

### 2. assisted_teleop.py
障害物回避機能付きテレオペレーションノードです。

**購読トピック:**
- `/cmd_vel` (geometry_msgs/Twist)
- `/scan` (sensor_msgs/LaserScan)

**配信トピック:**
- `/cmd_vel_nav` (geometry_msgs/Twist)

### 3. cancel_navigation.py
ナビゲーションをキャンセルするユーティリティノードです。

### 4. move_goal.py
ウェイポイント定義ファイルを読み込み、Nav2アクションサーバーに対して順次ゴールを送信し、自動でウェイポイント追従走行を行うノードです。

**主な機能・仕様:**
* **連続・滑らかな走行（ゴール手前スイッチ方式）：**
  ロボットが現在のアクティブな経由地に到達する前に次のゴールを送信し続けることで、ロボットの立ち止まりや減速を防止し、滑らかなカーブで連続走行させます。
* **判定閾値（距離）の自動変更：**
  * **`0.5 m`：** `stop`（一時停止）、`wait_time`（待機）、`change_map`（地図切り替え）が設定されているウェイポイントで適用されます（現地で精密に合わせるため）。
  * **指定値 (m)：** 各ウェイポイントの yaml 定義に `threshold` が明示的に設定されている場合、その設定値が優先されます。
  * **`2.0 m`：** 上記以外の通常のウェイポイントで適用されるデフォルト値（MPPIの自動減速に捕まるのを防ぎ、最高速度での走行を維持します）。
* **位置判定周期：** `10 Hz (0.1秒周期)` で自己位置のチェックを行い、高速走行時でも正確なタイミングで上書き送信を行います。
* **最終地点での完全停止：** 最終ウェイポイントのみは上書き送信を行わないため、Nav2側の標準のゴール到達判定（`xy_goal_tolerance` / `yaw_goal_tolerance`）が走り、目標座標・向きに正確に停止します。

**購読トピック/情報:**
* TF (`/map` -> `/sirius3/base_footprint`) からの自己位置取得

**配信トピック:**
* `target_odom` (nav_msgs/Odometry) - アクティブなウェイポイント座標の配信
* `/stop` (std_msgs/Bool) - 一時停止時のストップコマンド配信
* `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - 地図切り替え時の初期位置設定

**YAML設定例 (`waypoints.yaml`):**
```yaml
waypoints:
  - number: 1
    x: 2.5
    y: 1.0
    angle_radians: 0.0
    threshold: 2.5      # 直線区間：2.5m手前で次のゴールを送り、最高速度を維持！
  - number: 2
    x: 5.0
    y: 1.0
    angle_radians: 1.57
    threshold: 1.0      # カーブ区間：1.0m手前まで引き付けて旋回し、壁へのインカットを防ぐ
  - number: 3
    x: 7.0
    y: 2.0
    angle_radians: 0.0
    stop: true          # 一時停止する点：自動的に0.5m手前で精密に判定されて停止します
```

## セットアップ

### 1. 依存パッケージのインストール

```bash
# robot_localization
sudo apt install ros-jazzy-robot-localization

# witmotion_ros（HWT905使用の場合）
cd ~/sirius_jazzy_ws/src
git clone https://github.com/ElettraSciComp/witmotion_IMU_ros.git
cd ~/sirius_jazzy_ws
colcon build --packages-select witmotion_ros
```

### 2. sirius_navigationパッケージのビルド

```bash
cd ~/sirius_jazzy_ws
colcon build --packages-select sirius_navigation
source install/setup.bash
```

### 3. HWT905デバイス権限設定（実機使用の場合）

```bash
# 一時的な設定
sudo chmod 666 /dev/ttyUSB0

# 永続的な設定（推奨）
sudo usermod -a -G dialout $USER
# 設定後、再ログインまたは再起動が必要
```

## 使用例

### シミュレーション環境

```bash
# ターミナル1: Gazeboシミュレーション
ros2 launch sirius_description sim.launch.py

# ターミナル2: センサーフュージョン
ros2 launch sirius_navigation sensor_fusion.launch.py
```

### 実機環境（HWT905使用）

```bash
# ターミナル1: HWT905 + センサーフュージョン
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true

# ターミナル2: ナビゲーション
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$HOME/sirius_jazzy_ws/params/nav2_params.yaml
```

## トピック構成

### 入力トピック
- `/odom` (nav_msgs/Odometry) - ホイールオドメトリ
- `/imu` (sensor_msgs/Imu) - IMUデータ（HWT905またはシミュレーション）
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - RViz2からの初期位置

### 出力トピック
- `/odom/filtered` (nav_msgs/Odometry) - EKFで融合された自己位置推定

### TFフレーム
```
map
  └─ odom (AMCLが配信)
      └─ base_footprint (EKFが配信)
          └─ imu_link
```

## トラブルシューティング

### HWT905が認識されない

```bash
# デバイスの確認
ls -l /dev/ttyUSB* /dev/ttyACM*

# 接続状況の確認
dmesg | grep tty
```

### 権限エラー

```bash
# グループに追加（永続的）
sudo usermod -a -G dialout $USER
# 再ログイン

# または一時的に権限変更
sudo chmod 666 /dev/ttyUSB0
```

### EKFがIMUを認識しない

```bash
# トピックの確認
ros2 topic list | grep imu
ros2 topic hz /imu

# EKFの診断情報
ros2 topic echo /diagnostics | grep ekf
```

### 座標系が反転している

HWT905とROS2の座標系の違いによる問題です。

**症状**: 加速度やジャイロのデータが反転している

**解決方法**:
```bash
# パラメータを確認
ros2 param get /hwt905_imu imu_publisher.use_native_orientation
# Expected: False

# 詳細は座標系変換ガイドを参照
```

詳細は [HWT905_COORDINATE_TRANSFORM.md](./HWT905_COORDINATE_TRANSFORM.md) を参照してください。

### データが不安定

1. `ekf_fusion.yaml`のセンサー設定を確認
2. プロセスノイズを調整
3. センサータイムアウトを増やす

詳細は [ERROR_FIX_GUIDE.md](./ERROR_FIX_GUIDE.md) を参照してください。

## パフォーマンスチューニング

### 更新周波数の最適化

**HWT905:**
```bash
# 標準（推奨）
frequency:=50.0

# 高精度
frequency:=100.0

# 最高精度（CPU負荷高）
frequency:=200.0
```

**EKF:**
```yaml
# ekf_fusion.yaml
frequency: 30.0  # 標準: 20-30Hz
```

### センサー信頼度の調整

`ekf_fusion.yaml`で各センサーのprocess_noise_covarianceを調整:
- 値を小さくする → センサーの信頼度を上げる
- 値を大きくする → センサーの信頼度を下げる

## 開発情報

### パッケージ構成

```
sirius_navigation/
├── launch/                    # ローンチファイル
│   ├── witmotion_hwt905.launch.py
│   ├── sensor_fusion.launch.py
│   └── assisted_teleop.launch.py
├── sirius_navigation/         # Pythonノード
│   ├── ekf_pose_initializer.py
│   ├── assisted_teleop.py
│   └── cancel_navigation.py
├── test/                      # テストコード
├── *.sh                       # 起動スクリプト
└── *.md                       # ドキュメント
```

### テスト

```bash
cd ~/sirius_jazzy_ws
colcon test --packages-select sirius_navigation
colcon test-result --verbose
```

## ライセンス

このパッケージは、Siriusロボットプロジェクトの一部として提供されています。

## 貢献

バグ報告や機能リクエストは、GitHubのIssueで受け付けています。

## サポート

質問や問題がある場合は、以下のドキュメントを参照してください:
- [QUICKSTART_HWT905.md](./QUICKSTART_HWT905.md) - 最速セットアップ
- [README_HWT905.md](./README_HWT905.md) - 詳細ガイド
- [ERROR_FIX_GUIDE.md](./ERROR_FIX_GUIDE.md) - エラー対処
