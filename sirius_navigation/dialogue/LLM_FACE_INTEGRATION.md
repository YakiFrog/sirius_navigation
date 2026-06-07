# LLM & 顔アプリの双方向連携仕様

このドキュメントでは、ROS 2ナビゲーションノード（`llm_dynamic_goal`）と、Electronベースの顔アプリ（`sirius_face_anim2`）の間で行われる、リアルタイムの双方向通信仕様について説明します。

---

## 通信アーキテクチャ

```mermaid
graph LR
    subgraph 顔アプリ (Electron / Docker)
        Backend[Python対話システム]
    end
    
    subgraph ROS 2 環境 (ホストPC側)
        ROS2Node[llm_dynamic_goal.py ノード]
    end

    Backend -->|HTTP POST 送信 ポート 50060| ROS2Node
    ROS2Node -->|gRPC発話指示 ポート 50052 /Speak| Backend
    ROS2Node -->|gRPCパラメータ制御 ポート 50051 /UpdateParameters| FaceAppElectron
```

---

## 1. コマンド入力用のHTTPブリッジ (ポート 50060)

顔アプリがDockerコンテナ内で動作している場合（ROS 2環境がないため、`ros2 topic pub` コマンドが使えない状況）に備え、`llm_dynamic_goal.py` 内部で軽量なHTTPサーバーを起動しています。

- **ポート番号**: `50060`
- **エンドポイント**: `POST /instruction`
- **データ形式**: `{"instruction": "指示文字列"}`
- **動作**: POSTリクエストを受け取ると、ROS 2ノード内で即座に `process_instruction` が呼び出され、ナビゲーションや定型対話のアクションが開始されます。

---

## 2. 外部ライブラリの動的割り込み (sys.pathの優先差し込み)

ホストPCのシステム環境に入っている古い `protobuf` がインポートされて `cannot import name 'runtime_version'` エラーが発生するのを防ぐため、顔アプリ側の仮想環境（`venv`）のパッケージディレクトリを検索パスの先頭に差し込んでいます。
```python
home_dir = os.path.expanduser("~")
venv_packages = os.path.join(home_dir, "sirius_face_anim2/venv/lib/python3.12/site-packages")
sys.path.insert(0, venv_packages)
```
これにより、システムの環境を汚さずに `grpc` 通信を安定して行うことができます。

---

## 3. 音声発話・パラメータ制御 (gRPC ポート 50051 & 50052)

ROS 2ノードは、顔アプリに搭載されているgRPCサーバーに対して以下の要求を行います。

1. **発話指示とインジケータ消灯 (Speak)**:
   - シリウスくんが喋り始める際、ポート `50051` に対して `isThinking`（青枠）と `isParseControl`（黄枠）を両方 `0.0` にリセットし、画面の発光枠線を消灯させます。
   - その後、ポート `50052` を呼び出して合成音声を再生します。
2. **状態取得 (GetStatus)**:
   - ポート `50051` を呼び出して、現在のバッテリー残量（`batteryLevel`）や充電状態（`batteryCharging`）を取得します。

### 今回追加された音声案内
- **旋回開始時**: `「そっちを向くのだ！」`
- **旋回成功完了時**: `「旋回が完了したのだ！」`
- **旋回失敗時**: `「旋回に失敗したのだ。」`
- **目的地到着時**: `「目的地に到着したのだ！」`
- **障害物スタック（進めないとき）**: `「行く手が遮られていて、進めないのだ...」`
