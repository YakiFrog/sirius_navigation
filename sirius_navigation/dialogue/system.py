# -*- coding: utf-8 -*-

# システム状態・ロボット情報（バッテリー、速度変更など）に関連するセリフテンプレート
SYSTEM_TEMPLATES = {
    # バッテリー報告 (フォーマット変数 {level}, {charging_str} が使えます)
    "battery_report": "[happy]現在のバッテリー残量は {level:.1f}パーセントなのだ！状態は {charging_str} なのだ。",
    "battery_error": "[sad]バッテリー残量データが不正なのだ。",
    "battery_fail": "[sad]バッテリー状態が確認できないのだ。",
    
    # 速度変更時 (フォーマット変数 {speed} が使えます)
    "speed_change": "[happy]速度を {speed:.2f} に変更するのだ！"
}
