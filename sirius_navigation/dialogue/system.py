# -*- coding: utf-8 -*-

# システム状態・ロボット情報（バッテリー、速度変更など）に関連するセリフテンプレート
SYSTEM_TEMPLATES = {
    # バッテリー報告 (フォーマット変数 {level}, {charging_str} が使えます)
    "battery_report": "[happy]バッテリー残量は {level:.0f}パーセント で、状態は {charging_str} なのだ！",
    "battery_error": "[hurt]バッテリー残量が取得できなかったのだ。",
    "battery_fail": "[hurt]バッテリー情報の取得に失敗したのだ。",
    
    # 速度変更時 (フォーマット変数 {speed} が使えます)
    "speed_change": "速度を {speed:.2f} に変更したのだ！"
}
