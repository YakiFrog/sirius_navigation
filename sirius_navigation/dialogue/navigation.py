# -*- coding: utf-8 -*-

# 移動・ナビゲーション動作に関連するセリフテンプレート
NAVIGATION_TEMPLATES = {
    # 移動開始時 (フォーマット変数 {distance} が使えます)
    "forward_start": "[happy]{distance:.1f}メートル前進するのだ！",
    "backward_start": "[surprised]{distance:.1f}メートル後退するのだ！",
    "turn_start": "[wink]そっちを向くのだ！",
    "goto_start": "[happy]座標({x:.1f}, {y:.1f})に向かうのだ！",

    # 動作完了・失敗時
    "turn_success": "[happy]旋回が完了したのだ！",
    "turn_failure": "[sad]旋回に失敗したのだ。",
    "arrival": "[happy]目的地に到着したのだ！",
    "stuck": "[sad]行く手が遮られていて、進めないのだ...",
    "parse_failure": "[sad]指示の理解に失敗したのだ。",
}
