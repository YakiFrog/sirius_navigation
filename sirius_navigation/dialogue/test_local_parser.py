# -*- coding: utf-8 -*-
import unittest
try:
    from .local_parser import parse_local_rules
except ImportError:
    try:
        from sirius_navigation.dialogue.local_parser import parse_local_rules
    except ImportError:
        from local_parser import parse_local_rules

class TestLocalParser(unittest.TestCase):
    def setUp(self):
        # 標準的なモック状態データ
        self.state_info = {
            "current_x": 1.0,
            "current_y": 2.0,
            "executing": False,
            "queue_len": 0,
            "stuck": False,
            "people_count": 2,
            "face_active": True,
            "current_expression": "normal",
            "current_speed_setting": 0.90,
            "current_vel_x": 0.0,
            "current_vel_theta": 0.0,
            "last_action_status": "success",
            "last_action_type": "forward",
            "last_target_value": 2.0,
            "last_action_start_x": 1.0,
            "last_action_start_y": 0.0,
            "last_action_start_yaw": 0.0,
            "last_cmd_was_backward": False,
            "last_cmd_was_forward": True,
        }

    def test_cancel_commands(self):
        for kw in ["止まれ", "ストップ", "キャンセル", "一時停止", "待機"]:
            res = parse_local_rules(kw, self.state_info)
            self.assertIsNotNone(res)
            self.assertTrue(res.get("cancel"))

    def test_chat_keywords(self):
        res = parse_local_rules("自己紹介して", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("シリウスなのだ", res.get("speak", ""))

        res = parse_local_rules("ういんくして", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("ウインクしてみたのだ", res.get("speak", ""))

    def test_speed_control(self):
        # 文字列指定
        res = parse_local_rules("ゆっくり進んで", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertTrue(any(c["type"] == "speed" and c["value"] == 0.2 for c in commands))

        # 数値直接指定
        res = parse_local_rules("速度を 0.5 にして", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0]["type"], "speed")
        self.assertEqual(commands[0]["value"], 0.5)

    def test_battery_query(self):
        def mock_battery_callback():
            return "バッテリーテストメッセージ"
        
        res = parse_local_rules("バッテリーはどう？", self.state_info, battery_callback=mock_battery_callback)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertEqual(res.get("speak"), "バッテリーテストメッセージ")

    def test_origin_return(self):
        res = parse_local_rules("原点に戻って", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0]["type"], "goto")
        self.assertEqual(commands[0]["value"], [0.0, 0.0])

    def test_negative_coordinate_with_unicode_minus(self):
        for text in ["−3,3移動して", "－3，3移動して"]:
            res = parse_local_rules(text, self.state_info)
            self.assertIsNotNone(res)
            commands = res.get("commands", [])
            self.assertEqual(len(commands), 1)
            self.assertEqual(commands[0]["type"], "goto")
            self.assertEqual(commands[0]["value"], [-3.0, 3.0])

    def test_capabilities(self):
        res = parse_local_rules("何ができるの？", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("前進、後退、右向き", res.get("speak"))

    def test_status_and_mood_queries(self):
        # 気分
        res = parse_local_rules("今の機嫌はどう？", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("落ち着いた普通の気分", res.get("speak"))

        # 人数
        res = parse_local_rules("周りに何人いる？", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("2人 の人が検知されている", res.get("speak"))

    def test_correction_rules(self):
        # 行き過ぎ (前進の直後 -> 後退)
        self.state_info["last_action_type"] = "forward"
        self.state_info["last_target_value"] = 2.0
        res = parse_local_rules("ちょっと行き過ぎだよ", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(commands[0]["type"], "backward")
        self.assertGreater(commands[0]["value"], 0.0)

        # 逆 (旋回の直後 -> 逆旋回)
        self.state_info["last_action_type"] = "turn"
        self.state_info["last_target_value"] = 1.57 # 左90度
        res = parse_local_rules("逆向きだよ", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(commands[0]["type"], "turn")
        self.assertEqual(commands[0]["value"], -1.57)

    def test_shapes_drawing(self):
        res = parse_local_rules("2mの四角形を描いて", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(len(commands), 8) # 4x (forward + turn)
        self.assertEqual(commands[0]["type"], "forward")
        self.assertEqual(commands[0]["value"], 2.0)
        self.assertEqual(commands[1]["type"], "turn")
        self.assertEqual(commands[1]["value"], -1.5708)

    def test_loop_commands(self):
        res = parse_local_rules("3回繰り返して右に90度回る", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        # "右に90度回る" は turn (value=-1.57079...)
        self.assertEqual(len(commands), 3)
        for c in commands:
            self.assertEqual(c["type"], "turn")
            self.assertAlmostEqual(c["value"], -1.570796, places=4)

if __name__ == '__main__':
    unittest.main()
