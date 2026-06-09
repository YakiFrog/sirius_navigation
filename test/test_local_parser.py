# -*- coding: utf-8 -*-
import unittest
from sirius_navigation.dialogue.local_parser import parse_local_rules, style_sirius_speak

class TestLocalParserColcon(unittest.TestCase):
    def setUp(self):
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
        for kw in ["止まれ", "ストップ", "キャンセル"]:
            res = parse_local_rules(kw, self.state_info)
            self.assertIsNotNone(res)
            self.assertTrue(res.get("cancel"))

    def test_chat_keywords(self):
        res = parse_local_rules("自己紹介して", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("シリウスなのだ", res.get("speak", ""))

        res = parse_local_rules("あなたの名前は", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertEqual(res.get("commands"), [])
        self.assertIn("シリウス", res.get("speak", ""))

        res = parse_local_rules("ういんくして", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("ウインクしてみたのだ", res.get("speak", ""))

        res = parse_local_rules("センサーは何がある？", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertIn("Hokuyo", res.get("speak", ""))

        res = parse_local_rules("ダジャレ行って", self.state_info)
        self.assertIsNotNone(res)
        self.assertTrue(res.get("fast_path"))
        self.assertEqual(res.get("commands"), [])
        self.assertIn("布団", res.get("speak", ""))

    def test_face_effect_commands(self):
        res = parse_local_rules("右見て", self.state_info)
        self.assertIsNotNone(res)
        self.assertEqual(res.get("commands", [])[0]["type"], "look")
        self.assertEqual(res.get("commands", [])[0]["value"], "right")

        res = parse_local_rules("揺れて", self.state_info)
        self.assertIsNotNone(res)
        self.assertEqual(res.get("commands", [])[0]["type"], "effect")
        self.assertEqual(res.get("commands", [])[0]["value"], "shake")

        res = parse_local_rules("表情リセットして", self.state_info)
        self.assertIsNotNone(res)
        self.assertEqual(res.get("commands", [])[0]["type"], "reset")

    def test_ambiguous_commands_ask_for_clarification(self):
        for text, expected in [
            ("顔制御して", "顔をどうしますか"),
            ("移動して", "どこへ移動"),
            ("動いて", "どう動きますか"),
            ("回って", "どちらに何度"),
            ("手前に来て", "どの位置まで"),
        ]:
            res = parse_local_rules(text, self.state_info)
            self.assertIsNotNone(res)
            self.assertTrue(res.get("fast_path"))
            self.assertEqual(res.get("commands"), [])
            self.assertIn(expected, res.get("speak", ""))

    def test_speed_control(self):
        res = parse_local_rules("ゆっくり進んで", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertTrue(any(c["type"] == "speed" and c["value"] == 0.2 for c in commands))

    def test_humor_control_and_style(self):
        res = parse_local_rules("ユーモアレベルを0.9にして", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(commands[0]["type"], "humor")
        self.assertEqual(commands[0]["value"], 0.9)

        self.assertIn("したで", style_sirius_speak("[happy]目的地に到着したのだ！", 0.9))
        self.assertIn("しました", style_sirius_speak("[happy]目的地に到着したのだ！", 0.0))

    def test_forward_word_inside_non_motion_word_does_not_move(self):
        res = parse_local_rules("あなたの名前は", self.state_info)
        self.assertIsNotNone(res)
        self.assertEqual(res.get("commands"), [])

        res = parse_local_rules("手前に来て", self.state_info)
        self.assertIsNotNone(res)
        self.assertEqual(res.get("commands"), [])

        res = parse_local_rules("ダジャレ行って", self.state_info)
        self.assertIsNotNone(res)
        self.assertEqual(res.get("commands"), [])

        res = parse_local_rules("1メートル前進して", self.state_info)
        self.assertIsNotNone(res)
        self.assertEqual(res.get("commands", [])[0]["type"], "forward")

    def test_battery_query(self):
        res = parse_local_rules("バッテリーはどう？", self.state_info, battery_callback=lambda: "テスト中")
        self.assertIsNotNone(res)
        self.assertEqual(res.get("speak"), "テスト中")

    def test_origin_return(self):
        res = parse_local_rules("原点に戻って", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(commands[0]["type"], "goto")
        self.assertEqual(commands[0]["value"], [0.0, 0.0])

    def test_negative_coordinate_with_unicode_minus(self):
        for text in ["−3,3移動して", "－3，3移動して", "3,ー3に移動して"]:
            res = parse_local_rules(text, self.state_info)
            self.assertIsNotNone(res)
            commands = res.get("commands", [])
            self.assertEqual(commands[0]["type"], "goto")
            expected = [-3.0, 3.0] if text.startswith(("−", "－")) else [3.0, -3.0]
            self.assertEqual(commands[0]["value"], expected)

    def test_system_status_query(self):
        self.state_info["obstacle_distances"] = {"front": 1.2, "left": 999.0, "right": 999.0, "back": 999.0}
        res = parse_local_rules("システム状態は", self.state_info)
        self.assertIsNotNone(res)
        speak = res.get("speak", "")
        self.assertIn("システム状態", speak)
        self.assertIn("現在位置", speak)
        self.assertIn("前方 1.2メートル", speak)
        self.assertNotIn("周りには", speak)

    def test_correction_rules(self):
        self.state_info["last_action_type"] = "forward"
        self.state_info["last_target_value"] = 2.0
        res = parse_local_rules("ちょっと行き過ぎだよ", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(commands[0]["type"], "backward")
        self.assertGreater(commands[0]["value"], 0.0)

    def test_loop_commands(self):
        res = parse_local_rules("3回繰り返して右に90度回る", self.state_info)
        self.assertIsNotNone(res)
        commands = res.get("commands", [])
        self.assertEqual(len(commands), 3)

    def test_obstacle_query(self):
        self.state_info["obstacle_distances"] = {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0}
        res = parse_local_rules("障害物はある？", self.state_info)
        self.assertIsNotNone(res)
        self.assertIn("見当たらない", res.get("speak", ""))

        self.state_info["obstacle_distances"] = {"front": 0.3, "left": 999.0, "right": 999.0, "back": 999.0}
        res = parse_local_rules("どこに障害物がある？", self.state_info)
        self.assertIsNotNone(res)
        self.assertIn("前方 0.3メートル", res.get("speak", ""))

if __name__ == '__main__':
    unittest.main()
