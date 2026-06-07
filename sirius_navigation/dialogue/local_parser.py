# -*- coding: utf-8 -*-
import re
import math
import json
import urllib.request
import urllib.error
import unicodedata

# 雑談・自己紹介用キーワード
try:
    from .chat import CHAT_KEYWORDS
except ImportError:
    try:
        from dialogue.chat import CHAT_KEYWORDS
    except ImportError:
        CHAT_KEYWORDS = {}

EXPRESSION_KEYWORDS = [
    ("ウィンク", "wink"),
    ("ウインク", "wink"),
    ("wink", "wink"),
    ("笑顔", "happy"),
    ("にこにこ", "happy"),
    ("ニコニコ", "happy"),
    ("怒", "angry"),
    ("おこ", "angry"),
    ("悲", "sad"),
    ("泣", "sad"),
    ("ぴえん", "pien"),
    ("ピエン", "pien"),
    ("驚", "surprised"),
    ("びっくり", "surprised"),
    ("寝", "sleeping"),
    ("目をつぶ", "sleeping"),
    ("おやすみ", "sleeping"),
    ("元に戻", "normal"),
    ("通常に戻", "normal"),
    ("普通にして", "normal"),
    ("リセット", "normal"),
]


def extract_expression_commands(part_norm):
    commands = []
    seen = set()
    candidates = []
    for kw, exp in EXPRESSION_KEYWORDS:
        idx = part_norm.find(kw)
        if idx != -1 and exp not in seen:
            candidates.append((idx, kw, exp))
            seen.add(exp)
    for _, _, exp in sorted(candidates, key=lambda x: x[0]):
        commands.append({"type": "expression", "value": exp})
    return commands


def extract_effect_commands(part_norm):
    commands = []
    seen = set()
    for kw, param_name, amount in [
        ("頬を赤らめ", "blushAmount", 1.0),
        ("頬を赤く", "blushAmount", 1.0),
        ("赤らめ", "blushAmount", 1.0),
        ("赤く", "blushAmount", 1.0),
        ("キラキラ", "sparkleAmount", 1.0),
        ("目をキラキラ", "sparkleAmount", 1.0),
        ("目がキラキラ", "sparkleAmount", 1.0),
        ("キラッ", "sparkleAmount", 1.0),
    ]:
        if kw in part_norm and param_name not in seen:
            commands.append({"type": "parameter", "value": {"name": param_name, "amount": amount}})
            seen.add(param_name)
    return commands


def extract_face_commands(part_norm):
    candidates = []
    seen = set()
    for kw, exp in EXPRESSION_KEYWORDS:
        idx = part_norm.find(kw)
        key = ("expression", exp)
        if idx != -1 and key not in seen:
            candidates.append((idx, {"type": "expression", "value": exp}))
            seen.add(key)
    for kw, param_name, amount in [
        ("頬を赤らめ", "blushAmount", 1.0),
        ("頬を赤く", "blushAmount", 1.0),
        ("赤らめ", "blushAmount", 1.0),
        ("赤く", "blushAmount", 1.0),
        ("目をキラキラ", "sparkleAmount", 1.0),
        ("目がキラキラ", "sparkleAmount", 1.0),
        ("キラキラ", "sparkleAmount", 1.0),
        ("キラッ", "sparkleAmount", 1.0),
    ]:
        idx = part_norm.find(kw)
        key = ("parameter", param_name)
        if idx != -1 and key not in seen:
            candidates.append((idx, {"type": "parameter", "value": {"name": param_name, "amount": amount}}))
            seen.add(key)
    return [cmd for _, cmd in sorted(candidates, key=lambda x: x[0])]

# テンプレート群
DIALOGUE_TEMPLATES = {
    "arrival": "[happy]目的地に到着したのだ！",
    "stuck": "[sad]進めなくなっちゃったのだ！障害物があるかもしれないのだ。",
    "parse_failure": "[sad]指示の理解に失敗したのだ。",
    "forward_start": "[happy]{distance:.1f}メートル前進するのだ！",
    "backward_start": "[happy]{distance:.1f}メートル後退するのだ！",
    "turn_success": "[happy]旋回が完了したのだ！",
    "turn_failure": "[sad]旋回に失敗したのだ。",
    "speed_change": "[happy]速度を {speed:.2f} に変更するのだ！",
    "goto_start": "[happy]座標 X{x:.1f}、Y{y:.1f} に向かうのだ！",
    "battery_report": "[happy]現在のバッテリー残量は {level:.1f}パーセントなのだ！状態は {charging_str} なのだ。",
    "battery_error": "[sad]バッテリー残量データが不正なのだ。",
    "battery_fail": "[sad]バッテリー状態が確認できないのだ。",
}

EXPRESSION_JA = {
    "normal": "落ち着いた普通の気分",
    "happy": "うれしい気分",
    "angry": "ちょっと怒っている気分",
    "sad": "少ししょんぼりした気分",
    "surprised": "びっくりしている気分",
    "cat": "少し得意げな気分",
    "wink": "ちょっとおちゃめな気分",
    "pien": "ぴえんな気分",
    "sleeping": "眠たい気分",
}

def normalize_kanji_numbers(s):
    s = s.replace("点", ".")
    comp_map = {
        "二十": "2", "thirty": "3", "三十": "3", "四十": "4", "五十": "5",
        "六十": "6", "七十": "7", "八十": "8", "九十": "9"
    }
    for k, v in comp_map.items():
        s = s.replace(k + "〇", v + "0")
        s = s.replace(k + "一", v + "1")
        s = s.replace(k + "二", v + "2")
        s = s.replace(k + "三", v + "3")
        s = s.replace(k + "四", v + "4")
        s = s.replace(k + "五", v + "5")
        s = s.replace(k + "六", v + "6")
        s = s.replace(k + "七", v + "7")
        s = s.replace(k + "八", v + "8")
        s = s.replace(k + "九", v + "9")
        s = s.replace(k, v + "0")
    
    teens_map = {
        "十一": "11", "十二": "12", "十三": "13", "十四": "14", "十五": "15",
        "十六": "16", "十七": "17", "十八": "18", "十九": "19", "十": "10"
    }
    for k, v in teens_map.items():
        s = s.replace(k, v)
        
    singles = {
        "〇": "0", "一": "1", "二": "2", "三": "3", "四": "4",
        "五": "5", "六": "6", "七": "7", "八": "8", "九": "9"
    }
    for k, v in singles.items():
        s = s.replace(k, v)
    return s

def parse_part(part_raw):
    part_norm = normalize_kanji_numbers(part_raw.strip().lower())
    
    # 1. Goto coordinates
    coord_match = re.search(r"(?:goto|go\s*to|座標指定|座標|目標座標)?\s*\(?\s*(-?\d+(?:\.\d+)?)\s*[,，\s]\s*(-?\d+(?:\.\d+)?)\s*\)?", part_norm)
    if coord_match and ("座標" in part_norm or "goto" in part_norm or "," in part_norm):
        try:
            x = float(coord_match.group(1))
            y = float(coord_match.group(2))
            return {"type": "goto", "value": [x, y]}
        except ValueError:
            pass

    # 2. Speed
    speed_match = re.search(r"(?:速度|スピード|speed)\s*(\d+(?:\.\d+)?)", part_norm)
    if speed_match:
        return {"type": "speed", "value": float(speed_match.group(1))}
    speed_unit_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m/s|の速度)", part_norm)
    if speed_unit_match:
        return {"type": "speed", "value": float(speed_unit_match.group(1))}

    # 3. Compass face directions
    face_map = {
        "北": 90.0, "kita": 90.0,
        "東": 0.0, "higashi": 0.0,
        "南": -90.0, "minami": -90.0,
        "西": 180.0, "nishi": 180.0,
        "前": 0.0, "正面": 0.0, "mae": 0.0, "shoumen": 0.0
    }
    for d_name, angle in face_map.items():
        if d_name in part_norm and ("向" in part_norm or "向き" in part_norm or "むい" in part_norm or "むく" in part_norm or "face" in part_norm):
            return {"type": "face", "value": angle}

    # 4. Turn/Spin with numbers
    angle_match = re.search(r"(\d+(?:\.\d+)?)\s*(度|deg|°|rad|ラジアン|回転|旋回)", part_norm)
    if not angle_match:
        angle_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:に)?\s*(右|左|migi|hidari|旋回|回転|時計回り|反時計回り)", part_norm)
        
    if angle_match:
        val_str = angle_match.group(1)
        val = float(val_str)
        
        is_right = any(x in part_norm for x in ["右", "migi", "時計回り", "cw"])
        is_left = any(x in part_norm for x in ["左", "hidari", "反時計回り", "ccw"])
        direction_sign = -1.0 if is_right else 1.0
        is_spin = any(x in part_norm for x in ["旋回", "回転", "spin"])
        
        if "rad" in part_norm or "ラジアン" in part_norm:
            rad_val = val * direction_sign
            if is_spin:
                return {"type": "spin", "value": math.degrees(rad_val)}
            else:
                return {"type": "turn", "value": rad_val}
        else:
            unit_str = angle_match.group(2) if len(angle_match.groups()) > 1 else ""
            if unit_str in ["回転", "旋回"]:
                deg_val = val * 360.0 * direction_sign
                return {"type": "spin", "value": deg_val}
            
            deg_val = val * direction_sign
            if is_spin:
                return {"type": "spin", "value": deg_val}
            else:
                return {"type": "turn", "value": math.radians(deg_val)}

    # 5. Forward/Backward with numbers
    dist_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|メートル|cm|センチ)?", part_norm)
    if dist_match:
        val = float(dist_match.group(1))
        if "cm" in part_norm or "センチ" in part_norm:
            val = val / 100.0
            
        is_backward = any(x in part_norm for x in ["下", "後退", "sagatt", "usirosag", "ushirosag", "back", "reverse", "さがって", "後ろ", "うしろ", "ushiro", "backward"])
        is_forward = any(x in part_norm for x in ["前", "進", "mae", "forward", "straight", "まえ", "すす", "行っ"])
        
        if is_backward:
            return {"type": "backward", "value": val}
        elif is_forward:
            return {"type": "forward", "value": val}
    return None

def parse_no_number_part(part_raw):
    part_norm = part_raw.strip().lower()
    
    # 1. Compass/Face directions
    face_map = {
        "北": 90.0, "kita": 90.0,
        "東": 0.0, "higashi": 0.0,
        "南": -90.0, "minami": -90.0,
        "西": 180.0, "nishi": 180.0,
        "前": 0.0, "正面": 0.0, "mae": 0.0, "shoumen": 0.0
    }
    for d_name, angle in face_map.items():
        if d_name in part_norm and ("向" in part_norm or "向き" in part_norm or "むい" in part_norm or "むく" in part_norm or "face" in part_norm):
            return {"type": "face", "value": angle}

    # 2. Expression control rules
    if any(x in part_norm for x in ["顔", "表情", "しよ", "して", "むいて", "になって", "戻", "通常", "普通", "目", "頬", "キラキラ", "ウインク", "ウィンク", "wink"]):
        face_cmds = extract_face_commands(part_norm)
        if face_cmds:
            return face_cmds[0] if len(face_cmds) == 1 else {"type": "face_sequence", "value": face_cmds}

    is_right = any(pat in part_norm for pat in ["右", "migi", "みぎ", "みぎむ"])
    is_left = any(pat in part_norm for pat in ["左", "hidari", "ひだり", "ひだりむ"])
    is_back = any(pat in part_norm for pat in ["後ろ", "うしろ", "ushiro", "裏", "うら"])

    if any(x in part_norm for x in ["旋回", "回転", "senkai", "spin", "まわ", "回っ", "回って"]):
        deg = 360.0
        if is_right or "時計回り" in part_norm:
            deg = -360.0
        return {"type": "spin", "value": deg}
    
    if (is_right or is_left or is_back) and not any(x in part_norm for x in ["前", "進", "下", "後退", "sagatt", "usirosag", "ushirosag", "back", "mae", "すす"]):
        if is_back:
            val = 3.14159
        else:
            val = -1.5708 if is_right else 1.5708
            if any(x in part_norm for x in ["少し", "ちょっと", "微", "すこし"]):
                val = -0.5236 if is_right else 0.5236
        return {"type": "turn", "value": val}

    if any(x in part_norm for x in ["下", "後退", "sagatt", "usirosag", "ushirosag", "back", "reverse", "さがって", "後ろ", "うしろ", "ushiro"]):
        val = 1.0
        if any(x in part_norm for x in ["motto", "もっと", "大きく", "たくさん", "さらに"]):
            val = 2.0
        return {"type": "backward", "value": val}

    if any(x in part_norm for x in ["前", "進", "mae", "forward", "straight", "まえ", "すす", "行っ"]):
        val = 1.5
        if any(x in part_norm for x in ["少し", "ちょっと", "すこし"]):
            val = 1.0
        elif any(x in part_norm for x in ["もっと", "大きく", "たくさん", "さらに"]):
            val = 2.5
        return {"type": "forward", "value": val}
    return None

def parse_any_part(part_raw):
    cmd = parse_part(part_raw)
    if cmd is None:
        cmd = parse_no_number_part(part_raw)
    return cmd

def parse_local_rules(instruction, state_info, battery_callback=None):
    """
    ローカルルールベース判定。LLMを呼び出す前に、シンプルな表現をルールベースで直接解析する。
    返り値: dict (成功時), None (マッチせずLLMへフォールバックが必要な場合)
    """
    norm_inst = instruction.strip().replace(" ", "").replace("　", "").lower()
    
    # 1. 停止・キャンセルの判定
    cancel_patterns = ["止ま", "とまれ", "止め", "とめ", "ストップ", "停止", "stop", "cancel", "キャンセル", "とまって", "待機", "だめ", "無理", "おわり"]
    if any(pat in norm_inst for pat in cancel_patterns):
        return {"commands": [], "cancel": True}
    
    # 2. バッテリー情報の判定
    if any(x in norm_inst for x in ["バッテリー", "ばってりー", "電池", "でんち"]):
        if battery_callback:
            battery_msg = battery_callback()
            return {"commands": [], "cancel": False, "fast_path": True, "speak": battery_msg}
        return {"commands": [], "cancel": False, "fast_path": True}

    # 2.2 原点復帰は「直前動作の取り消し」より優先して、map座標(0,0)への移動として扱う
    if any(x in norm_inst for x in ["原点", "げんてん", "origin", "ホーム", "home"]):
        if any(x in norm_inst for x in ["戻", "もど", "行", "い", "移動", "向か", "帰", "かえ"]):
            return {
                "commands": [{"type": "goto", "value": [0.0, 0.0]}],
                "cancel": False,
                "fast_path": True,
                "speak": "[happy]原点、Xゼロ、Yゼロに向かうのだ！"
            }

    # 2.5 表情・演出の高速判定
    face_cmds = extract_face_commands(norm_inst)
    if face_cmds:
        cmds = face_cmds
        speak_msg = "[happy]表情を変えるのだ！" if len(cmds) == 1 else "[happy]表情を順番に変えるのだ！"
        return {"commands": cmds, "cancel": False, "fast_path": True, "speak": speak_msg}
        
    # 3. 雑談・キャラクター紹介の高速応答
    for kw, reply in CHAT_KEYWORDS.items():
        if kw in norm_inst:
            return {"commands": [], "cancel": False, "fast_path": True, "speak": reply}

    # 3.5 機能説明・能力質問の高速応答
    capability_queries = ["なにができる", "何ができる", "何ができるの", "なにができるの", "できること", "何が可能", "できる?", "できる？", "何が得意"]
    if any(q in norm_inst for q in capability_queries):
        speak_msg = (
            "[happy]ボクは前進、後退、右向き、左向き、その場旋回、座標への移動、速度変更、"
            "表情変更、停止、再開、キャンセルができるのだ！"
        )
        return {"commands": [], "cancel": False, "fast_path": True, "speak": speak_msg}

    smalltalk_queries = [
        "じゃんけん", "あっち向いて", "なぞなぞ", "クイズ", "冗談", "ジョーク",
        "何が見える", "なにが見える", "周りに何が見える", "周囲に何が見える"
    ]
    if any(q in norm_inst for q in smalltalk_queries):
        return {
            "commands": [],
            "cancel": False,
            "fast_path": True,
            "speak": "[happy]ごめんなさい、ボクはナビゲーション用のロボットなのだ。周りの人やバッテリー、位置や状態の説明はできるけれど、遊びや視覚の判定はできないのだ。"
        }

    weather_queries = ["今日の天気", "天気は", "天気どう", "天気", "weather"]
    if any(q in norm_inst for q in weather_queries):
        return {
            "commands": [],
            "cancel": False,
            "fast_path": True,
            "speak": "[sad]ごめんなさい、私はナビゲーション用のアシスタントなので、お天気の情報は持っていないのだ。"
        }

    # 4. 状況・気分・エラー確認クエリの判定
    status_queries = ["状況", "状態", "ステータス", "どうなってる", "何してる", "どこにいる"]
    people_queries = [
        "何人", "何にん", "人数", "人の数", "周りの人", "周囲の人", "周囲に人",
        "周りに何人", "周りに何にん", "何人いる", "何人いるか", "人数は", "人が何人",
        "教えて", "いるか"
    ]
    mood_queries = ["気分", "機嫌", "きぶん", "きげん", "調子どう", "調子は", "元気", "げんき"]
    error_queries = ["なんで失敗", "何で失敗", "なぜ失敗", "なぜ止まった", "なんで止まった", "何で止まった", "失敗した理由", "止まった理由"]
    is_status_query = any(q in norm_inst for q in status_queries)
    is_people_query = any(q in norm_inst for q in people_queries)
    is_mood_query = any(q in norm_inst for q in mood_queries)
    is_error_query = any(q in norm_inst for q in error_queries)
    
    if is_status_query or is_people_query or is_mood_query or is_error_query:
        current_x = state_info.get("current_x", 0.0)
        current_y = state_info.get("current_y", 0.0)
        executing = state_info.get("executing", False)
        queue_len = state_info.get("queue_len", 0)
        stuck = state_info.get("stuck", False)
        people_count = state_info.get("people_count", 0)
        face_active = state_info.get("face_active", False)
        current_expression = state_info.get("current_expression", "normal")
        
        speak_msg = ""
        if is_mood_query:
            mood_str = EXPRESSION_JA.get(current_expression, f"{current_expression}な気分")
            if stuck:
                speak_msg = f"[sad]今は{mood_str}だけど、進路が塞がれていて少し困っているのだ。"
            elif executing:
                speak_msg = f"[happy]今は{mood_str}なのだ！移動中だから、ちょっと集中しているのだ。"
            else:
                speak_msg = f"[happy]今は{mood_str}なのだ！いつでも次の指示を待っているのだ。"
        elif is_status_query or is_people_query:
            people_str = f"ゼロ人" if people_count == 0 else f"{people_count}人"
            
            if stuck:
                speak_msg = f"[sad]周りには {people_str} の人が検知されているのだ。"
            elif executing:
                speak_msg = f"[happy]周りには {people_str} の人が検知されているのだ。"
            else:
                speak_msg = f"[happy]周りには {people_str} の人が検知されているのだ。"
        else:
            last_status = state_info.get("last_action_status", "")
            last_type = state_info.get("last_action_type", "")
            if last_status == "failed_stuck":
                speak_msg = "[sad]さっきは進もうとしたんだけど、行く手が障害物で遮られちゃって、これ以上進めなくて停止したのだ。"
            elif last_status in ["failed_cancelled", "cancelled"]:
                speak_msg = "[happy]さっきはユーザー指示で動作を途中でキャンセルしたのだ。"
            elif last_type == "turn" and last_status == "failed":
                speak_msg = "[sad]さっきは旋回しようとしたんだけど、目標の角度まで回りきれずに途中で止まっちゃったのだ。"
            else:
                speak_msg = "[happy]直前のアクションは正常に完了しているか、まだエラーは発生していないのだ！"
        
        return {"commands": [], "cancel": False, "fast_path": True, "speak": speak_msg}

    # 5. 是正・訂正・相対調整のルールベース判定
    correction_patterns_map = {
        "行き過ぎ": "overshot", "いきすぎ": "overshot", "回りすぎ": "overshot", "まわりすぎ": "overshot",
        "進みすぎ": "overshot", "すすみすぎ": "overshot", "下がりすぎ": "overshot", "さがりすぎ": "overshot",
        "動きすぎ": "overshot", "うごきすぎ": "overshot", "すぎだ": "overshot", "すぎよ": "overshot",
        "逆": "opposite", "ぎゃく": "opposite", "反対": "opposite", "はんたい": "opposite",
        "足りない": "more", "たりない": "more", "不足": "more", "ふそく": "more",
        "戻って": "goback", "もどって": "goback", "やり直し": "goback", "やりなおし": "goback"
    }
    correction_type = None
    for kw, c_t in correction_patterns_map.items():
        if kw in norm_inst:
            correction_type = c_t
            break
            
    if correction_type is not None:
        last_type = state_info.get("last_action_type", "")
        last_target = state_info.get("last_target_value", 0.0)
        start_x = state_info.get("last_action_start_x", 0.0)
        start_y = state_info.get("last_action_start_y", 0.0)
        start_yaw = state_info.get("last_action_start_yaw", 0.0)
        
        cmd_list = []
        speak_msg = ""
        
        if correction_type == "overshot":
            if last_type == "forward":
                val = max(0.3, last_target * 0.3 if isinstance(last_target, (int, float)) else 0.5)
                cmd_list.append({"type": "backward", "value": val})
                speak_msg = f"[sad]ごめんなさい、ちょっと進みすぎちゃったのだ！{val:.1f}メートル下がるのだ。"
            elif last_type == "backward":
                val = max(0.3, last_target * 0.3 if isinstance(last_target, (int, float)) else 0.5)
                cmd_list.append({"type": "forward", "value": val})
                speak_msg = f"[sad]ごめんなさい、ちょっと下がりすぎちゃったのだ！{val:.1f}メートル進むのだ。"
            elif last_type in ["turn", "spin"]:
                val = -last_target * 0.3 if isinstance(last_target, (int, float)) else -0.5
                cmd_list.append({"type": last_type, "value": val})
                dir_str = "右" if val < 0 else "左"
                speak_msg = f"[sad]ごめんなさい、ちょっと回りすぎちゃったのだ！少し{dir_str}に戻るのだ。"
                
        elif correction_type == "opposite":
            if last_type == "forward":
                cmd_list.append({"type": "backward", "value": last_target if isinstance(last_target, (int, float)) else 1.0})
                speak_msg = "[surprised]あ、逆向きだったのだ！反対側に進むのだ。"
            elif last_type == "backward":
                cmd_list.append({"type": "forward", "value": last_target if isinstance(last_target, (int, float)) else 1.0})
                speak_msg = "[surprised]あ、逆向きだったのだ！反対側に進むのだ。"
            elif last_type in ["turn", "spin"]:
                val = -last_target if isinstance(last_target, (int, float)) else -1.57
                cmd_list.append({"type": last_type, "value": val})
                speak_msg = "[surprised]あ、逆向きだったのだ！反対側を向くのだ。"
                
        elif correction_type == "more":
            if last_type in ["forward", "backward"]:
                val = max(0.5, last_target * 0.5 if isinstance(last_target, (int, float)) else 1.0)
                cmd_list.append({"type": last_type, "value": val})
                speak_msg = f"[happy]もう少し動かすのだ！追加で {val:.1f}メートル移動するのだ。"
            elif last_type in ["turn", "spin"]:
                val = last_target * 0.5 if isinstance(last_target, (int, float)) else 0.5
                cmd_list.append({"type": last_type, "value": val})
                speak_msg = "[happy]もう少し回るのだ！追加で旋回するのだ。"
                
        elif correction_type == "goback":
            if last_type in ["forward", "backward", "goto"]:
                cmd_list.append({"type": "goto", "value": [start_x, start_y]})
                cmd_list.append({"type": "face", "value": math.degrees(start_yaw)})
                speak_msg = "[happy]元いた場所に戻るのだ！"
            elif last_type in ["turn", "spin"]:
                cmd_list.append({"type": "face", "value": math.degrees(start_yaw)})
                speak_msg = "[happy]元の向きに戻るのだ！"
                
        if cmd_list:
            return {"commands": cmd_list, "cancel": False, "speak": speak_msg, "fast_path": True}

    # 6. 四角・正方形などの図形描画
    shape_match = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|メートル)?の(?:四角|正方形|スクエア)", norm_inst)
    if shape_match:
        size = float(shape_match.group(1))
        cmds = []
        for _ in range(4):
            cmds.append({"type": "forward", "value": size})
            cmds.append({"type": "turn", "value": -1.5708})
        return {"commands": cmds, "cancel": False, "speak": f"[happy]一辺{size}メートルの正方形を描くように移動するのだ！"}

    # 7. 繰り返し（ループ）
    loop_match = re.search(r"(\d+)\s*(?:回|回繰り返して|回繰り返し)", norm_inst)
    if loop_match:
        times = int(loop_match.group(1))
        base_part = re.sub(r"(?:のを)?\d+\s*(?:回|回繰り返して|回繰り返し).*", "", norm_inst).strip()
        base_cmds = []
        parts = [p.strip() for p in re.split(r"して|て|、|そして", base_part) if p.strip()]
        for part in parts:
            parsed_cmd = parse_any_part(part)
            if parsed_cmd:
                if isinstance(parsed_cmd, dict) and parsed_cmd.get("type") in ["effect_sequence", "face_sequence"]:
                    base_cmds.extend(parsed_cmd.get("value", []))
                elif isinstance(parsed_cmd, dict) and parsed_cmd.get("type") == "expression_sequence":
                    base_cmds.extend(parsed_cmd.get("value", []))
                else:
                    base_cmds.append(parsed_cmd)
        if base_cmds:
            cmds = base_cmds * times
            return {"commands": cmds, "cancel": False, "speak": f"[happy]指示された動きを{times}回繰り返すのだ！"}

    # 8. 一般的な数値指定コマンドのパース（複数コマンドの連結に対応）
    has_number = any(char.isdigit() for char in norm_inst) or any(x in norm_inst for x in ["一", "二", "三", "四", "五", "六", "七", "八", "九", "十", "点", "度", "㍍", "メートル"])
    if has_number:
        parts = [p.strip() for p in re.split(r"して|て|、|そして", norm_inst) if p.strip()]
        parsed_cmds = []
        for part in parts:
            cmd = parse_any_part(part)
            if cmd:
                if isinstance(cmd, dict) and cmd.get("type") in ["effect_sequence", "face_sequence"]:
                    parsed_cmds.extend(cmd.get("value", []))
                elif isinstance(cmd, dict) and cmd.get("type") == "expression_sequence":
                    parsed_cmds.extend(cmd.get("value", []))
                else:
                    parsed_cmds.append(cmd)
        if parsed_cmds:
            return {"commands": parsed_cmds, "cancel": False}

    # 9. 数値指定を含まない標準的な指示（簡易前進・後退・旋回・スピード変更）
    slow_patterns = ["ゆっくり", "遅く", "おそく", "おそい", "遅い", "スピード下げ", "スピード落と", "速度下げ", "yukkuri", "slow", "下げて", "スピードおと"]
    normal_patterns = ["ふつう", "普通", "通常", "normal"]
    fast_patterns = ["早く", "急いで", "スピード上げ", "速度上げ", "fast", "speedup", "上げて"]
    
    speed_val = None
    if any(pat in norm_inst for pat in slow_patterns):
        speed_val = 0.2
    elif any(pat in norm_inst for pat in normal_patterns):
        speed_val = 0.9
    elif any(pat in norm_inst for pat in fast_patterns):
        speed_val = 1.0

    cmd_list = []
    if speed_val is not None:
        cmd_list.append({"type": "speed", "value": speed_val})
        
    no_num_cmd = parse_no_number_part(norm_inst)
    if no_num_cmd:
        if isinstance(no_num_cmd, dict) and no_num_cmd.get("type") in ["expression_sequence", "face_sequence", "effect_sequence"]:
            cmd_list.extend(no_num_cmd.get("value", []))
        else:
            cmd_list.append(no_num_cmd)
        return {"commands": cmd_list, "cancel": False}

    has_dir = any(x in norm_inst for x in ["前", "進", "下", "後退", "sagatt", "usirosag", "ushirosag", "back", "mae", "右", "左", "migi", "hidari", "旋回", "回転", "senkai", "spin", "goto", "座標", "すす", "行っ", "さが", "下が", "さがっ", "まわ", "回っ", "むい", "後ろ", "うしろ", "ushiro", "裏", "うら"])
    if has_dir:
        last_cmd_was_backward = state_info.get("last_cmd_was_backward", False)
        last_cmd_was_forward = state_info.get("last_cmd_was_forward", False)
        
        if norm_inst in ["motto", "もっと", "もうすこし", "もう少し", "もうちょっと", "さらに", "すこし", "少し", "ちょっと"]:
            val = 1.0 if any(x in norm_inst for x in ["すこし", "少し", "ちょっと"]) else 2.0
            if last_cmd_was_backward:
                cmd_list.append({"type": "backward", "value": max(1.0, val)})
                return {"commands": cmd_list, "cancel": False}
            elif last_cmd_was_forward:
                cmd_list.append({"type": "forward", "value": max(1.0, val)})
                return {"commands": cmd_list, "cancel": False}
    else:
        if speed_val is not None:
            return {"commands": cmd_list, "cancel": False}

    return None
