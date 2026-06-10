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
        try:
            from chat import CHAT_KEYWORDS
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

RESET_KEYWORDS = ["リセット", "元に戻", "通常に戻", "普通にして", "表情戻"]

EFFECT_KEYWORDS = [
    ("揺れ", "shake"),
    ("ゆれ", "shake"),
    ("震え", "shake"),
    ("ふるえ", "shake"),
    ("ブルブル", "shake"),
    ("ぶるぶる", "shake"),
    ("shake", "shake"),
]

LOOK_KEYWORDS = [
    ("右を見", "right"),
    ("右見", "right"),
    ("右に目線", "right"),
    ("右へ目線", "right"),
    ("lookright", "right"),
    ("左を見", "left"),
    ("左見", "left"),
    ("左に目線", "left"),
    ("左へ目線", "left"),
    ("lookleft", "left"),
    ("上を見", "up"),
    ("上見", "up"),
    ("上に目線", "up"),
    ("lookup", "up"),
    ("下を見", "down"),
    ("下見", "down"),
    ("下に目線", "down"),
    ("lookdown", "down"),
    ("正面を見", "center"),
    ("正面見", "center"),
    ("こっち見", "center"),
    ("こっちを見", "center"),
    ("中央を見", "center"),
    ("中心を見", "center"),
    ("lookcenter", "center"),
]

FORWARD_PATTERNS = [
    "前進", "前方", "進め", "進ん", "進む", "進ま",
    "mae", "forward", "straight", "まえ", "すす"
]

BACKWARD_PATTERNS = [
    "後退", "後ろ", "うしろ", "後方", "下が", "下げ", "さがって", "さがれ",
    "sagatt", "usirosag", "ushirosag", "back", "reverse", "ushiro", "backward"
]

LATERAL_TURN_PATTERNS = [
    "右", "左", "migi", "hidari", "みぎ", "ひだり"
]

def contains_any(text, patterns):
    return any(pattern in text for pattern in patterns)

def contains_forward_intent(text):
    if contains_any(text, FORWARD_PATTERNS):
        return True
    return re.search(r"(?<![手名目])前(?:に|へ|を)", text) is not None

def contains_motion_intent(text):
    return (
        contains_forward_intent(text)
        or contains_any(text, BACKWARD_PATTERNS)
        or contains_any(text, [
            "旋回", "回転", "senkai", "spin", "goto", "座標", "まわ", "回っ",
            "むい", "向い", "向き", "裏", "うら"
        ] + LATERAL_TURN_PATTERNS)
    )


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
    for kw in RESET_KEYWORDS:
        idx = part_norm.find(kw)
        key = ("reset", None)
        if idx != -1 and key not in seen:
            candidates.append((idx, {"type": "reset", "value": None}))
            seen.add(key)
    for kw, exp in EXPRESSION_KEYWORDS:
        idx = part_norm.find(kw)
        key = ("expression", exp)
        if exp == "normal" and ("reset", None) in seen:
            continue
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
    for kw, effect_type in EFFECT_KEYWORDS:
        idx = part_norm.find(kw)
        key = ("effect", effect_type)
        if idx != -1 and key not in seen:
            candidates.append((idx, {"type": "effect", "value": effect_type}))
            seen.add(key)
    for kw, direction in LOOK_KEYWORDS:
        idx = part_norm.find(kw)
        key = ("look", direction)
        if idx != -1 and key not in seen:
            candidates.append((idx, {"type": "look", "value": direction}))
            seen.add(key)
    return [cmd for _, cmd in sorted(candidates, key=lambda x: x[0])]

# テンプレート群
try:
    from .navigation import NAVIGATION_TEMPLATES
    from .system import SYSTEM_TEMPLATES
except ImportError:
    try:
        from navigation import NAVIGATION_TEMPLATES
        from system import SYSTEM_TEMPLATES
    except ImportError:
        NAVIGATION_TEMPLATES = {}
        SYSTEM_TEMPLATES = {}

DIALOGUE_TEMPLATES = {}
DIALOGUE_TEMPLATES.update(NAVIGATION_TEMPLATES)
DIALOGUE_TEMPLATES.update(SYSTEM_TEMPLATES)


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

DEFAULT_HUMOR_LEVEL = 0.7

def clamp_humor_level(value):
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return DEFAULT_HUMOR_LEVEL

def strip_expression_tag(text):
    match = re.match(r"^(\[[^\]]+\])(.+)$", text or "")
    if match:
        return match.group(1), match.group(2)
    return "", text or ""

def infer_expression_tag(body):
    text = body or ""
    lowered = text.lower()
    if re.match(r"^\[[^\]]+\]", text):
        return ""
    if any(word in text for word in ["ごめん", "申し訳", "失敗", "無理", "できない", "確認できない", "わから", "分から", "障害物"]):
        return "[sad]"
    if any(word in text for word in ["どこ", "どの", "どう", "何を", "教えて", "詳しい", "指定", "かな", "やろか", "ですか"]):
        return "[normal]"
    if any(word in text for word in ["うわ", "びっくり", "危な", "アカン", "だめ", "ダメ"]):
        return "[surprised]"
    if any(word in text for word in ["ダジャレ", "ジョーク", "冗談", "ウインク", "布団", "ミカン"]) or "wink" in lowered:
        return "[wink]"
    if any(word in text for word in ["了解", "到着", "できる", "行く", "向かう", "進む", "再開", "サービス", "最高", "よろしく"]):
        return "[happy]"
    return "[normal]"

def style_sirius_speak(text, humor_level=DEFAULT_HUMOR_LEVEL):
    """SiriusFace のユーモアレベルに合わせて、ナビ側の定型文も軽く口調統一する。"""
    tag, body = strip_expression_tag(text)
    humor = clamp_humor_level(humor_level)
    if not body:
        return text
    if not tag:
        tag = infer_expression_tag(body)

    if humor <= 0.0:
        polite = body
        replacements = [
            ("ボク", "私"),
            ("僕", "私"),
            ("したのだ！", "しました。"),
            ("したのだ。", "しました。"),
            ("するのだ！", "します。"),
            ("するのだ。", "します。"),
            ("ほしいのだ。", "ください。"),
            ("なのだ！", "です。"),
            ("なのだ。", "です。"),
            ("なのだ？", "ですか？"),
            ("のだ！", "です。"),
            ("のだ。", "です。"),
            ("ちゃった", "しました"),
            ("ごめんなさい、", "申し訳ありません、"),
        ]
        for src, dst in replacements:
            polite = polite.replace(src, dst)
        return f"{tag}{polite}"

    kansai = body.replace("ボク", "僕")
    if humor >= 0.5:
        replacements = [
            ("したのだ！", "したで！"),
            ("したのだ。", "したで。"),
            ("するのだ！", "するで！"),
            ("するのだ。", "するで。"),
            ("向かうのだ！", "向かうで！"),
            ("戻すのだ！", "戻すで！"),
            ("変えるのだ！", "変えるで！"),
            ("ほしいのだ。", "ほしいんや。"),
            ("なのだ！", "やで！"),
            ("なのだ。", "やで。"),
            ("なのだ？", "やろか？"),
            ("のだ！", "やで！"),
            ("のだ。", "やで。"),
        ]
        for src, dst in replacements:
            kansai = kansai.replace(src, dst)
    else:
        replacements = [
            ("なのだ！", "なのだ。"),
            ("のだ！", "のだ。"),
            ("やで！", "やで。"),
        ]
        for src, dst in replacements:
            kansai = kansai.replace(src, dst)
    return f"{tag}{kansai}"

def parse_humor_command(norm_inst):
    if not any(word in norm_inst for word in ["ユーモア", "ゆーもあ", "humor", "真面目", "まじめ", "デフォルト"]):
        return None

    if any(word in norm_inst for word in ["デフォルト", "標準", "戻"]):
        return {"type": "humor", "value": DEFAULT_HUMOR_LEVEL}
    if any(word in norm_inst for word in ["真面目", "まじめ", "敬語", "丁寧"]):
        return {"type": "humor", "value": 0.0}
    if any(word in norm_inst for word in ["高め", "上げ", "おもしろ", "面白", "生意気"]):
        return {"type": "humor", "value": 0.9}
    if any(word in norm_inst for word in ["低め", "下げ", "控えめ"]):
        return {"type": "humor", "value": 0.3}

    match = re.search(r"(?:ユーモア|ゆーもあ|humor)(?:レベル)?(?:を|は|=|:)?([01](?:\.\d+)?)", norm_inst)
    if match:
        return {"type": "humor", "value": clamp_humor_level(match.group(1))}
    return None

def build_clarification_response(norm_inst):
    """内容不足で動作を推測する必要がある指示なら、確認質問を返す。"""
    vague_face_words = ["顔制御", "顔を制御", "顔操作", "顔を操作", "表情制御", "表情を制御", "表情操作", "表情を操作"]
    vague_face_change_words = ["顔変えて", "顔を変えて", "表情変えて", "表情を変えて"]
    if any(word in norm_inst for word in vague_face_words + vague_face_change_words):
        return {
            "commands": [],
            "cancel": False,
            "fast_path": True,
            "speak": "[normal]顔をどうしますか？ウインク、笑顔、右を見る、キラキラなどを指定してほしいのだ。"
        }

    vague_motion_exact = {
        "移動して": "[normal]どこへ移動しますか？座標、前後左右、距離のどれかを指定してほしいのだ。",
        "移動しろ": "[normal]どこへ移動しますか？座標、前後左右、距離のどれかを指定してほしいのだ。",
        "動いて": "[normal]どう動きますか？前進、後退、右向き、左向き、座標移動などで指定してほしいのだ。",
        "動け": "[normal]どう動きますか？前進、後退、右向き、左向き、座標移動などで指定してほしいのだ。",
        "行って": "[normal]どこへ行きますか？目的地か方向を指定してほしいのだ。",
        "行け": "[normal]どこへ行きますか？目的地か方向を指定してほしいのだ。",
        "回って": "[normal]どちらに何度回りますか？右90度、左に旋回、のように指定してほしいのだ。",
        "向いて": "[normal]どちらを向きますか？右、左、後ろ、北などを指定してほしいのだ。",
        "ナビして": "[normal]どこへ案内しますか？目的地か座標を指定してほしいのだ。",
        "案内して": "[normal]どこへ案内しますか？目的地か座標を指定してほしいのだ。",
    }
    if norm_inst in vague_motion_exact:
        return {
            "commands": [],
            "cancel": False,
            "fast_path": True,
            "speak": vague_motion_exact[norm_inst]
        }

    vague_approach_words = ["手前に来", "こっち来", "こっちに来", "近くに来", "近づいて"]
    if any(word in norm_inst for word in vague_approach_words):
        return {
            "commands": [],
            "cancel": False,
            "fast_path": True,
            "speak": "[normal]どの位置まで近づきますか？距離か座標を指定してほしいのだ。"
        }

    vague_control_words = ["制御して", "操作して", "コントロールして"]
    if norm_inst in vague_control_words:
        return {
            "commands": [],
            "cancel": False,
            "fast_path": True,
            "speak": "[normal]何をどう制御しますか？移動、速度、表情などを具体的に指定してほしいのだ。"
        }

    return None

MINUS_TRANSLATION = str.maketrans({
    "−": "-",
    "－": "-",
    "―": "-",
    "–": "-",
    "—": "-",
    "﹣": "-",
    "ｰ": "-",
})

def normalize_instruction_text(s):
    s = unicodedata.normalize("NFKC", s).translate(MINUS_TRANSLATION)
    return re.sub(r"(^|[,\s、(（\[【{=:：xXyY])ー(?=\d)", r"\1-", s)

def normalize_kanji_numbers(s):
    s = normalize_instruction_text(s)
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
    coord_match = re.search(r"(?:^|[^\d.])(?:goto|go\s*to|座標指定|座標|目標座標)?\s*\(?\s*([+-]?\d+(?:\.\d+)?)\s*[,，\s]\s*([+-]?\d+(?:\.\d+)?)\s*\)?", part_norm)
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
    angle_match = re.search(r"(\d+(?:\.\d+)?)\s*(度|deg|°|rad|ラジアン|回転|旋回|周)", part_norm)
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
            if unit_str in ["回転", "旋回", "周"]:
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
            
        is_backward = contains_any(part_norm, BACKWARD_PATTERNS)
        is_forward = contains_forward_intent(part_norm)
        
        if is_backward:
            return {"type": "backward", "value": val}
        elif is_forward:
            return {"type": "forward", "value": val}
    return None

def parse_no_number_part(part_raw):
    part_norm = normalize_instruction_text(part_raw.strip().lower())
    
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
    if any(x in part_norm for x in [
        "顔", "表情", "しよ", "して", "むいて", "になって", "戻", "通常", "普通",
        "目", "頬", "キラキラ", "ウインク", "ウィンク", "wink", "見て", "見る",
        "目線", "揺れ", "ゆれ", "震え", "ふるえ", "ブルブル", "ぶるぶる", "shake"
    ]):
        face_cmds = extract_face_commands(part_norm)
        if face_cmds:
            return face_cmds[0] if len(face_cmds) == 1 else {"type": "face_sequence", "value": face_cmds}

    is_right = any(pat in part_norm for pat in ["右", "migi", "みぎ", "みぎむ"])
    is_left = any(pat in part_norm for pat in ["左", "hidari", "ひだり", "ひだりむ"])
    is_back = any(pat in part_norm for pat in ["後ろ", "うしろ", "ushiro", "裏", "うら"])

    if any(x in part_norm for x in ["旋回", "回転", "senkai", "spin", "まわ", "回っ", "回って", "周", "一周", "1周"]):
        deg = 360.0
        if is_right or "時計回り" in part_norm:
            deg = -360.0
        if is_left or "反時計回り" in part_norm:
            deg = 360.0
        return {"type": "spin", "value": deg}
    
    if (is_right or is_left or is_back) and not (contains_forward_intent(part_norm) or contains_any(part_norm, BACKWARD_PATTERNS)):
        if is_back:
            val = 3.14159
        else:
            val = -1.5708 if is_right else 1.5708
            if any(x in part_norm for x in ["少し", "ちょっと", "微", "すこし"]):
                val = -0.5236 if is_right else 0.5236
        return {"type": "turn", "value": val}

    if contains_any(part_norm, BACKWARD_PATTERNS):
        val = 1.0
        if any(x in part_norm for x in ["motto", "もっと", "大きく", "たくさん", "さらに"]):
            val = 2.0
        return {"type": "backward", "value": val}

    if contains_forward_intent(part_norm):
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
    norm_inst = normalize_instruction_text(instruction).strip().replace(" ", "").replace("　", "").lower()

    humor_cmd = parse_humor_command(norm_inst)
    if humor_cmd:
        return {"commands": [humor_cmd], "cancel": False, "fast_path": True}

    # 0. 現在の速度確認
    speed_queries = ["今の速度", "現在の速度", "速度を教えて", "スピード教えて", "すぴーど教えて", "スピードは", "速度は"]
    if any(q in norm_inst for q in speed_queries):
        speed_setting = state_info.get("current_speed_setting", 0.90)
        vel_x = state_info.get("current_vel_x", 0.0)
        vel_theta = state_info.get("current_vel_theta", 0.0)
        return {
            "commands": [],
            "cancel": False,
            "fast_path": True,
            "speak": f"[happy]現在の速度設定は {speed_setting:.2f}、実速度は前後 {vel_x:.2f} メートル毎秒、回転 {vel_theta:.2f} ラジアン毎秒なのだ。"
        }

    speed_set_match = re.search(r"(?:速度|スピード|すぴーど)?(?:を)?(\d+(?:\.\d+)?)(?:にして|に変更|へ変更)?$", norm_inst)
    if speed_set_match and (
        any(x in norm_inst for x in ["速度", "スピード", "すぴーど"])
        or ("にして" in norm_inst and not (contains_motion_intent(norm_inst) or any(x in norm_inst for x in ["m", "メートル"])))
    ):
        speed_val = float(speed_set_match.group(1))
        return {"commands": [{"type": "speed", "value": speed_val}], "cancel": False}
    
    # 1. 停止・キャンセルの判定
    cancel_patterns = ["止ま", "とまれ", "止め", "とめ", "ストップ", "停止", "stop", "cancel", "キャンセル", "とまって", "待機", "だめ", "無理", "おわり"]
    if any(pat in norm_inst for pat in cancel_patterns):
        return {"commands": [], "cancel": True}

    # 1.5 直前の目標・一時停止からの再開
    resume_goal_queries = ["前回の目的地", "さっきの目的地", "前の目的地", "続き", "続行", "再開", "もう一回同じ", "同じ目的地"]
    if any(q in norm_inst for q in resume_goal_queries):
        return {"commands": [], "cancel": False, "fast_path": True, "speak": "[happy]前回の目標が残っていれば、再開できるのだ。再開して、と言ってほしいのだ。"}
    
    # 2. バッテリー情報の判定
    if any(x in norm_inst for x in ["バッテリー", "ばってりー", "電池", "でんち"]):
        if battery_callback:
            battery_msg = battery_callback()
            return {"commands": [], "cancel": False, "fast_path": True, "speak": battery_msg}
        return {"commands": [], "cancel": False, "fast_path": True}
    # 2.1 障害物情報の判定
    obstacle_keywords = ["障害物", "しょうがいぶつ", "壁", "かべ"]
    if any(q in norm_inst for q in obstacle_keywords) and any(q in norm_inst for q in ["ある", "どこ", "状況", "検知", "確認", "位置"]):
        obs_dists = state_info.get("obstacle_distances", {"front": 999.0, "left": 999.0, "right": 999.0, "back": 999.0})
        detected = []
        for direction in ["front", "back", "left", "right"]:
            dist = obs_dists.get(direction, 999.0)
            if dist < 2.5:
                dir_ja = "前方" if direction == "front" else "後方" if direction == "back" else "左側" if direction == "left" else "右側"
                detected.append(f"{dir_ja} {dist:.1f}メートル")
        if detected:
            min_dist = min([obs_dists.get(d, 999.0) for d in ["front", "back", "left", "right"]])
            tag = "[sad]" if min_dist < 0.8 else "[normal]"
            speak_msg = f"{tag}障害物があるのだ！{', '.join(detected)}の位置に検知しているのだ。"
        else:
            speak_msg = "[happy]周囲 2.5メートル以内には、目立った障害物は見当たらないのだ！"
        return {"commands": [], "cancel": False, "fast_path": True, "speak": speak_msg}
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
        cmd_types = {cmd.get("type") for cmd in cmds}
        if len(cmds) > 1:
            speak_msg = "[happy]顔の動きを順番に変えるのだ！"
        elif "look" in cmd_types:
            speak_msg = "[happy]目線を動かすのだ！"
        elif "effect" in cmd_types:
            speak_msg = "[happy]演出を出すのだ！"
        elif "reset" in cmd_types:
            speak_msg = "[normal]表情を戻すのだ！"
        elif "parameter" in cmd_types:
            speak_msg = "[happy]顔の演出を変えるのだ！"
        else:
            speak_msg = "[happy]表情を変えるのだ！"
        return {"commands": cmds, "cancel": False, "fast_path": True, "speak": speak_msg}

    clarification = build_clarification_response(norm_inst)
    if clarification:
        return clarification
        
    # 3. 雑談・キャラクター紹介の高速応答
    for kw, reply in CHAT_KEYWORDS.items():
        if kw in norm_inst:
            return {"commands": [], "cancel": False, "fast_path": True, "speak": reply}

    # 3.5 機能説明・能力質問の高速応答
    capability_queries = ["なにができる", "何ができる", "何ができるの", "なにができるの", "できること", "何が可能", "できる?", "できる？", "何が得意"]
    if any(q in norm_inst for q in capability_queries):
        speak_msg = (
            "[happy]ボクは前進、後退、右向き、左向き、その場旋回、座標への移動、速度変更、"
            "表情変更、ウインク、照れ、キラキラ、視線移動、揺れ演出、表情リセット、停止、再開、キャンセルができるのだ！"
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
        "人いる", "人がいる", "いる人"
    ]
    mood_queries = ["気分", "機嫌", "きぶん", "きげん", "調子どう", "調子は", "元気", "げんき"]
    error_queries = [
        "なんで失敗", "何で失敗", "なぜ失敗", "なぜ止まった", "なんで止まった", "何で止まった",
        "失敗した理由", "止まった理由", "なぜ曲がれない", "なんで曲がれない", "曲がれない理由",
        "なんでだよ", "なぜだよ", "どうして"
    ]
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
        elif is_status_query:
            obs_dists = state_info.get("obstacle_distances", {})
            obstacle_parts = []
            for direction, label in [("front", "前方"), ("back", "後方"), ("left", "左側"), ("right", "右側")]:
                dist = obs_dists.get(direction, 999.0)
                if isinstance(dist, (int, float)) and dist < 2.5:
                    obstacle_parts.append(f"{label} {dist:.1f}メートル")
            obstacle_text = "、".join(obstacle_parts) if obstacle_parts else "目立った障害物なし"
            move_text = "実行中" if executing else "待機中"
            stuck_text = "詰まり検知あり" if stuck else "詰まり検知なし"
            face_text = "顔サーバー接続中" if face_active else "顔サーバー未接続"
            vel_x = state_info.get("current_vel_x", 0.0)
            vel_theta = state_info.get("current_vel_theta", 0.0)
            speak_msg = (
                f"[happy]システム状態は、{move_text}、{stuck_text}なのだ。"
                f"現在位置は X{current_x:.2f}、Y{current_y:.2f}。"
                f"速度は前後 {vel_x:.2f} メートル毎秒、回転 {vel_theta:.2f} ラジアン毎秒。"
                f"キューは {queue_len} 件、{face_text}。"
                f"障害物は {obstacle_text} なのだ。"
            )
        elif is_people_query:
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
            obs_dists = state_info.get("obstacle_distances", {})
            direction_labels = {
                "front": "前方",
                "back": "後方",
                "left": "左側",
                "right": "右側",
            }
            near_obstacles = [
                f"{direction_labels[key]} {dist:.1f}メートル"
                for key, dist in obs_dists.items()
                if key in direction_labels and isinstance(dist, (int, float)) and dist < 0.8
            ]
            obstacle_text = "、".join(near_obstacles)

            if last_status == "failed_stuck":
                if last_type in ["turn", "spin"]:
                    if obstacle_text:
                        speak_msg = f"[sad]さっきは旋回しようとしたんだけど、{obstacle_text}に障害物が近くて、安全のため途中で止まったのだ。"
                    else:
                        speak_msg = "[sad]さっきは旋回しようとしたんだけど、近くに障害物を検知して安全に回れなかったのだ。"
                else:
                    speak_msg = "[sad]さっきは進もうとしたんだけど、行く手が障害物で遮られちゃって、これ以上進めなくて停止したのだ。"
            elif last_status in ["failed_cancelled", "cancelled"]:
                speak_msg = "[happy]さっきはユーザー指示で動作を途中でキャンセルしたのだ。"
            elif last_type in ["turn", "spin"] and last_status == "failed":
                if obstacle_text:
                    speak_msg = f"[sad]さっきは旋回しようとしたんだけど、{obstacle_text}に障害物が近くて、目標角度まで回りきれなかったのだ。"
                else:
                    speak_msg = "[sad]さっきは旋回しようとしたんだけど、目標の角度まで回りきれずに途中で止まっちゃったのだ。"
            else:
                speak_msg = "[happy]直前のアクションは正常に完了しているか、まだエラーは発生していないのだ！"
        
        return {"commands": [], "cancel": False, "fast_path": True, "speak": speak_msg}

    # 5. 是正・訂正・相対調整のルールベース判定
    correction_patterns_map = {
        "行き過ぎ": "overshot", "いきすぎ": "overshot", "回りすぎ": "overshot", "まわりすぎ": "overshot",
        "進みすぎ": "overshot", "すすみすぎ": "overshot", "下がりすぎ": "overshot", "さがりすぎ": "overshot",
        "動きすぎ": "overshot", "うごきすぎ": "overshot", "すぎだ": "overshot", "すぎよ": "overshot",
        "逆": "opposite", "ぎゃく": "opposite", "反対": "opposite", "はんたい": "opposite", "違う": "opposite", "ちがう": "opposite",
        "そっちじゃない": "opposite", "そっちじゃ無い": "opposite", "方向違う": "opposite",
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
            elif last_type == "goto":
                cmd_list.append({"type": "goto", "value": [start_x, start_y]})
                cmd_list.append({"type": "face", "value": math.degrees(start_yaw)})
                speak_msg = "[surprised]了解、違ったのだ！元いた場所に戻るのだ。"
                
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
        base_part = re.sub(r"(?:のを)?\d+\s*(?:回|回繰り返して|回繰り返し)", "", norm_inst).strip()
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
    slow_patterns = ["ゆっくり", "遅く", "おそく", "おそい", "遅い", "はやすぎ", "速すぎ", "早すぎ", "スピード下げ", "すぴーど下げ", "スピード落と", "すぴーど落と", "速度下げ", "速度落と", "yukkuri", "slow", "下げて", "スピードおと", "すぴーどおと", "低速", "安全速度"]
    normal_patterns = ["ふつう", "普通", "通常", "normal", "標準速度", "いつもの速度", "速度戻", "スピード戻", "すぴーど戻"]
    fast_patterns = ["早く", "はやく", "速く", "急いで", "いそいで", "おそすぎ", "遅すぎ", "スピード上げ", "すぴーど上げ", "速度上げ", "速度アップ", "スピードアップ", "すぴーどあっぷ", "すぴーどあげ", "はやくうご", "もっとはやく", "もっと速く", "fast", "speedup", "上げて", "高速"]
    
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

    has_dir = contains_motion_intent(norm_inst)
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
