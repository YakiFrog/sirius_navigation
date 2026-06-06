# -*- coding: utf-8 -*-

from .chat import CHAT_KEYWORDS
from .navigation import NAVIGATION_TEMPLATES
from .system import SYSTEM_TEMPLATES

# 下位互換性と分かりやすさのため、すべてのテンプレートを1つの辞書に統合して公開します
DIALOGUE_TEMPLATES = {}
DIALOGUE_TEMPLATES.update(NAVIGATION_TEMPLATES)
DIALOGUE_TEMPLATES.update(SYSTEM_TEMPLATES)
