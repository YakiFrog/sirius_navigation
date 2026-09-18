"""Shared Nav2 navigation mode definitions and remote-command helpers."""

import json


NAVIGATION_MODE_CONFIGS = {
    "normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.90,
            "FollowPath.time_steps": 60,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 0.90,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.90,
            "FollowPath.ax_min": -0.90,
            "FollowPath.az_max": 1.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.90, 0.0, 0.90],
            "min_velocity": [-0.90, 0.0, -0.90],
            "max_accel": [0.90, 0.0, 1.50],
            "max_decel": [-0.90, 0.0, -1.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "normal_active": {
        "/controller_server": {
            "FollowPath.vx_max": 1.00,
            "FollowPath.time_steps": 54,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 1.00,
            "FollowPath.vx_std": 0.40,
            "FollowPath.wz_std": 0.48,
            "FollowPath.ax_max": 1.50,
            "FollowPath.ax_min": -1.50,
            "FollowPath.az_max": 2.20,
            "FollowPath.CostCritic.cost_weight": 20.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [1.00, 0.0, 1.00],
            "min_velocity": [-0.60, 0.0, -1.00],
            "max_accel": [1.50, 0.0, 2.20],
            "max_decel": [-1.50, 0.0, -2.20],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "safe": {
        "/controller_server": {
            "FollowPath.vx_max": 0.40,
            "FollowPath.time_steps": 135,
            "FollowPath.vx_min": -0.20,
            "FollowPath.wz_max": 0.40,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.20,
            "FollowPath.ax_max": 0.40,
            "FollowPath.ax_min": -0.40,
            "FollowPath.az_max": 1.00,
            "FollowPath.CostCritic.cost_weight": 15.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.40, 0.0, 0.40],
            "min_velocity": [-0.20, 0.0, -0.40],
            "max_accel": [0.40, 0.0, 1.00],
            "max_decel": [-0.40, 0.0, -1.00],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    # slow(4): 速度が低く障害物前で膠着しやすい。
    #   ・CostCritic 20->10: 障害物コストを下げ、過剰回避で固まるのを防ぐ。
    #   ・vx_std/wz_std 拡大: 探索ノイズを増やし、逃げ道となる軌道を見つけやすくする。
    "slow": {
        "/controller_server": {
            "FollowPath.vx_max": 0.20,
            "FollowPath.time_steps": 200,  # 低速でも先読み距離(約4m)を確保するため維持
            "FollowPath.vx_min": -0.10,
            "FollowPath.wz_max": 0.20,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.20,
            "FollowPath.ax_min": -0.20,
            "FollowPath.az_max": 0.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.20, 0.0, 0.20],
            "min_velocity": [-0.10, 0.0, -0.20],
            "max_accel": [0.20, 0.0, 0.50],
            "max_decel": [-0.20, 0.0, -0.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "wait_normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.90,
            "FollowPath.time_steps": 60,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 0.90,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.90,
            "FollowPath.ax_min": -0.90,
            "FollowPath.az_max": 1.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
            "WaitPath.desired_linear_vel": 0.90,
        },
        "/velocity_smoother": {
            "max_velocity": [0.90, 0.0, 0.90],
            "min_velocity": [-0.90, 0.0, -0.90],
            "max_accel": [0.90, 0.0, 1.50],
            "max_decel": [-0.90, 0.0, -1.50],
        },
        # The local costmap remains enabled for collision detection. Only the
        # global dynamic-obstacle layer is disabled so replanning keeps the
        # original static-map route while the wait controller is selected.
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": False},
    },
    # strict系(6/7/8): 「回避しつつパス追従を重視」する設計。
    #   ・global obstacle_layer を False->True: 再計画で障害物を迂回させる。
    #     （旧設定はパスが障害物を貫通し、全軌道衝突→NoValidControl→
    #       BTリカバリのSpinで回り続ける原因になっていた）
    #   ・PathAlignCritic 60->12: normal(8)より高くして追従を強めるが、
    #     過剰な横偏差ペナルティで回避できなくなるのを防ぐ。
    #   ・CostCritic/PathFollow/PathAngle/Twirling/PreferForward/Goal は
    #     normal相当に揃え、strict同士で挙動が発散しないようにする。
    "strict_normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.90,
            "FollowPath.time_steps": 60,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 0.90,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.90,
            "FollowPath.ax_min": -0.90,
            "FollowPath.az_max": 1.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 12.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.90, 0.0, 0.90],
            "min_velocity": [-0.60, 0.0, -0.90],
            "max_accel": [0.90, 0.0, 1.50],
            "max_decel": [-0.90, 0.0, -1.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "strict_safe": {
        "/controller_server": {
            "FollowPath.vx_max": 0.40,
            "FollowPath.time_steps": 135,
            "FollowPath.vx_min": -0.20,
            "FollowPath.wz_max": 0.40,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.20,
            "FollowPath.ax_max": 0.40,
            "FollowPath.ax_min": -0.40,
            "FollowPath.az_max": 1.00,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 12.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.40, 0.0, 0.40],
            "min_velocity": [-0.20, 0.0, -0.40],
            "max_accel": [0.40, 0.0, 1.00],
            "max_decel": [-0.40, 0.0, -1.00],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "strict_slow": {
        "/controller_server": {
            "FollowPath.vx_max": 0.20,
            "FollowPath.time_steps": 200,
            "FollowPath.vx_min": -0.10,
            "FollowPath.wz_max": 0.20,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.20,
            "FollowPath.ax_min": -0.20,
            "FollowPath.az_max": 0.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 12.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.20, 0.0, 0.20],
            "min_velocity": [-0.10, 0.0, -0.20],
            "max_accel": [0.20, 0.0, 0.50],
            "max_decel": [-0.20, 0.0, -0.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
}


NAVIGATION_MODE_INFO = {
    "normal": {"label": "通常", "speed": 0.90, "strict": False},
    "normal_active": {"label": "通常・探索強化", "speed": 1.00, "strict": False},
    "safe": {"label": "安全", "speed": 0.40, "strict": False},
    "slow": {"label": "超低速", "speed": 0.20, "strict": False},
    "wait_normal": {"label": "待機優先", "speed": 0.90, "strict": False},
    "strict_normal": {"label": "パス厳守・通常", "speed": 0.90, "strict": True},
    "strict_safe": {"label": "パス厳守・安全", "speed": 0.40, "strict": True},
    "strict_slow": {"label": "パス厳守・超低速", "speed": 0.20, "strict": True},
}


# Speed-only parameter subset shared by the legacy numeric speed command.
# NAVIGATION_MODE_CONFIGS stays the single source of truth for these values so
# the named modes and the numeric speed command cannot drift apart.
SPEED_CONTROLLER_PARAM_KEYS = (
    "FollowPath.vx_max",
    "FollowPath.vx_min",
    "FollowPath.wz_max",
    "FollowPath.vx_std",
    "FollowPath.wz_std",
    "FollowPath.ax_max",
    "FollowPath.ax_min",
    "FollowPath.az_max",
    "FollowPath.time_steps",
)

# String buckets accepted by the legacy speed command, mapped to named modes.
_SPEED_STRING_MODES = {
    "slow": "slow",
    "safe": "safe",
    "normal": "normal",
    "fast": "normal_active",
}

# Numeric speed upper bounds -> named mode (checked in ascending order).
_SPEED_NUMERIC_BUCKETS = (
    (0.25, "slow"),
    (0.55, "safe"),
    (0.95, "normal"),
)
_SPEED_NUMERIC_DEFAULT_MODE = "normal_active"


def speed_setting_to_mode(speed_setting):
    """Map a legacy numeric/string speed setting to a named navigation mode."""
    if isinstance(speed_setting, str):
        return _SPEED_STRING_MODES.get(speed_setting.strip().lower(), "normal")
    try:
        value = float(speed_setting)
    except (TypeError, ValueError):
        return "normal"
    for threshold, mode in _SPEED_NUMERIC_BUCKETS:
        if value <= threshold:
            return mode
    return _SPEED_NUMERIC_DEFAULT_MODE


def build_speed_parameters(mode):
    """Return only the speed-related params of a named mode (no critics/costmap)."""
    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")
    config = NAVIGATION_MODE_CONFIGS[canonical]
    controller = {
        key: value
        for key, value in config.get("/controller_server", {}).items()
        if key in SPEED_CONTROLLER_PARAM_KEYS
    }
    params = {"/controller_server": controller}
    if "/velocity_smoother" in config:
        params["/velocity_smoother"] = dict(config["/velocity_smoother"])
    return params


def navigation_mode_controller(mode):
    """Return the controller plugin selected by a navigation mode."""
    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")
    return "WaitPath" if canonical == "wait_normal" else "FollowPath"


def normalize_navigation_mode(mode):
    """Return a canonical navigation mode, including the legacy strict alias."""
    normalized = str(mode or "").strip().lower()
    if normalized == "strict":
        normalized = "strict_safe"
    return normalized if normalized in NAVIGATION_MODE_CONFIGS else None


def parse_navigation_mode_command(instruction):
    """Parse a remote nav_mode JSON command; return None for other commands."""
    stripped = str(instruction or "").strip()
    if not (stripped.startswith("{") and stripped.endswith("}")):
        return None
    try:
        payload = json.loads(stripped)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict) or payload.get("type") != "nav_mode":
        return None
    mode = normalize_navigation_mode(payload.get("mode"))
    if mode is None:
        raise ValueError(f"unknown navigation mode: {payload.get('mode')!r}")
    return mode


def navigation_mode_confirmation(mode):
    """Build an explicit Japanese confirmation for an applied mode."""
    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")
    info = NAVIGATION_MODE_INFO[canonical]
    return (
        f"[happy]Nav2走行モードを{info['label']}、"
        f"最高速度{info['speed']:.1f}メートル毎秒に変更したのだ！"
    )
