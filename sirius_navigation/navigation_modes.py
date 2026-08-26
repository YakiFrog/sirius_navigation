"""Shared Nav2 navigation mode definitions and remote-command helpers."""

import json


NAVIGATION_MODE_CONFIGS = {
    "normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.90,
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
    "slow": {
        "/controller_server": {
            "FollowPath.vx_max": 0.20,
            "FollowPath.vx_min": -0.10,
            "FollowPath.wz_max": 0.20,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.20,
            "FollowPath.ax_max": 0.20,
            "FollowPath.ax_min": -0.20,
            "FollowPath.az_max": 0.50,
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
            "max_velocity": [0.20, 0.0, 0.20],
            "min_velocity": [-0.10, 0.0, -0.20],
            "max_accel": [0.20, 0.0, 0.50],
            "max_decel": [-0.20, 0.0, -0.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "strict_normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.90,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 0.90,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.90,
            "FollowPath.ax_min": -0.90,
            "FollowPath.az_max": 1.50,
            "FollowPath.CostCritic.cost_weight": 15.0,
            "FollowPath.PathAlignCritic.cost_weight": 60.0,
            "FollowPath.PathFollowCritic.cost_weight": 3.0,
            "FollowPath.PathAngleCritic.cost_weight": 1.5,
            "FollowPath.TwirlingCritic.cost_weight": 5.0,
            "FollowPath.PreferForwardCritic.cost_weight": 25.0,
            "FollowPath.GoalCritic.cost_weight": 0.5,
            "FollowPath.GoalAngleCritic.cost_weight": 0.5,
        },
        "/velocity_smoother": {
            "max_velocity": [0.90, 0.0, 0.90],
            "min_velocity": [-0.60, 0.0, -0.90],
            "max_accel": [0.90, 0.0, 1.50],
            "max_decel": [-0.90, 0.0, -1.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": False},
    },
    "strict_safe": {
        "/controller_server": {
            "FollowPath.vx_max": 0.40,
            "FollowPath.vx_min": -0.20,
            "FollowPath.wz_max": 0.40,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.20,
            "FollowPath.ax_max": 0.40,
            "FollowPath.ax_min": -0.40,
            "FollowPath.az_max": 1.00,
            "FollowPath.CostCritic.cost_weight": 15.0,
            "FollowPath.PathAlignCritic.cost_weight": 60.0,
            "FollowPath.PathFollowCritic.cost_weight": 3.0,
            "FollowPath.PathAngleCritic.cost_weight": 1.5,
            "FollowPath.TwirlingCritic.cost_weight": 5.0,
            "FollowPath.PreferForwardCritic.cost_weight": 25.0,
            "FollowPath.GoalCritic.cost_weight": 0.5,
            "FollowPath.GoalAngleCritic.cost_weight": 0.5,
        },
        "/velocity_smoother": {
            "max_velocity": [0.40, 0.0, 0.40],
            "min_velocity": [-0.20, 0.0, -0.40],
            "max_accel": [0.40, 0.0, 1.00],
            "max_decel": [-0.40, 0.0, -1.00],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": False},
    },
    "strict_slow": {
        "/controller_server": {
            "FollowPath.vx_max": 0.20,
            "FollowPath.vx_min": -0.10,
            "FollowPath.wz_max": 0.20,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.20,
            "FollowPath.ax_max": 0.20,
            "FollowPath.ax_min": -0.20,
            "FollowPath.az_max": 0.50,
            "FollowPath.CostCritic.cost_weight": 20.0,
            "FollowPath.PathAlignCritic.cost_weight": 60.0,
            "FollowPath.PathFollowCritic.cost_weight": 3.0,
            "FollowPath.PathAngleCritic.cost_weight": 1.5,
            "FollowPath.TwirlingCritic.cost_weight": 5.0,
            "FollowPath.PreferForwardCritic.cost_weight": 25.0,
            "FollowPath.GoalCritic.cost_weight": 0.5,
            "FollowPath.GoalAngleCritic.cost_weight": 0.5,
        },
        "/velocity_smoother": {
            "max_velocity": [0.20, 0.0, 0.20],
            "min_velocity": [-0.10, 0.0, -0.20],
            "max_accel": [0.20, 0.0, 0.50],
            "max_decel": [-0.20, 0.0, -0.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": False},
    },
}


NAVIGATION_MODE_INFO = {
    "normal": {"label": "通常", "speed": 0.90, "strict": False},
    "normal_active": {"label": "通常・探索強化", "speed": 1.00, "strict": False},
    "safe": {"label": "安全", "speed": 0.40, "strict": False},
    "slow": {"label": "超低速", "speed": 0.20, "strict": False},
    "strict_normal": {"label": "パス厳守・通常", "speed": 0.90, "strict": True},
    "strict_safe": {"label": "パス厳守・安全", "speed": 0.40, "strict": True},
    "strict_slow": {"label": "パス厳守・超低速", "speed": 0.20, "strict": True},
}


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
