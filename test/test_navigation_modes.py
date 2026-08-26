"""Tests for direct remote-controller Nav2 mode switching."""

import threading
from unittest.mock import Mock

import pytest

from sirius_navigation.dialogue.llm_dynamic_goal import LlmDynamicGoal
from sirius_navigation.dialogue.modules.nav_controller import NavController
from sirius_navigation.navigation_modes import (
    NAVIGATION_MODE_CONFIGS,
    navigation_mode_confirmation,
    parse_navigation_mode_command,
)


def test_strict_normal_command_keeps_mode_information():
    command = '{"type":"nav_mode","mode":"strict_normal"}'

    assert parse_navigation_mode_command(command) == "strict_normal"
    assert "パス厳守・通常" in navigation_mode_confirmation("strict_normal")
    assert "0.9" in navigation_mode_confirmation("strict_normal")

    config = NAVIGATION_MODE_CONFIGS["strict_normal"]
    assert config["/controller_server"]["FollowPath.PathAlignCritic.cost_weight"] == 60.0
    assert config["/global_costmap/global_costmap"]["obstacle_layer.enabled"] is False


def test_unknown_navigation_mode_is_rejected_before_llm_fallback():
    with pytest.raises(ValueError):
        parse_navigation_mode_command('{"type":"nav_mode","mode":"warp"}')


def test_complete_mode_application_updates_state_only_after_all_services_succeed():
    node = Mock()
    node.lock = threading.Lock()
    controller = NavController(node)
    controller.set_node_parameters = Mock(return_value=True)

    assert controller.set_navigation_mode("strict_normal") is True
    assert node.current_navigation_mode == "strict_normal"
    assert node.current_speed_setting == 0.90
    assert controller.set_node_parameters.call_count == 3
    for call in controller.set_node_parameters.call_args_list:
        assert call.kwargs["wait_for_result"] is True


def test_remote_mode_handler_publishes_confirmed_mode_and_explicit_response():
    node = LlmDynamicGoal.__new__(LlmDynamicGoal)
    node.nav_ctrl = Mock()
    node.nav_ctrl.set_navigation_mode.return_value = True
    node.navigation_mode_pub = Mock()
    node.send_sirius_speak = Mock()
    node.get_logger = Mock(return_value=Mock())

    handled = node.handle_navigation_mode_instruction(
        '{"type":"nav_mode","mode":"strict_normal"}'
    )

    assert handled is True
    node.nav_ctrl.set_navigation_mode.assert_called_once_with("strict_normal")
    status = node.navigation_mode_pub.publish.call_args.args[0]
    assert status.data == "strict_normal"
    spoken = node.send_sirius_speak.call_args.args[0]
    assert "パス厳守・通常" in spoken
    assert "0.9" in spoken
