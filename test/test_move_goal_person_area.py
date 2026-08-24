# -*- coding: utf-8 -*-
"""Test person-area waypoint behavior."""

from pathlib import Path
import tempfile
import unittest

from sirius_navigation.move_goal import Nav2GoalClient, Waypoint
import yaml


class _Logger:
    """Provide the logger API used by the isolated behavior tests."""

    def info(self, _message):
        """Accept an info message."""
        pass


class TestMoveGoalPersonArea(unittest.TestCase):
    """Verify loading and navigation behavior for person areas."""

    def test_loads_person_area_from_yaml(self):
        """Load the person_area flag from waypoint YAML."""
        data = {
            'waypoints': [{
                'number': 1,
                'x': 1.0,
                'y': 2.0,
                'angle_radians': 0.0,
                'person_area': True,
            }]
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / 'waypoints.yaml'
            path.write_text(yaml.safe_dump(data), encoding='utf-8')
            client = object.__new__(Nav2GoalClient)
            waypoints = client.load_waypoints(str(path))

        self.assertTrue(waypoints[0].person_area)

    def test_person_area_uses_precise_threshold(self):
        """Use the same precise arrival radius as STOP waypoints."""
        client = object.__new__(Nav2GoalClient)
        client.default_threshold = 2.0
        waypoint = Waypoint(1, 1.0, 2.0, 0.0, person_area=True)

        self.assertEqual(client.get_arrival_threshold(waypoint), 0.5)

    def test_person_area_stops_robot_and_dispatches_next_goal(self):
        """Keep Nav2 active while the external stop signal holds the robot."""
        client = object.__new__(Nav2GoalClient)
        client.count = 0
        client.loop_count = 25
        client.waypoints = [
            Waypoint(1, 1.0, 2.0, 0.0, person_area=True),
            Waypoint(2, 3.0, 4.0, 0.0),
        ]
        client._person_area_stop_active = False
        client._person_area_hold_goal_index = None
        published = []
        client.publish_stop_command = (
            lambda value, index, pause_navigation=True: published.append(
                ('stop', value, index, pause_navigation)
            )
        )
        client.publish_person_area_arrived = (
            lambda value, index: published.append(('person_area', value, index))
        )
        client.send_goal = lambda: published.append(('send_goal',))
        client.get_logger = lambda: _Logger()

        client.hold_at_person_area(0)

        self.assertEqual(
            published,
            [
                ('stop', True, 0, False),
                ('person_area', True, 0),
                ('send_goal',),
            ],
        )
        self.assertEqual(client.count, 1)
        self.assertEqual(client.loop_count, 0)
        self.assertTrue(client._person_area_stop_active)
        self.assertEqual(client._person_area_hold_goal_index, 0)

    def test_resume_clears_arrival_before_sending_next_goal(self):
        """Clear the arrival state before resuming the next goal."""
        client = object.__new__(Nav2GoalClient)
        client._cancelled_by_user = False
        client._paused_by_user = True
        client._person_area_stop_active = True
        client._person_area_hold_goal_index = 0
        client.count = 1
        calls = []
        client.publish_person_area_arrived = (
            lambda value, index: calls.append(('person_area', value, index))
        )
        client.publish_stop_command = (
            lambda value, index: calls.append(('stop', value, index))
        )
        client.send_goal = lambda: calls.append(('send_goal',))
        client.get_logger = lambda: _Logger()

        client.resume_current_goal()

        self.assertEqual(
            calls,
            [('person_area', False, 0), ('stop', False, 1), ('send_goal',)],
        )
        self.assertIsNone(client._person_area_hold_goal_index)
        self.assertFalse(client._person_area_stop_active)

    def test_external_stop_false_clears_person_area_state(self):
        """Use an external /stop false message to finish person detection."""
        client = object.__new__(Nav2GoalClient)
        client._person_area_stop_active = True
        client._person_area_hold_goal_index = 0
        published = []
        client.publish_person_area_arrived = (
            lambda value, index: published.append((value, index))
        )
        client.get_logger = lambda: _Logger()
        message = type('Message', (), {'data': False})()

        client.stop_state_callback(message)

        self.assertEqual(published, [(False, 0)])
        self.assertFalse(client._person_area_stop_active)
        self.assertIsNone(client._person_area_hold_goal_index)


if __name__ == '__main__':
    unittest.main()
