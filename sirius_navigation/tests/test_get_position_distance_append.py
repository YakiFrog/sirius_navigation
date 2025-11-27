import unittest
import rclpy
from pathlib import Path
import sys
import os
import tempfile
import yaml

# Ensure src is available for imports
sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path.cwd().joinpath('src')))
sys.path.insert(0, str(Path.cwd().joinpath('src', 'sirius')))

import importlib.util
gpd_spec = importlib.util.spec_from_file_location('get_position_distance', str(Path.cwd().joinpath('src', 'sirius', 'sirius_navigation', 'sirius_navigation', 'get_position_distance.py')))
gpd = importlib.util.module_from_spec(gpd_spec)
gpd_spec.loader.exec_module(gpd)
GetPose = gpd.GetPose

class TestGetPositionDistanceAppend(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_finish_write_appends_waypoints(self):
        # Create temp YAML with initial waypoint
        fd, tmpfile_path = tempfile.mkstemp(suffix='.yaml')
        os.close(fd)

        initial_data = {
            'format_version': '1.0',
            'waypoints': [
                {
                    'number': 1,
                    'x': -0.5655804534580455,
                    'y': 0.19294177857759992,
                    'angle_radians': 0.07926099311958255
                }
            ]
        }

        with open(tmpfile_path, 'w', encoding='utf-8') as f:
            yaml.dump(initial_data, f, default_flow_style=False)

        # Point module's file_path to temp file
        gpd.file_path = tmpfile_path

        # Instantiate node and prepare positions_list to append
        node = GetPose()
        node.positions_list = [
            {'x': 5.0, 'y': 6.0, 'angle_radians': 0.5},
            {'x': 7.0, 'y': 8.0, 'angle_radians': 1.0}
        ]

        try:
            node.finish_write()
        except Exception as e:
            node.destroy_node()
            self.fail(f"finish_write raised an exception: {e}")
        finally:
            node.destroy_node()

        # Read back YAML and verify new waypoints appended
        with open(tmpfile_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self.assertEqual(len(data['waypoints']), 3)
        self.assertEqual(data['waypoints'][-2]['x'], 5.0)
        self.assertEqual(data['waypoints'][-2]['number'], 2)
        self.assertEqual(data['waypoints'][-1]['x'], 7.0)
        self.assertEqual(data['waypoints'][-1]['number'], 3)

        os.remove(tmpfile_path)


if __name__ == '__main__':
    unittest.main()
