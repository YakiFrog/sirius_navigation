import unittest
import rclpy
from pathlib import Path
import sys
import os
import tempfile
import yaml
from geometry_msgs.msg import TransformStamped

# Ensure src is available for imports
sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path.cwd().joinpath('src')))
sys.path.insert(0, str(Path.cwd().joinpath('src', 'sirius')))

import importlib.util
gpe_spec = importlib.util.spec_from_file_location('get_position_enter', str(Path.cwd().joinpath('src', 'sirius', 'sirius_navigation', 'sirius_navigation', 'get_position_enter.py')))
gpe = importlib.util.module_from_spec(gpe_spec)
gpe_spec.loader.exec_module(gpe)


class TestGetPositionEnterOverwriteLast(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_append_waypoint(self):
        # Create temporary YAML with initial waypoint
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
        gpe.file_path = tmpfile_path

        # Instantiate node
        node = gpe.GetPose()

        # Build a transform stub
        ts = TransformStamped()
        ts.transform.translation.x = 1.0
        ts.transform.translation.y = 2.0
        ts.transform.translation.z = 0.0
        ts.transform.rotation.x = 0.0
        ts.transform.rotation.y = 0.0
        ts.transform.rotation.z = 0.0
        ts.transform.rotation.w = 1.0

        # Replace lookup_transform to return our transform
        node.tfBuffer.lookup_transform = lambda *args, **kwargs: ts

        # Replace sys.exit so it does not kill the test process
        import sys as _sys
        original_exit = _sys.exit
        _sys.exit = lambda *args, **kwargs: (_ for _ in ()).throw(SystemExit)

        try:
            with self.assertRaises(SystemExit):
                node.get_position()
        finally:
            _sys.exit = original_exit
            node.destroy_node()

        # Read back YAML and verify a new waypoint is appended and numbered incrementally
        with open(tmpfile_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        self.assertEqual(len(data['waypoints']), 2)
        last_wp = data['waypoints'][-1]
        self.assertEqual(last_wp['x'], 1.0)
        self.assertEqual(last_wp['y'], 2.0)
        self.assertEqual(last_wp['number'], 2)

        os.remove(tmpfile_path)


if __name__ == '__main__':
    unittest.main()
