import unittest
import rclpy
from pathlib import Path
import sys
sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path.cwd().joinpath('src')))
sys.path.insert(0, str(Path.cwd().joinpath('src', 'sirius')))

import tf2_py as tf2
import importlib.util
gpd_spec = importlib.util.spec_from_file_location('get_position_distance', str(Path.cwd().joinpath('src', 'sirius', 'sirius_navigation', 'sirius_navigation', 'get_position_distance.py')))
gpd = importlib.util.module_from_spec(gpd_spec)
gpd_spec.loader.exec_module(gpd)
GetPose = gpd.GetPose


class TestTfHandling(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_timer_handles_connectivity_exception(self):
        node = GetPose()

        # Make lookup_transform raise ConnectivityException
        def raise_connectivity(*args, **kwargs):
            raise tf2.ConnectivityException('test connectivity')

        node.tfBuffer.lookup_transform = raise_connectivity

        # Should handle exception and return normally
        try:
            node.timer_callback()
        except Exception as e:
            self.fail(f"timer_callback raised an exception: {e}")
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
