import unittest
import rclpy
from pathlib import Path
import sys

sys.path.insert(0, str(Path.cwd()))
sys.path.insert(0, str(Path.cwd().joinpath('src')))
sys.path.insert(0, str(Path.cwd().joinpath('src', 'sirius')))

import importlib.util

# load module
gpe_spec = importlib.util.spec_from_file_location('get_position_enter', str(Path.cwd().joinpath('src', 'sirius', 'sirius_navigation', 'sirius_navigation', 'get_position_enter.py')))
gpe = importlib.util.module_from_spec(gpe_spec)
gpe_spec.loader.exec_module(gpe)

import tf2_py as tf2

class TestGetPositionEnterTfConnectivity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_connectivity_exception_handled(self):
        node = gpe.GetPose()

        # Make lookup_transform raise ConnectivityException
        def raise_connectivity(*args, **kwargs):
            raise tf2.ConnectivityException('test connectivity')

        node.tfBuffer.lookup_transform = raise_connectivity

        try:
            node.get_position()
        except Exception as e:
            node.destroy_node()
            self.fail(f"get_position raised an exception: {e}")
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
