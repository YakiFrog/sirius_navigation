from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sirius_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kotantu-desktop',
    maintainer_email='Frog7352@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cancel_navigation = sirius_navigation.cancel_navigation:main',
            'assisted_teleop = sirius_navigation.assisted_teleop:main',
            'keyboard_teleop_ja = sirius_navigation.keyboard_teleop_ja:main',
            'keyboard_dynamic_goal = sirius_navigation.keyboard_dynamic_goal:main',
            'llm_dynamic_goal = sirius_navigation.dialogue.llm_dynamic_goal:main',
            'ekf_pose_initializer = sirius_navigation.ekf_pose_initializer:main',
            'move_goal = sirius_navigation.move_goal:main',
            'target_follower = sirius_navigation.target_following.target_follower:main',
            'target_detector = sirius_navigation.target_following.target_detector:main',
            'get_position_distance = sirius_navigation.get_position_distance:main',
            'get_position_enter = sirius_navigation.get_position_enter:main',
            'sam3_ros_bridge = sirius_navigation.sam3_ros_bridge:main',
            'sam3_indexed_map_node = sirius_navigation.sam3_indexed_map_node:main',
            'sam3_colored_map_loader = sirius_navigation.sam3_colored_map_loader:main',
            'sam3_grid_visualizer = sirius_navigation.sam3_grid_visualizer:main',
            'twist_mux_monitor = sirius_navigation.twist_mux_monitor:main',
            'status_monitor = sirius_navigation.dialogue.status_monitor:main',
            'odom_path_publisher = sirius_navigation.odom_path_publisher:main',
        ],
    },
)
