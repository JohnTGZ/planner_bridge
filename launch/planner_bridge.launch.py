import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    planner_bridge_cfg_filepath = os.path.join(
        get_package_share_directory('planner_bridge'),
        'config',
        'config.yaml')

    with open(planner_bridge_cfg_filepath, 'r') as ymlfile:
        planner_bridge_cfg = yaml.safe_load(ymlfile)

    return LaunchDescription([
        Node(
            package='planner_bridge',
            executable='planner_bridge',
            name='planner_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[planner_bridge_cfg],
        )
    ])