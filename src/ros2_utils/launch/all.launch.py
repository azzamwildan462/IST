import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

path_config = os.path.join(get_package_share_directory("ros2_utils"), "configs")

def generate_launch_description():
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        respawn=True,
    )

    master = Node(
        package='master',
        executable='master',
        name='master',
        output='screen',
        respawn=True,
    )

    return LaunchDescription(
        [
            rosbridge_server, 
            master
        ]
    )
