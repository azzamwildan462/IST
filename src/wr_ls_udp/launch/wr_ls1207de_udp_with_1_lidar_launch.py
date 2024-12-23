import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    wr_node = Node(
        package="wr_ls_udp",
        executable="wr_ls_udp",
        parameters=parameters,
        output='screen'
    )

    ld.add_action(wr_node)

    return ld

parameters = [
    {"frame_id": "laser_link"},
    {"range_min": 0.01},
    {"range_max": 20.0},
    {"hostname": "192.168.0.10"},
    {"port": "2112" },
    {"timelimit": 5},
    {"checkframe": True},
    {"min_ang": -2.357},
    {"max_ang": 2.357},
    {"intensity": 1},
    {"skip": 0},
    {"time_offset": -0.001},
    {"auto_reboot": False},
    {"debug_mode": False},
    {"time_increment": 0.000061722},
    {"angle_resolution": 0.25}
    ]
