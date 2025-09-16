from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config = get_package_share_directory("rslidar_sdk") + "/rviz/rviz2.rviz"

    return LaunchDescription(
        [
            Node(
                namespace="rslidar_sdk",
                package="rslidar_sdk",
                executable="rslidar_sdk_node_kanan",
                output="screen",
            ),
            Node(
                namespace="rslidar_sdk",
                package="rslidar_sdk",
                executable="rslidar_sdk_node_kiri",
                output="screen",
            ),
            Node(
                namespace="rviz2",
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
