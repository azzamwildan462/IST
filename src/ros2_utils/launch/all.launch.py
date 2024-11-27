import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"

print("ws_path: ", ws_path)
print("path_config: ", path_config)

def generate_launch_description():
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        respawn=True,
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # fmt: off
        arguments=["-d",os.path.join(path_config,"robot.rviz"),
                   "--ros-args","--log-level","error",]
        # fmt: on
        
    )

    # =============================================================================

    master = Node(
        package='master',
        executable='master',
        name='master',
        output='screen',
        respawn=True,
        prefix='nice -n -8'
    )

    pose_estimator = Node(
        package='world_model',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        respawn=True,
        prefix='nice -n -9'
    )

    beckhoff = Node(
        package='hardware',
        executable='beckhoff',
        name='beckhoff',
        output='screen',
        respawn=True,
        prefix='nice -n -10 chrt -f 99'
    )

    # =============================================================================

    tf_base_link_to_body_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_body_link",
        # fmt: off
        arguments=["0.20","0.00","0.35","1.57","0.00","0.00","base_link","body_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_lidar1_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_lidar1_link",
        # fmt: off
        arguments=["0.20","0.00","0.35","1.57","0.00","0.00","base_link","lidar1_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_map_empty = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map_empty",
        # fmt: off
        arguments=["0.00","0.00","0.00","0.00","0.00","0.00","odom","map",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    # =============================================================================

    rtabmap_slam_rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        parameters=[
            {
                "subscribe_depth": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": True,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "scan_cloud_topic": "/fused_pointcloud", # The topic for the fused point cloud
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "odom_tf_linear_variance": 0.01,
                "odom_tf_angular_variance": 0.01,
                "publish_tf": False,
                "approx_sync": True,
                "Grid/CellSize": "0.025",  # Added by Pandu
                "Grid/FootprintHeight": "1.5",  # Added by Pandu
                "Grid/FootprintLength": "0.5",  # Added by Pandu
                "Grid/FootprintWidth": "0.5",  # Added by Pandu
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/Sensor": "0",  # Added to suppress warning
                "Icp/MaxCorrespondenceDistance": "0.1",  # Added from documentation
                "Icp/VoxelSize": "0.05",  # Added from documentation
                "Mem/IncrementalMemory": "False",  # Added by Pandu
                "RGBD/AngularUpdate": "0.01",  # Added from documentation
                "RGBD/LinearUpdate": "0.01",  # Added from documentation
                "RGBD/NeighborLinkRefining": "True",  # Added from documentation
                "RGBD/OptimizeFromGraphEnd": "False",  # Added from documentation
                "RGBD/ProximityBySpace": "True",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "10",  # Added to suppress warning
                "Reg/Force3DoF": "true",  # Added from documentation
                "Reg/Strategy": "1",  # Added from documentation
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("fused_pointcloud", "/multi_lidar_cloud"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        namespace="slam",
        parameters=[
            {
                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "map",
                "two_d_mode": True,
                "smooth_lagged_data": True,
                "history_length": 1.0,
                "frequency": 50.0,
                "odom0": "/odom",
                # fmt: off
                "odom0_config": [
                    True,True,False,
                    False,False,True,
                    False,False,False,
                    False,False,False,
                    False,False,False,
                ],
                # fmt: on

                # "odom0_differential": True,
                # "odom0_relative": True,
                # "pose0": "localization_pose",
                # # fmt: off
                # "pose0_config":[
                #     True,True,False,
                #     False,False,True,
                #     False,False,False,
                #     False,False,False,
                #     False,False,False,
                # ],
                # # fmt: on
                # "pose0_differential": False,
                # "pose0_relative": False,
            }
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    return LaunchDescription(
        [
            pose_estimator,
            tf_base_link_to_body_link,
            tf_base_link_to_lidar1_link,
            tf_map_empty,
            rviz2,
            rosbridge_server, 
            # beckhoff,
            master,
            # rtabmap_slam_rtabmap,
            ekf_node,
        ]
    )
