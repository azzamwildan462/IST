import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction

from ament_index_python.packages import get_package_share_directory

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"

livox_config = path_config + "livox_MID360.json"

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

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
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

    livox_lidar_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        namespace='lidar1',
        output='screen',
        parameters=[
            {
                "xfer_format": 0,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "lidar1_link",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": livox_config,
                "cmdline_input_bd_code": "livox0000000001",
                "use_full_angle": False,
                "azimuth_yaw_start": 0.0,
                "azimuth_yaw_end": 3.14,
            }
        ]
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

    obstacle_filter = Node(
        package='world_model',
        executable='obstacle_filter',
        name='obstacle_filter',
        output='screen',
        respawn=True,
        prefix='nice -n -8'
    )

    beckhoff = Node(
        package='hardware',
        executable='beckhoff',
        name='beckhoff',
        output='screen',
        respawn=True,
        prefix='nice -n -10 chrt -f 99'
    )

    vision_capture = Node(
        package='vision',
        executable='vision_capture',
        name='vision_capture',
        output='screen',
        respawn=True,
        prefix='nice -n -8'
    )

    lane_detection = Node(
        package='vision',
        executable='lane_detection',
        name='lane_detection',
        output='screen',
        respawn=True,
        prefix='nice -n -8'
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
        arguments=["0.20","0.00","0.35","0.00","0.00","0.00","base_link","lidar1_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_map_empty = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_map_empty",
        # fmt: off
        arguments=["0.00","0.00","0.00","0.00","0.00","0.00","map","odom",
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
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                # "scan_cloud_topic": "/fused_pointcloud", # The topic for the fused point cloud
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "odom_tf_linear_variance": 0.01,
                "odom_tf_angular_variance": 0.01,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": True,
                # "publish_tf_map_odom": True,
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
            ("scan_cloud", "/lidar1/livox/lidar"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix='nice -n -9',
        respawn=True,
    )

    rtabmap_viz_rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        namespace="slam",
        parameters=[
            {
                "subscribe_depth": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": True,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "approx_sync": True,
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("scan_cloud", "/lidar1/livox/lidar"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=False,
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        namespace="slam",
        parameters=[
            {
                # "use_sim_time": True,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_link_frame": "base_link",
                "world_frame": "map",
                "two_d_mode": True,
                "smooth_lagged_data": True,
                "history_length": 1.0,
                "frequency": 10.0,
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
                "odom0_differential": True,
                "odom0_relative": True,

                "pose0": "localization_pose",
                # fmt: off
                "pose0_config":[
                    True,True,False,
                    False,False,True,
                    False,False,False,
                    False,False,False,
                    False,False,False,
                ],
                # fmt: on
                "pose0_differential": False,
                "pose0_relative": False,
            }
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix='nice -n -9',
        respawn=True,
    )

    return LaunchDescription(
        [
            vision_capture,
            lane_detection,
            pose_estimator,
            obstacle_filter,
            tf_base_link_to_body_link,
            tf_base_link_to_lidar1_link,
            tf_map_empty,
            livox_lidar_driver,
            rviz2,
            rosbridge_server, 
            web_video_server,
            beckhoff,
            master,
            TimerAction(
                period=4.0,
                actions=[
                    rtabmap_slam_rtabmap,
                    # rtabmap_viz_rtabmap_viz,
                    ekf_node,
                ],
            ),
        ]
    )
