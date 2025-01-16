import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction, DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory

# Hokuyo stuff's
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


BECKHOFF_NORMAL = int(3)
BECKHOFF_DELETE_EEPROM = int(2)
BECKHOFF_SCAN_SLAVES = int(1)
BECKHOFF_NO_CONFIG = int(0)

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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    livox_lidar_driver = Node(
        package='livox_ros_driver2_local',
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

    hokuyo_lidar_driver = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name="urg_node2",
        namespace='',
        parameters=[
            {
                "ip_address": "192.168.0.10",
                "frame_id": "lidar2_link",
                "angle_min": -1.57,
                "angle_max": 1.57,
            },
        ],
        output="screen",
        respawn=True,
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hokuyo_lidar_driver,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_lidar_driver),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hokuyo_lidar_driver,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_lidar_driver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    rs2_cam_kiri = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="rs2_cam_kiri",
        namespace="cam_kiri",
        parameters=[
            {
                "camera_name": "cam_kiri",
                "camera_namespace": "cam_kiri",
                "serial_no": "138322251962",
                "enable_accel": True,
                "enable_gyro": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": True,
                "initial_reset": False,
                "depth_module.depth_profile": "640x480x15",
                "rgb_camera.color_profile": "640x480x15",
                "rgb_camera.white_balance": False,
            }
        ],
        remappings=[("/imu", "/imu_raw"), 
                    ("/cam_kiri/rs2_cam_kiri/color/image_raw", "/cam_kiri/image_bgr"),],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        prefix='nice -n -20 chrt -f 96',
    )

    rs2_cam_kanan = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="rs2_cam_kanan",
        namespace="cam_kanan",
        parameters=[
            {
                "camera_name": "cam_kanan",
                "camera_namespace": "cam_kanan",
                "serial_no": "146222253102",
                "enable_accel": True,
                "enable_gyro": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": True,
                "initial_reset": False,
                "depth_module.depth_profile": "640x480x15",
                "rgb_camera.color_profile": "640x480x15",
            }
        ],
        remappings=[("/imu", "/imu_raw"), 
                    ("/cam_kanan/rs2_cam_kanan/color/image_raw", "/cam_kanan/image_bgr"),],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        prefix='nice -n -20 chrt -f 96',
    )

    witty_lidar = Node(
        package="wr_ls_udp",
        executable="wr_ls_udp",
        parameters=[
            {"frame_id": "lidar2_link"},
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
        ],
        output='screen'
    )

    # =============================================================================

    ui_server = Node(
        package="web_ui",
        executable="ui_server.py",
        name="ui_server",
        parameters=[
            {
                "ui_root_path": os.path.join(ws_path,"src/web_ui/src")
            },
        ],
        output="screen",
        respawn=True,
    )

    # =============================================================================

    master = Node(
        package='master',
        executable='master',
        name='master',
        output='screen',
        parameters=[{
            "use_ekf_odometry": False
        }],
        respawn=True,
        prefix='nice -n -8'
    )

    pose_estimator = Node(
        package='world_model',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            "encoder_to_meter" : 0.000013215227,
        }],
        respawn=True,
        prefix='nice -n -9'
    )

    obstacle_filter = Node(
        package='world_model',
        executable='obstacle_filter',
        name='obstacle_filter',
        output='screen',
        parameters=[{
            "lidar_frame_id": "lidar2_link",
            "scan_yaw_start": -1.00, 
            "scan_yaw_end": 1.00,
            "scan_r_max": 2.00,
            "publish_filtered_lidar": True,
            "lidar_topic": "/scan", 
            "use_pointcloud2": False, # Untuk hokuyo atau witty
        }],
        respawn=True,
        prefix='nice -n -8'
    )

    beckhoff = Node(
        package='hardware',
        executable='beckhoff',
        name='beckhoff',
        output='screen',
        parameters=[{
            "if_name": "enx000000000083",
            "po2so_config": BECKHOFF_NO_CONFIG
        }],
        respawn=True,
        prefix='nice -n -20 chrt -f 99'
    )

    CANbus_HAL = Node(
        package='hardware',
        executable='CANbus_HAL',
        name='CANbus_HAL',
        output='screen',
        parameters=[{
            "if_name": "can0",
        }],
        respawn=True,
        prefix='nice -n -20 chrt -f 98'
    )


    imu_serial = Node(
        package="hardware",
        executable="serial_imu",
        name="serial_imu",
        output="screen",
        parameters=[{
            "port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        }],
        respawn=True,
        prefix='nice -n -20 chrt -f 90'
    )

    keyboard_input = Node(
        package='hardware',
        executable='keyboard_input',
        name='keyboard_input',
        output='screen',
        respawn=True,
        prefix=['xterm -e'],
    )

    # vision_capture = Node(
    #     package='vision',
    #     executable='vision_capture',
    #     name='vision_capture',
    #     output='screen',
    #     respawn=True,
    #     prefix='nice -n -8'
    # )

    # lane_detection = Node(
    #     package='vision',
    #     executable='lane_detection',
    #     name='lane_detection',
    #     output='screen',
    #     parameters=[{
    #         "low_h": 27,
    #         "high_h": 255,
    #         "low_l": 153,
    #         "high_l": 255,
    #         "low_s": 0,
    #         "high_s": 255,
    #         "gray_threshold": 200,
    #         "use_dynamic_config": True,
    #         "config_path": os.path.join(path_config,"dynamic_conf.yaml")
    #     }
    #     ],
    #     respawn=True,
    #     prefix='nice -n -8'
    # )
    

    vision_capture_kanan = Node(
        package='vision',
        executable='vision_capture',
        name='vision_capture_kanan',
        output='screen',
        namespace='cam_kanan',
        respawn=True,
        prefix='nice -n -8'
    )

    lane_detection_kanan = Node(
        package='vision',
        executable='single_detection',
        name='lane_detection_kanan',
        output='screen',
        namespace='lane_kanan',
        parameters=[{
            "use_dynamic_config": True,
            "config_path": os.path.join(path_config,"dynamic_conf.yaml"),
            "point_to_velocity_ratio": 0.003,
            "point_to_velocity_angle_threshold": 0.56,
            "metode_perhitungan": 1,
            "setpoint_x": 120,
            "setpoint_y": 240,
            "camera_namespace": "cam_kanan",
            "right_to_left_scan": False,
        }
        ],
        respawn=True,
        prefix='nice -n -8'
    )

    aruco_detection_kanan = Node(
        package='vision',
        executable='single_detection',
        name='aruco_detection_kanan',
        output='screen',
        namespace='aruco_kanan',
        parameters=[{
            "use_dynamic_config": True,
            "config_path": os.path.join(path_config,"dynamic_conf.yaml"),
            "setpoint_x": 320,
            "setpoint_y": 240,
            "detect_aruco": True,
            "use_frame_bgr": True,
            "aruco_dictionary_type": "DICT_4X4_50",
            "min_aruco_range": 100.0,
            "aruco_in_counter_threshold": 30,
            "aruco_out_counter_threshold": 20,
            "camera_namespace": "cam_kanan",
        }
        ],
        respawn=True,
        prefix='nice -n -8'
    )

    vision_capture_kiri = Node(
        package='vision',
        executable='vision_capture',
        name='vision_capture_kiri',
        output='screen',
        namespace='cam_kiri',
        parameters=[{
            "camera_path": "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_70F3B3DF-video-index0"
        }],
        respawn=True,
        prefix='nice -n -8'
    )

    lane_detection_kiri = Node(
        package='vision',
        executable='single_detection',
        name='lane_detection_kiri',
        output='screen',
        namespace='lane_kiri',
        parameters=[{
            "use_dynamic_config": True,
            "config_path": os.path.join(path_config,"dynamic_conf.yaml"),
            "point_to_velocity_ratio": 0.005,
            "point_to_velocity_angle_threshold": 0.6,
            "metode_perhitungan": 1,
            "setpoint_x": 320,
            "setpoint_y": 240,
            "camera_namespace": "cam_kiri",
            "right_to_left_scan": True,
        }
        ],
        respawn=True,
        prefix='nice -n -8'
    )

    aruco_detection_kiri = Node(
        package='vision',
        executable='single_detection',
        name='aruco_detection_kiri',
        output='screen',
        namespace='aruco_kiri',
        parameters=[{
            "use_dynamic_config": True,
            "config_path": os.path.join(path_config,"dynamic_conf.yaml"),
            "setpoint_x": 320,
            "setpoint_y": 240,
            "detect_aruco": True,
            "use_frame_bgr": True,
            "aruco_dictionary_type": "DICT_4X4_50",
            "min_aruco_range": 100.0,
            "aruco_in_counter_threshold": 30,
            "aruco_out_counter_threshold": 20,
            "camera_namespace": "cam_kiri",
        }
        ],
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

    tf_base_link_to_lidar2_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_lidar2_link",
        # fmt: off
        arguments=["0.20","0.00","0.35","0.00","0.00","0.00","base_link","lidar2_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_imu_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_imu_link",
        # fmt: off
        arguments=["0.50","0.00","0.55","0.00","0.00","0.00","base_link","imu_link",
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
            # rs2_cam_kiri,
            # rs2_cam_kanan,
            # vision_capture_kanan,
            # lane_detection_kanan,
            # aruco_detection_kanan,
            # vision_capture_kiri,
            # lane_detection_kiri,
            # aruco_detection_kiri,

            pose_estimator,
            tf_map_empty,
            tf_base_link_to_body_link,
            tf_base_link_to_lidar1_link,
            tf_base_link_to_lidar2_link,
            tf_base_link_to_imu_link,

            # witty_lidar,
            # DeclareLaunchArgument('auto_start', default_value='true'),
            # hokuyo_lidar_driver,
            # urg_node2_node_configure_event_handler,
            # urg_node2_node_activate_event_handler,
            # obstacle_filter,

            # imu_serial,

            # rosbridge_server, 
            # web_video_server,

            # beckhoff,
            # CANbus_HAL,

            master,
            # ui_server,

            # joy_node,
            keyboard_input,



            rviz2,
            # vision_capture,
            # lane_detection,
            # livox_lidar_driver,
            # TimerAction(
            #     period=4.0,
            #     actions=[
            #         rtabmap_slam_rtabmap,
            #         # rtabmap_viz_rtabmap_viz,
            #         ekf_node,
            #     ],
            # ),
        ]
    )
