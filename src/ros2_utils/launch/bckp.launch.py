import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable

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
    # SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    # SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        respawn=True,
    )

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
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

    telemetry = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry",
        parameters=[{
            "INFLUXDB_URL": "http://172.30.37.21:8086",
            "INFLUXDB_USERNAME": "awm462",
            "INFLUXDB_PASSWORD": "wildan462",
            "INFLUXDB_ORG": "awmawm",
            "INFLUXDB_BUCKET": "awmawm",
            "ROBOT_NAME": "ist_1", 
        }],
        output="screen",
        respawn=True,
    )

    wit_ros2_imu = Node(
        package="communication",
        executable="wit_ros2_imu.py",
        name="wit_ros2_imu",
        remappings=[('/wit/imu', '/imu/data')],
        # parameters=[{'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'},
        #             {"baud": 9600}],
        output="screen",
        respawn=True,
    )

    master = Node(
        package='master',
        executable='master',
        name='master',
        output='screen',
        parameters=[{
            "use_ekf_odometry": True,
            "offset_sudut_steering": -0.04,
            "waypoint_file_path": os.path.join(path_config,"waypoint.csv"),
            "pid_terms": [0.0070, 0.000000, 0, 0.02, -1.4, 0.4, -0.0005, 0.0005],
            "metode_following": 1,
            "enable_obs_detection": False,
            "timeout_terminal_1": 10.0,
            "timeout_terminal_2": 10.0,
            "wheelbase": 1.0,
        }],
        respawn=True,
        prefix='nice -n -8 chrt -f 95'
    )

    pose_estimator = Node(
        package='world_model',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            # "encoder_to_meter" : 0.000013215227,
            "encoder_to_meter" : 0.000013215227 * 7.1 / 7,
            "offset_sudut_steering": -0.04,
            "gyro_type": 0,
        }],
        respawn=True,
        prefix='nice -n -9 chrt -f 80'
    )

    obstacle_filter = Node(
        package='world_model',
        executable='obstacle_filter',
        name='obstacle_filter',
        output='screen',
        parameters=[{
            "lidar_frame_id": "lidar2_link",
            "scan_yaw_start": -0.5, 
            "scan_yaw_end": 0.5,
            "scan_r_max": 1.5,
            "publish_filtered_lidar": True,
            "lidar_topic": "/scan", 
            "use_pointcloud2": False, # Bernilai False Untuk hokuyo atau witty
            "scan_box_x_min": 0.5,
            "scan_box_y_min": -0.5,
            "scan_box_x_max": 2.5,
            "scan_box_y_max": 0.5,
            "use_scan_box": False,
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
            "if_name": "enp5s0",
            "po2so_config": 0,
            "dac_velocity_maximum": 4.0,
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
            "if_name": "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_203631654D4D-if00",
            "use_socket_can": False,
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
            "port": "/dev/imu_usb",
            "port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            "use_boost": False,
        }],
        respawn=True,
        # prefix='nice -n -20 chrt -f 90'
    )

    keyboard_input = Node(
        package='hardware',
        executable='keyboard_input',
        name='keyboard_input',
        output='screen',
        respawn=True,
        prefix=['xterm -e'],
    )

    vision_capture = Node(
        package='vision',
        executable='vision_capture',
        name='vision_capture',
        output='screen',
        parameters=[{
            # "camera_path": "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c-video-index0",
            "camera_path": "/dev/v4l/by-id/usb-Chicony_Electronics_Co._Ltd._HD_User_Facing_0001-video-index0",
            "hardcoded_image": os.path.join(ws_path,"src/vision/assets/coba_kam.png")
        }],
        respawn=True,
        prefix='nice -n -8'
    )

    lane_detection = Node(
        package='vision',
        executable='lane_detection',
        name='lane_detection',
        output='screen',
        namespace='lane',
        parameters=[{
            "use_dynamic_config": True,
            "config_path": os.path.join(path_config,"dynamic_conf.yaml"),
            "use_frame_bgr": True,
            "absolute_image_topic": "/ascamera_multi/ascamera_multi/rgb0/image",
            # "absolute_image_topic": "/image_bgr",
            "lookahead_distance_y": 15,
            "erode_size": 5,
            "dilate_size": 5,
            "morph_close_size": 5,
            "jarak_minimal": 100,
            "jarak_maksimal": 400,
            "pangkal_x": 320,
            "pangkal_y": 480,
            "batas_x_kiri_scan": 3,
            "batas_x_kanan_scan": 637,
            "minimum_contour_area": 2000,
            "deadzone_kanan_kiri_thr": 0.8,
            # "video_path": os.path.join(ws_path,"src/ros2_utils/video/raw"),
            "video_path": "",
        }
        ],
        respawn=True,
        prefix='nice -n -8'
    )

    aruco_detection = Node(
        package='vision',
        executable='single_detection',
        name='aruco_detection',
        output='screen',
        namespace='aruco_detection',
        parameters=[{
            "use_dynamic_config": True,
            "config_path": os.path.join(path_config,"dynamic_conf.yaml"),
            "setpoint_x": 450,
            "setpoint_y": 240,
            "detect_aruco": True,
            "use_frame_bgr": True,
            "aruco_dictionary_type": "DICT_4X4_50",
            "min_aruco_range": 200.0,
            "aruco_in_counter_threshold": 30,
            "aruco_out_counter_threshold": 20,
            # "absolute_image_topic": "/cam_kanan/image_bgr",
            # "absolute_image_topic": "/image_bgr",
            "absolute_image_topic": "/ascamera_multi/ascamera_multi/rgb0/image",
        }
        ],
        respawn=True,
        prefix='nice -n -8'
    )

    ascamera_kiri = Node(
        namespace= "ascamera_kiri",
        package='ascamera',
        executable='ascamera_node',
        name='ascamera_kiri',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": 3},
            {"usb_path": "2"},
            {"confiPath": os.path.join(ws_path,"src/ascamera/configurationfiles")},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        prefix='nice -n -8 chrt -f 97' 
    )

    ascamera_kanan = Node(
        namespace= "ascamera_kanan",
        package='ascamera',
        executable='ascamera_node',
        name='ascamera_kanan',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": 3},
            {"usb_path": "4"},
            {"confiPath": os.path.join(ws_path,"src/ascamera/configurationfiles")},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        prefix='nice -n -8 chrt -f 97' 
    )

    ascamera_multi = Node(
        namespace= "ascamera_multi",
        package='ascamera',
        executable='ascamera_node',
        name='ascamera_multi',
        respawn=True,
        output='screen',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": os.path.join(ws_path,"src/ascamera/configurationfiles")},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 30},
        ],
        prefix='nice -n -20' 
    )
    

    vision_capture_kanan = Node(
        package='vision',
        executable='vision_capture',
        name='vision_capture_kanan',
        output='screen',
        namespace='cam_kanan',
        parameters=[{
            # "camera_path": "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c-video-index0",
            "camera_path": "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_B72C07EF-video-index0",
            # "hardcoded_image": os.path.join(ws_path,"src/vision/assets/cam_kanan.jpeg")
        }],
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
            "point_to_velocity_ratio": 0.002,
            "point_to_velocity_angle_threshold": 0.3,
            "metode_perhitungan": 3,
            "setpoint_x": 260,
            "setpoint_y": 165,
            "camera_namespace": "cam_kanan",
            "right_to_left_scan": True, # Awal false, pindah kamera aku ganti jadi true
            # "absolute_image_topic": "/cam_kanan/image_bgr",
            "absolute_image_topic": "/ascamera_multi/ascamera_multi/rgb0/image",
            # "absolute_image_topic": "/ascamera_kanan/ascamera_kanan/rgb0/image",
            "erode_size": 3,
            "dilate_size": 5,
            "debug_mode": False, 
            "rotation_angle": 0.75,
            "maximum_error_jarak_setpoint": 600.0,
            "pid_terms": [1.0, 0.000000, 0, 0.02, -0.1, 0.1, -0.0005, 0.0005],
            "pangkal_x": 500,
            "pangkal_y": 300,
            "video_path": os.path.join(ws_path,"src/vision/assets/cam_kanan.mp4"),
        }
        ],
        respawn=True,
        prefix='nice -n -19 chrt -f 94'
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
            # "absolute_image_topic": "/cam_kanan/image_bgr",
            "absolute_image_topic": "/ascamera_multi/ascamera_multi/rgb0/image",
            # "absolute_image_topic": "/ascamera_kanan/ascamera_kanan/rgb0/image",
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
            # "camera_path": "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_B72C07EF-video-index0",
            "camera_path": "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c-video-index0",
            # "hardcoded_image": os.path.join(ws_path,"src/vision/assets/cam_kiri.jpeg")
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
            "point_to_velocity_ratio": 0.002,
            "point_to_velocity_angle_threshold": 0.3,
            "metode_perhitungan": 3,
            "setpoint_x": 495,
            "setpoint_y": 125,
            "camera_namespace": "cam_kiri",
            "right_to_left_scan": False,
            # "absolute_image_topic": "/cam_kiri/image_bgr",
            "absolute_image_topic": "/ascamera_multi/ascamera_multi/rgb1/image",
            # "absolute_image_topic": "/ascamera_kiri/ascamera_kiri/rgb0/image",
            "erode_size": 5,
            "dilate_size": 5,
            "debug_mode": False, 
            "rotation_angle": -0.7,
            "maximum_error_jarak_setpoint": 600.0,
            "pid_terms": [1.0, 0.000000, 0, 0.02, -0.1, 0.1, -0.0005, 0.0005],
            "pangkal_x": 100,
            "pangkal_y": 300,
        }
        ],
        respawn=True,
        prefix='nice -n -19 chrt -f 94' 
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
            # "absolute_image_topic": "/cam_kiri/image_bgr",
            "absolute_image_topic": "/ascamera_multi/ascamera_multi/rgb1/image",
            # "absolute_image_topic": "/ascamera_kiri/ascamera_kiri/rgb0/image",
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
        arguments=["1.378","0.00","0.22","0.00","0.00","3.1415","base_link","lidar2_link",
            "--ros-args","--log-level","error",],
        # arguments=["1.529","0.00","0.35","0.00","0.00","0.00","base_link","lidar2_link",
        #     "--ros-args","--log-level","error",],
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
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
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
                "use_sim_time": False,
                "Rtabmap/DetectionRate": "20.0", # Added by Azzam
                "Threads": 12, # Added by Azzam
            }
        ],
        remappings=[
            # ("scan_cloud", "/scan"),
            ("scan", "/scan"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix='nice -n -9 chrt -f 90',
        respawn=True,
    )

    rtabmap_slam_rtabmap2 = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        parameters=[
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "subscribe_depth": False,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                'scan_max_range': 15.0,  # LiDAR max range (meters)
                'scan_voxel_size': 0.05,  # Downsampling resolution (meters)

                "odom_tf_linear_variance": 0.01,
                "odom_tf_angular_variance": 0.01,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": False,

                "Grid/Sensor": "0",  # Added to suppress warning
                'Grid/UpdateRate': "1.0",  # Update map every 1 second (default is often higher)
                'Grid/CellSize': "0.1",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation

                # Odometry (ICP-based for LiDAR)
                'Reg/Strategy': "1",  # Use ICP-based scan matching
                'Icp/PointToPlane': "True",  # Use point-to-plane ICP
                'Icp/MaxTranslation': "1.5",  # Max allowed movement per frame (meters)
                'Icp/MaxCorrespondenceDistance': "1.3",  # ICP point matching distance
                'Icp/VoxelSize': "0.1",  # ICP downsampling (meters)
                'Icp/Iterations': "100",  # ICP iteration count
                'Icp/CorrespondenceRatio': "0.3",  # Min correspondence ratio

                "Mem/IncrementalMemory": "False",  # Added by Pandu
                "RGBD/AngularUpdate": "0.01",  # Added from documentation
                "RGBD/LinearUpdate": "0.01",  # Added from documentation
                "RGBD/NeighborLinkRefining": "True",  # Added from documentation
                "RGBD/OptimizeFromGraphEnd": "False",  # Added from documentation

                # Loop Closure
                "RGBD/OptimizeMaxError": "0.0",  # Max optimization error
                'RGBD/ProximityPathMaxNeighbors': "10",  # Improve loop closure
                "RGBD/ProximityBySpace": "True",  # Added from documentation
                'Kp/MaxFeatures': "1000",  # Max keypoints for loop closure
                'Mem/RehearsalSimilaritys': "0.3",  # Max keypoints for loop closure
                # 'Mem/STMSize': "30",  # Short-term memory size
                # 'Mem/LaserScanMaxSize': "2000",  # Max scan points stored
                'Mem/NotLinkedNodesKept': "False",  # Keep unlinked nodes

                # Graph Optimization
                'Mem/InitWMWithAllNodes': "True",  # Use all nodes for graph optimization

                "use_sim_time": False,
                # "Rtabmap/DetectionRate": "20.0", # Added by Azzam
                "Threads": 12, # Added by Azzam
                # 'publish_compressed_map': "True",
            }
        ],
        remappings=[
            # ("scan_cloud", "/scan"),
            ("scan", "/scan"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        prefix='nice -n -9 chrt -f 90',
        respawn=True,
    )

    rtabmap_slam_rtabmap3 = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        parameters=[
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "subscribe_depth": False,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                'scan_max_range': 15.0,  # LiDAR max range (meters)
                'scan_voxel_size': 0.05,  # Downsampling resolution (meters)
                "qos_scan": 1,

                "odom_tf_linear_variance": 0.01,
                "odom_tf_angular_variance": 0.01,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": True,

                "Rtabmap/DetectionRate": "20.0", # Added by Azzam

                "Mem/STMSize": "10",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  # 
                "Mem/InitWMWithAllNodes": "True",  # 
                "Mem/RehearsalSimilaritys": "0.8",  #
                "Mem/UseOdomFeatures": "True",  #
                'Mem/NotLinkedNodesKept': "True",  # Keep unlinked nodes

                "Kp/MaxFeatures": "2000",

                "RGBD/Enabled": "True",
                "RGBD/ProximityPathMaxNeighbors": "10",
                "RGBD/OptimizeFromGraphEnd": "True",
                "RGBD/AngularUpdate": "0.0001",  # Added from documentation
                "RGBD/LinearUpdate": "0.0001",  # Added from documentation
                "RGBD/ProximityBySpace": "True",  # Added from documentation
                "RGBD/OptimizeMaxError": "0.0",  # Added from documentation

                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Robust": "True",  # Added by Azzam
                "GTSAM/Incremental": "True",

                "Odom/Strategy": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/AlignWithGround": "True",  # Added by Azzam
                "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
                "OdomF2M/BundleAdjustment": "0",

                "Reg/Strategy": "1",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam

                "Icp/Strategy": "0",  # Added by Azzam
                "Icp/MaxTranslation": "1.0", # Added by Azzam
                "Icp/MaxRotation": "0.9", # Added by Azzam
                "Icp/RangeMin": "0.0", # Added by Azzam
                "Icp/RangeMax": "15.0", # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0", # Added by Azzam
                "Icp/Iterations": "100", # Added by Azzam
                "Icp/PointToPlane": "False", # Added by Azzam

                "Grid/Sensor": "0",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "15.0",  # Added by Azzam
                'Grid/UpdateRate': "1.0",  # Update map every 1 second (default is often higher)
                'Grid/CellSize': "0.1",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "True",  # Added by Azzam
                "GridGlobal/UpdateError": "0.2", # Added by Azzam

                "use_sim_time": False,
                "Threads": 12, # Added by Azzam
            }
        ],
        remappings=[
            # ("scan_cloud", "/scan"),
            ("scan", "/scan"),
            # ("/tf_static", "/tf"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        # prefix='nice -n -9 chrt -f 90',
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
                "approx_sync": False,
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("scan_cloud", "/lidar1/livox/lidar"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=False,
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace="slam",
        output='screen',
        remappings=[('scan', '/scan'),('odom', '/odom')],
        respawn=True,
        arguments=['-configuration_directory', path_config, '-configuration_basename', 'cartographer_config.lua', '-load_state_filename', '/home/ist/test_map/garasi_icar.pbstream', '-load_frozen_state', 'true']
    )

    cartographer_mapping = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace="slam",
        output='screen',
        remappings=[('scan', '/scan'),('odom', '/odom')],
        respawn=True,
        arguments=['-configuration_directory', path_config, '-configuration_basename', 'cartographer_config.lua']
    )

    cartographer_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        namespace="slam",
        output='screen',
        respawn=True,
        arguments=['-resolution', '0.05']
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
                "history_length": 100.0,
                "frequency": 30.0,

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

                "pose0": "localization_pose", # Ini untuk rtabmap
                # "pose0": "tracked_pose", # Ini untuk cartographer
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
            ascamera_multi,

            # ascamera_kanan,
            # ascamera_kiri,
            # rs2_cam_kiri,
            # rs2_cam_kanan,
            # vision_capture_kanan,
            # vision_capture_kiri,

            # lane_detection_kanan,
            # aruco_detection_kanan,
            # lane_detection_kiri,
            # aruco_detection_kiri,


            TimerAction(
                period=8.0,
                actions=[
                    lane_detection,
                    aruco_detection,
                ],
            ),


            # =============================================================================

            pose_estimator,
            # tf_map_empty,
            tf_base_link_to_body_link,
            tf_base_link_to_lidar1_link,
            tf_base_link_to_lidar2_link,
            tf_base_link_to_imu_link,
            
            # =============================================================================

            # witty_lidar,

            TimerAction(
                period=9.0,
                actions=[
                    DeclareLaunchArgument('auto_start', default_value='true'),
                    hokuyo_lidar_driver,
                    urg_node2_node_configure_event_handler,
                    urg_node2_node_activate_event_handler,
                ],
            ),

            # livox_lidar_driver,

            obstacle_filter,

            # =============================================================================

            TimerAction(
                period=1.0,
                actions=[
                    # imu_serial,
                    wit_ros2_imu,
                ],
            ),


            # imu_serial,
            beckhoff,
            CANbus_HAL,

            # =============================================================================

            rosapi_node,
            rosbridge_server, 
            web_video_server,
            master,
            ui_server,

            # =============================================================================

            # joy_node,
            # keyboard_input,

            # =============================================================================

            # telemetry,

            # =============================================================================
            
            # rviz2,
            # vision_capture,
            # TimerAction(
            #     period=4.0,
            #     actions=[
            #         rtabmap_slam_rtabmap,
            #         # rtabmap_viz_rtabmap_viz,
            #         ekf_node,
            #     ],
            # ),


            TimerAction(
                period=9.0,
                actions=[
                    # rtabmap_slam_rtabmap,
                    # rtabmap_slam_rtabmap2,
                    rtabmap_slam_rtabmap3,

                    ekf_node,
                ],
            ),

            # cartographer_node,
            # cartographer_mapping,
            # cartographer_grid_node,

            # rviz2,
        ]
    )
