import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition

# Hokuyo stuff's
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

BECKHOFF_NORMAL = int(3)
BECKHOFF_DELETE_EEPROM = int(2)
BECKHOFF_SCAN_SLAVES = int(1)
BECKHOFF_NO_CONFIG = int(0)

path_config_buffer = os.getenv("AMENT_PREFIX_PATH", "")
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"
path_launch = ws_path + "src/ros2_utils/launch/"

livox_config = path_config + "livox_MID360.json"

print("ws_path: ", ws_path)
print("path_config: ", path_config)


def generate_launch_description():
    # SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    # SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    SetEnvironmentVariable("TESSDATA_PREFIX", "/usr/share/tesseract-ocr/4.00"),

    rosbridge_server = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        respawn=True,
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi_node",
        output="screen",
        respawn=True,
    )

    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
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

    ## Untuk config lihat di lidar_kanan.yaml
    bpearl_lidar_kanan = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node_kanan",
        name="bpearl_lidar_kanan",
        namespace="",
        output="screen",
    )

    bpearl_lidar_kiri = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node_kiri",
        name="bpearl_lidar_kiri",
        namespace="",
        output="screen",
    )

    livox_lidar_driver = Node(
        package="livox_ros_driver2_local",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        namespace="lidar1",
        output="screen",
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
        ],
    )

    hokuyo1_lidar_driver = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name="urg_node2_1",
        namespace="",
        parameters=[
            {
                "ip_address": "172.16.32.30",
                "frame_id": "lidar1_link",
                "angle_min": -1.57,
                "angle_max": 1.57,
            },
        ],
        output="screen",
        respawn=True,
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    hokuyo1_lidar_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hokuyo1_lidar_driver,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo1_lidar_driver),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    hokuyo1_lidar_activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hokuyo1_lidar_driver,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo1_lidar_driver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    hokuyo2_lidar_driver = LifecycleNode(
        package="urg_node2",
        executable="urg_node2_node",
        name="urg_node2_2",
        namespace="lidar2",
        parameters=[
            {
                "ip_address": "172.16.32.40",
                "frame_id": "lidar2_link",
                "angle_min": -1.57,
                "angle_max": 1.57,
            },
        ],
        output="screen",
        respawn=True,
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    hokuyo2_lidar_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hokuyo2_lidar_driver,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo2_lidar_driver),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    hokuyo2_lidar_activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hokuyo2_lidar_driver,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo2_lidar_driver),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration("auto_start")),
    )

    rs2_cam_main = Node(
        package="realsense2_camera",
        executable="framos_realsense2_camera_node",
        name="rs2_cam_main",
        parameters=[
            {
                "camera_name": "camera",
                "camera_namespace": "",
                "enable_accel": False,
                "enable_gyro": False,
                "enable_depth": True,
                "enable_color": True,
                "enable_sync": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": True,
                "initial_reset": False,
                # "rgb_camera.color_profile": "640x360x15",
                # "depth_module.depth_profile": "640x360x15",
                # "rgb_camera.color_profile": "848x480x30",
                # # "depth_module.depth_profile": "848x480x30",
                "rgb_camera.color_profile": "1280x720x15",
                # "depth_module.depth_profile": "848x480x30",
                "rgb_camera.inter_packet_delay": "15",
                "depth_module.inter_packet_delay": "15",
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        output="screen",
        ##        prefix='nice -n -14 chrt -f 60',
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
        remappings=[
            ("/imu", "/imu_raw"),
            ("/cam_kiri/rs2_cam_kiri/color/image_raw", "/cam_kiri/image_bgr"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        ##        prefix='nice -n -20 chrt -f 86',
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
        remappings=[
            ("/imu", "/imu_raw"),
            ("/cam_kanan/rs2_cam_kanan/color/image_raw", "/cam_kanan/image_bgr"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        ##        prefix='nice -n -20 chrt -f 86',
    )

    witty_lidar = Node(
        package="wr_ls_udp",
        executable="wr_ls_udp",
        parameters=[
            {"frame_id": "lidar2_link"},
            {"range_min": 0.01},
            {"range_max": 20.0},
            {"hostname": "192.168.0.10"},
            {"port": "2112"},
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
            {"angle_resolution": 0.25},
        ],
        output="screen",
    )

    # =============================================================================

    ui_server = Node(
        package="web_ui",
        executable="ui_server.py",
        name="ui_server",
        parameters=[
            {"ui_root_path": os.path.join(ws_path, "src/web_ui/src")},
        ],
        output="screen",
        respawn=True,
    )

    upload_server = Node(
        package="web_ui",
        executable="upload_server.py",
        name="upload_server",
        parameters=[
            {"config_dir": path_config},
            {"launch_dir": path_launch}
        ],
        output="screen",
        respawn=True,
    )

    # =============================================================================

    telemetry = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry",
        parameters=[
            {
                "INFLUXDB_URL": "http://172.30.37.21:8086",
                "INFLUXDB_USERNAME": "awm462",
                "INFLUXDB_PASSWORD": "wildan462",
                "INFLUXDB_ORG": "awmawm",
                "INFLUXDB_BUCKET": "awmawm",
                "ROBOT_NAME": "ist_1",
            }
        ],
        output="screen",
        respawn=True,
    )

    telemetry_icar = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry_icar",
        parameters=[
            {
                "INFLUXDB_URL": "http://172.30.37.21:8086",
                "INFLUXDB_USERNAME": "awm462",
                "INFLUXDB_PASSWORD": "wildan462",
                "INFLUXDB_ORG": "awmawm",
                "INFLUXDB_BUCKET": "awmawm",
                "ROBOT_NAME": "icar_omoda",
            }
        ],
        output="screen",
        respawn=True,
    )

    wit_ros2_imu = Node(
        package="communication",
        executable="wit_ros2_imu.py",
        name="wit_ros2_imu",
        # remappings=[('/wit/imu', '/imu/data')],
        # parameters=[{'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'},
        #             {"baud": 9600}],
        output="screen",
        respawn=True,
    )

    master = Node(
        package="master",
        executable="master",
        name="master",
        output="screen",
        parameters=[
            {
                "use_ekf_odometry": True,
                "offset_sudut_steering": -0.04,
                "waypoint_file_path": os.path.join(path_config, "waypoint.csv"),
                "terminal_file_path": os.path.join(path_config, "terminal.csv"),
                "pid_terms": [0.0070, 0.000000, 0, 0.02, -1.4, 0.4, -0.0005, 0.0005],
                "metode_following": 0,
                "enable_obs_detection": True,
                "enable_obs_detection_camera": False,
                "timeout_terminal_1": 10.0,
                "timeout_terminal_2": 10.0,
                "wheelbase": 0.9,
                "profile_max_velocity": 1.0,
                "complementary_terms": [0.60, 0.03, 0.01, 0.94],
                "use_filtered_pose": False,
                "camera_scan_min_x_": 0.2,
                "camera_scan_max_x_": 3.0,
                "camera_scan_min_y_": -1.0,
                "camera_scan_max_y_": 1.0,
                "threshold_icp_score": 0.7,
                "debug_motion": True,
                "all_obstacle_thr": 50000.0,
            }
        ],
        respawn=True,
        ##        prefix='nice -n -8 chrt -f 70'
    )

    all_obstacle_filter = Node(
        package="world_model",
        executable="all_obstacle_filter",
        name="all_obstacle_filter",
        output="screen",
        parameters=[
            {
                "scan_box_x_min": 0.0,
                "scan_box_x_max": 1.5,
                "scan_box_y_min": -0.5,
                "scan_box_y_max": 0.5,
                "scan_box_z_min": 0.3,
                "scan_box_z_max": 1.0,
                "lidar_kanan_topic": "/lidar_kanan_points",
                "lidar_kiri_topic": "/lidar_kiri_points",
                "camera_topic": "/camera/rs2_cam_main/depth/color/points",
                "lidar_kanan_frame_id": "lidar_kanan_link",
                "lidar_kiri_frame_id": "lidar_kiri_link",
                "camera_frame_id": "camera_depth_optical_frame",
                "scan_toribay_x_min": -2.0,
                "scan_toribay_x_max": -0.3,
                "scan_toribay_y_min": -0.7,
                "scan_toribay_y_max": -0.7,
                "scan_toribay_z_min": 0.3,
                "scan_toribay_z_max": 1.0,
                "gain_lidar": 70.0,
            }
        ],
        respawn=True,
        # remappings=[('/hardware/imu', '/can/imu')],
        # prefix='nice -n -9 chrt -f 60'
    )

    camera_obstacle_detector = Node(
        package="world_model",
        executable="camera_obstacle_detector",
        name="camera_obstacle_detector",
        output="screen",
        parameters=[
            {
                "roi_min_x": 0.0,
                "roi_max_x": 3.0,
                "roi_min_y": -1.0,
                "roi_max_y": 1.0,
            }
        ],
        respawn=True,
        # remappings=[('/hardware/imu', '/can/imu')],
        # prefix='nice -n -9 chrt -f 60'
    )

    lidar_obstacle_filter = Node(
        package="world_model",
        executable="lidar_obstacle_filter",
        name="lidar_obstacle_filter",
        output="screen",
        parameters=[
            {
                ## Untuk 1 jalur
                "scan_min_y": -1.0,
                "scan_max_y": 1.0,
                "scan_range": 3.5,
                "obstacle_error_tolerance": 0.4,
                "pgm_path": "/home/ist/map_baru_bagus.pgm",
                "yaml_path": "/home/ist/map_baru_bagus.yaml",
                "resolution": 0.3,
                "origin_x": -211.59,
                "origin_y": -22.645,
                "icp_max_range": 8.0,
                ## Untuk 2 Jalur
                # "scan_min_y": -1.0,
                # "scan_max_y": 1.0,
                # "scan_range": 3.5,
                # "obstacle_error_tolerance": 0.4,
                # "pgm_path": "/home/ist/lagi_map_baru.pgm",
                # "yaml_path": "/home/ist/lagi_map_baru.yaml",
                # "resolution": 0.3,
                # "origin_x": -218.581,
                # "origin_y": -21.8307,
            }
        ],
        respawn=True,
        # remappings=[('/hardware/imu', '/can/imu')],
        # prefix='nice -n -9 chrt -f 60'
    )

    pose_estimator = Node(
        package="world_model",
        executable="pose_estimator",
        name="pose_estimator",
        output="screen",
        parameters=[
            {
                # "encoder_to_meter" : 0.000013215227,
                # "encoder_to_meter" : 0.000013215227 * 7.1 / 7,
                # "encoder_to_meter" : 0.0000123550127, # sama navis
                # "encoder_to_meter" : 0.0000279741933,
                # "encoder_to_meter" : 0.0000282567609, # Hari pertama sebelum di IST
                "encoder_to_meter": 0.0000279741933,  # Hari pertama sebelum di IST
                "offset_sudut_steering": -0.04,
                "gyro_type": 0,
                "timer_period": 40,
            }
        ],
        respawn=True,
        # remappings=[('/hardware/imu', '/can/imu')],
        ##        prefix='nice -n -9 chrt -f 60'
    )

    # pose_estimator_icp = Node(
    #     package='world_model',
    #     executable='pose_estimator_icp',
    #     name='pose_estimator_icp',
    #     namespace="manual_icp",
    #     output='screen',
    #     parameters=[{
    #         "encoder_to_meter" : 0.0000279741933, # Hari pertama sebelum di IST
    #         "offset_sudut_steering": -0.04,
    #         "gyro_type": 0,
    #         "timer_period": 40,
    #         "lidar_frame_id": "lidar2_link",
    #         "lidar_topic": "/scan",
    #         "icp_minimal_travel": 0.5,
    #         "icp_max_correspondence_distance": 0.1,
    #         "icp_max_translation_distance": 0.1,
    #         "icp_max_linear_correction": 0.5,
    #         "icp_max_angular_correction": 0.5,
    #         "icp_gain": 0.005,
    #         "map_file": os.path.join(ws_path, "tools/rtabmap3.pcd"),
    #         "icp_max_iterations": 50,
    #         "icp_transformation_epsilon": 1e-8,
    #         "threshold_icp_score": 20.0,
    #     }],
    #     respawn=True,
    #     # remappings=[('/hardware/imu', '/can/imu')],
    #     prefix='nice -n -9 chrt -f 60'
    # )

    obstacle_filter = Node(
        package="world_model",
        executable="obstacle_filter",
        name="obstacle_filter",
        output="screen",
        parameters=[
            {
                "lidar_frame_id": "lidar2_link",
                "scan_yaw_start": -0.5,
                "scan_yaw_end": 0.5,
                "scan_r_max": 1.5,
                "publish_filtered_lidar": True,
                "lidar_topic": "/scan",
                "use_pointcloud2": False,  # Bernilai False Untuk hokuyo atau witty
                "scan_box_x_min": 0.5,
                "scan_box_y_min": -0.5,
                "scan_box_x_max": 2.5,
                "scan_box_y_max": 0.5,
                "use_scan_box": False,
            }
        ],
        respawn=True,
        ##        prefix='nice -n -8'
    )

    beckhoff = Node(
        package="hardware",
        executable="beckhoff",
        name="beckhoff",
        output="screen",
        parameters=[
            {
                # "if_name": "enp45s0",
                "if_name": "enp5s0",
                # "if_name": "enxf8e43b8f7f88",
                "po2so_config": 0,
                "dac_velocity_maximum": 4.0,
                "brake_idle_position": -50000,
            }
        ],
        respawn=True,
        # prefix='nice -n -20 chrt -f 89'
    )

    CANbus_HAL = Node(
        package="hardware",
        executable="CANbus_HAL",
        name="CANbus_HAL",
        output="screen",
        parameters=[
            {
                "if_name": "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_203631654D4D-if00",
                "use_socket_can": False,
                "counter_divider_can_send": 4,
            }
        ],
        remappings=[("/can/imu", "/hardware/imu")],
        respawn=True,
        ##        prefix='nice -n -19 chrt -f 88'
    )

    CANbus_HAL_socket_can0 = Node(
        package="hardware",
        executable="CANbus_HAL",
        name="CANbus_HAL_socket_can0",
        output="screen",
        parameters=[
            {
                "if_name": "can0",
                "bitrate": 125000,
                "use_socket_can": True,
                "can_to_car": False,
            }
        ],
        respawn=True,
        remappings=[("/can/imu", "/hardware/imu")],
        ##        prefix='nice -n -18 chrt -f 88'
    )
    CANbus_HAL_socket_can1 = Node(
        package="hardware",
        executable="CANbus_HAL",
        name="CANbus_HAL_socket_can1",
        output="screen",
        parameters=[
            {
                "if_name": "can1",
                "bitrate": 125000,
                "use_socket_can": True,
                "can_to_car": True,
            }
        ],
        respawn=True,
        ##        prefix='nice -n -17 chrt -f 87'
    )

    imu_serial = Node(
        package="hardware",
        executable="serial_imu",
        name="serial_imu",
        output="screen",
        parameters=[
            {
                "port": "/dev/imu_usb",
                "port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
                "use_boost": False,
                "is_riontech": True,
                "baudrate": 115200,
                "port": "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10KUSZ2-if00-port0",
            }
        ],
        respawn=True,
        # prefix='nice -n -20 chrt -f 80'
    )

    keyboard_input = Node(
        package="hardware",
        executable="keyboard_input",
        name="keyboard_input",
        output="screen",
        respawn=True,
        prefix=["xterm -e"],
    )

    vision_capture = Node(
        package="vision",
        executable="vision_capture",
        name="vision_capture",
        output="screen",
        parameters=[
            {
                # "camera_path": "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c-video-index0",
                "camera_path": "/dev/v4l/by-id/usb-Chicony_Electronics_Co._Ltd._HD_User_Facing_0001-video-index0",
                "hardcoded_image": os.path.join(
                    ws_path, "src/vision/assets/coba_kam.png"
                ),
            }
        ],
        respawn=True,
        ##        prefix='nice -n -8'
    )

    forklift_detector = Node(
        package="vision",
        executable="forklift_detector",
        name="forklift_detector",
        output="screen",
        respawn=True,
        ##        prefix='nice -n -8'
    )

    lane_detection = Node(
        package="vision",
        executable="lane_detection",
        name="lane_detection",
        output="screen",
        namespace="lane",
        parameters=[
            {
                "use_dynamic_config": True,
                "config_path": os.path.join(path_config, "dynamic_conf.yaml"),
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
        ##        prefix='nice -n -8'
    )

    aruco_detection = Node(
        package="vision",
        executable="single_detection",
        name="aruco_detection",
        output="screen",
        namespace="aruco_detection",
        parameters=[
            {
                "use_dynamic_config": True,
                "config_path": os.path.join(path_config, "dynamic_conf.yaml"),
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
        ##        prefix='nice -n -8'
    )

    ascamera_kiri = Node(
        namespace="ascamera_kiri",
        package="ascamera",
        executable="ascamera_node",
        name="ascamera_kiri",
        respawn=True,
        output="both",
        parameters=[
            {"usb_bus_no": 3},
            {"usb_path": "2"},
            {"confiPath": os.path.join(ws_path, "src/ascamera/configurationfiles")},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        ##        prefix='nice -n -8 chrt -f 87'
    )

    ascamera_kanan = Node(
        namespace="ascamera_kanan",
        package="ascamera",
        executable="ascamera_node",
        name="ascamera_kanan",
        respawn=True,
        output="both",
        parameters=[
            {"usb_bus_no": 3},
            {"usb_path": "4"},
            {"confiPath": os.path.join(ws_path, "src/ascamera/configurationfiles")},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        ##        prefix='nice -n -8 chrt -f 87'
    )

    ascamera_multi = Node(
        namespace="ascamera_multi",
        package="ascamera",
        executable="ascamera_node",
        name="ascamera_multi",
        respawn=True,
        output="screen",
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": os.path.join(ws_path, "src/ascamera/configurationfiles")},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 30},
        ],
        ##        prefix='nice -n -20'
    )

    vision_capture_kanan = Node(
        package="vision",
        executable="vision_capture",
        name="vision_capture_kanan",
        output="screen",
        namespace="cam_kanan",
        parameters=[
            {
                # "camera_path": "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c-video-index0",
                "camera_path": "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_B72C07EF-video-index0",
                # "hardcoded_image": os.path.join(ws_path,"src/vision/assets/cam_kanan.jpeg")
            }
        ],
        respawn=True,
        ##        prefix='nice -n -8'
    )

    lane_detection_kanan = Node(
        package="vision",
        executable="single_detection",
        name="lane_detection_kanan",
        output="screen",
        namespace="lane_kanan",
        parameters=[
            {
                "use_dynamic_config": True,
                "config_path": os.path.join(path_config, "dynamic_conf.yaml"),
                "point_to_velocity_ratio": 0.002,
                "point_to_velocity_angle_threshold": 0.3,
                "metode_perhitungan": 3,
                "setpoint_x": 260,
                "setpoint_y": 165,
                "camera_namespace": "cam_kanan",
                "right_to_left_scan": True,  # Awal false, pindah kamera aku ganti jadi true
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
                "video_path": os.path.join(ws_path, "src/vision/assets/cam_kanan.mp4"),
            }
        ],
        respawn=True,
        ##        prefix='nice -n -19 chrt -f 84'
    )

    aruco_detection_kanan = Node(
        package="vision",
        executable="single_detection",
        name="aruco_detection_kanan",
        output="screen",
        namespace="aruco_kanan",
        parameters=[
            {
                "use_dynamic_config": True,
                "config_path": os.path.join(path_config, "dynamic_conf.yaml"),
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
        ##        prefix='nice -n -8'
    )

    vision_capture_kiri = Node(
        package="vision",
        executable="vision_capture",
        name="vision_capture_kiri",
        output="screen",
        namespace="cam_kiri",
        parameters=[
            {
                # "camera_path": "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_B72C07EF-video-index0",
                "camera_path": "/dev/v4l/by-id/usb-046d_罗技高清网络摄像机_C930c-video-index0",
                # "hardcoded_image": os.path.join(ws_path,"src/vision/assets/cam_kiri.jpeg")
            }
        ],
        respawn=True,
        ##        prefix='nice -n -8'
    )

    lane_detection_kiri = Node(
        package="vision",
        executable="single_detection",
        name="lane_detection_kiri",
        output="screen",
        namespace="lane_kiri",
        parameters=[
            {
                "use_dynamic_config": True,
                "config_path": os.path.join(path_config, "dynamic_conf.yaml"),
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
        ##        prefix='nice -n -19 chrt -f 84'
    )

    aruco_detection_kiri = Node(
        package="vision",
        executable="single_detection",
        name="aruco_detection_kiri",
        output="screen",
        namespace="aruco_kiri",
        parameters=[
            {
                "use_dynamic_config": True,
                "config_path": os.path.join(path_config, "dynamic_conf.yaml"),
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
        ##        prefix='nice -n -8'
    )

    forklift_detector_vision = Node(
        package="vision",
        executable="single_detection",
        name="forklift_detector_vision",
        output="screen",
        namespace="forklift_detector_vision",
        parameters=[
            {
                "use_dynamic_config": True,
                "config_path": os.path.join(path_config, "dynamic_conf.yaml"),
                "point_to_velocity_ratio": 0.002,
                "point_to_velocity_angle_threshold": 0.3,
                "metode_perhitungan": 3,
                "setpoint_x": 495,
                "setpoint_y": 125,
                # "camera_namespace": "cam_kiri",
                "right_to_left_scan": False,
                # "absolute_image_topic": "/cam_kiri/image_bgr",
                "absolute_image_topic": "/camera/rs2_cam_main/color/image_raw",
                # "absolute_image_topic": "/ascamera_kiri/ascamera_kiri/rgb0/image",
                "debug_mode": False,
                "rotation_angle": -0.7,
                "maximum_error_jarak_setpoint": 600.0,
                "pid_terms": [1.0, 0.000000, 0, 0.02, -0.1, 0.1, -0.0005, 0.0005],
                "pangkal_x": 100,
                "pangkal_y": 300,
                "erode_size": 9,
                "dilate_size": 9,
                "is_detect_forklift": True,
                "detect_aruco": False,
                "threshold_forklift_px_size": 1000,
                "roi_detect_forklift": [100.0, 100.0, 1100.0, 700.0],
            }
        ],
        respawn=True,
        ##        prefix='nice -n -19 chrt -f 84'
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

    tf_base_link_to_lidar_kanan_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_lidar_kanan_link",
        # fmt: off
        arguments=["1.22","-0.45","0.800","0.00","0.00","1.57","base_link","lidar_kanan_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_lidar_kiri_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_lidar_kiri_link",
        # fmt: off
        arguments=["1.22","0.45","0.800","0.00","0.00","-1.57","base_link","lidar_kiri_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_lidar1_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_lidar1_link",
        # fmt: off
        arguments=["1.378","0.00","0.22","0.00","0.00","3.1415","base_link","lidar1_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        # # fmt: off
        # arguments=["1.378","0.00","0.22","0.00","0.00","3.1415","base_link","lidar1_link",
        #     "--ros-args","--log-level","error",],
        # # fmt: on
        respawn=True,
    )

    tf_base_link_to_lidar2_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_lidar2_link",
        # fmt: off
        arguments=["-0.2","0.00","0.8","0.00","0.00","0.00","base_link","lidar2_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_imu_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_imu_link",
        # fmt: off
        arguments=["0.00","0.00","0.00","0.00","0.00","0.00","base_link","imu_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_camera_link",
        # fmt: off
        arguments=["1.428","0.00","0.97","0.00","0.12","0.00","base_link","camera_link",
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
                "subscribe_depth": True,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                "scan_max_range": 50.0,  # LiDAR max range (meters)
                "scan_voxel_size": 0.05,  # Downsampling resolution (meters)
                "qos_scan": 1,
                "wait_for_transform": 2.0,
                "odom_tf_linear_variance": 0.01,
                "odom_tf_angular_variance": 0.01,
                # "odom_tf_linear_variance": 0.0001,
                # "odom_tf_angular_variance": 0.0001,
                # "odom_tf_linear_variance": 0.0000000001,
                # "odom_tf_angular_variance": 0.0000000001,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": True,
                "use_saved_map": False,
                "sync_queue_size": 30,
                "topic_queue_size": 10,
                "Rtabmap/DetectionRate": "5.0",  # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11",  # Routine period untuk cek loop closure
                "Mem/STMSize": "50",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  #
                "Mem/InitWMWithAllNodes": "True",  #
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                "Mem/NotLinkedNodesKept": "True",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure
                "Kp/MaxFeatures": "2000",
                "RGBD/Enabled": "True",
                "RGBD/OptimizeFromGraphEnd": "False",  # True agar robot tidak lompat
                "RGBD/NeighborLinkRefining": "True",  # Added from documentation
                "RGBD/AngularUpdate": "0.01",  # Added from documentation
                "RGBD/LinearUpdate": "0.01",  # Added from documentation
                "RGBD/OptimizeMaxError": "3.0",  # Added from documentation
                "RGBD/InvertedReg": "False",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "20",
                "RGBD/ProximityMaxGraphDepth": "50",
                "RGBD/ProximityByTime": "False",
                "RGBD/ProximityBySpace": "True",  # Added from documentation,
                "RGBD/LocalBundleOnLoopClosure": "False",
                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Iterations": "70",  # Added by Azzam
                "Optimizer/Epsilon": "0.00001",  # Added by Azzam
                "Optimizer/Robust": "False",  # Added by Azzam
                "Optimizer/GravitySigma": "0.3",
                "Optimizer/VarianceIgnored": "False",
                "Optimizer/LandmarksIgnored": "False",  # Added by Azzam
                "Optimizer/PriorsIgnored": "True",  # Added by Azzam
                "GTSAM/Incremental": "False",  # Added by Azzam
                "Bayes/PredictionMargin": "0",  # Added by Azzam
                "Bayes/FullPredictionUpdate": "False",  # Added by Azzam
                "Bayes/PredictionLC": "0.1",  # Added by Azzam
                "Odom/Strategy": "1",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                "Odom/FilteringStrategy": "1",
                # "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
                # "OdomF2M/BundleAdjustment": "0",
                # "OdomF2M/ScanMaxSize": "20000",
                "Reg/Strategy": "1",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam
                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "1.5",  # Added by Azzam
                "Icp/MaxRotation": "0.7",  # Added by Azzam
                "Icp/RangeMin": "0.0",  # Added by Azzam
                "Icp/RangeMax": "25.0",  # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0",  # Added by Azzam
                "Icp/Iterations": "30",  # Added by Azzam
                "Icp/PointToPlane": "True",  # Added by Azzam
                "Icp/VoxelSize": "0.05",  # Added by Azzam
                "Icp/PointToPlaneMinComplexity": "0.19",  # to be more robust to long corridors with low geometry
                "Icp/PointToPlaneLowComplexityStrategy": "1",  # to be more robust to long corridors with low geometry
                "Vis/MaxDepth": "20.0",
                "Vis/MinInliers": "15",
                "Grid/Sensor": "0",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "150.0",  # Added by Azzam
                "Grid/UpdateRate": "1.0",  # Update map every 1 second (default is often higher)
                "Grid/CellSize": "0.3",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
                "GridGlobal/UpdateError": "0.04",  # Added by Azzam
                "Grid/RayTracing": "False",  # Added by Azzam
                "use_sim_time": False,
                "Threads": 10,  # Added by Azzam
            }
        ],
        remappings=[
            ("scan", "/scan"),
            ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        # prefix='nice -n -9 chrt -f 80',
        respawn=True,
    )

    rtabmap_slam_robust = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        parameters=[
            {
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_scan": True,
                "subscribe_scan_cloud": False,
                "subscribe_stereo": False,
                "subscribe_rgbd": False,
                "subscribe_rgb": False,
                "subscribe_Odometry": True,
                "scan_max_range": 50.0,  # LiDAR max range (meters)
                "scan_voxel_size": 0.05,  # Downsampling resolution (meters)
                "qos_scan": 1,
                "wait_for_transform": 2.0,
                "odom_tf_linear_variance": 0.0001,
                "odom_tf_angular_variance": 0.0001,
                # "odom_tf_linear_variance": 0.0000000001,
                # "odom_tf_angular_variance": 0.0000000001,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": True,
                "use_saved_map": False,
                "sync_queue_size": 30,
                "topic_queue_size": 10,
                "icp_odometry": False,
                "visual_odometry": False,
                "Rtabmap/DetectionRate": "5.0",  # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11",  # Routine period untuk cek loop closure
                "Mem/STMSize": "20",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  #
                "Mem/InitWMWithAllNodes": "True",  #
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                "Mem/NotLinkedNodesKept": "True",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure
                "Kp/MaxFeatures": "2000",
                "RGBD/Enabled": "True",
                "RGBD/OptimizeFromGraphEnd": "False",  # True agar robot tidak lompat
                "RGBD/NeighborLinkRefining": "False",  # Added from documentation
                "RGBD/AngularUpdate": "0.1",  # Added from documentation
                "RGBD/LinearUpdate": "0.1",  # Added from documentation
                "RGBD/OptimizeMaxError": "3.0",  # Added from documentation
                "RGBD/InvertedReg": "False",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "5",
                "RGBD/ProximityMaxGraphDepth": "20",
                "RGBD/ProximityByTime": "False",
                "RGBD/ProximityBySpace": "True",
                # "RGBD/LocalBundleOnLoopClosure": "True",
                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Iterations": "10",  # Added by Azzam
                "Optimizer/Epsilon": "0.00001",  # Added by Azzam
                "Optimizer/Robust": "False",  # Added by Azzam
                "Optimizer/GravitySigma": "0.3",
                "Optimizer/VarianceIgnored": "False",
                "Optimizer/LandmarksIgnored": "False",  # Added by Azzam
                "Optimizer/PriorsIgnored": "True",  # Added by Azzam
                "GTSAM/Incremental": "False",  # Added by Azzam
                "Bayes/PredictionMargin": "0",  # Added by Azzam
                "Bayes/FullPredictionUpdate": "False",  # Added by Azzam
                "Bayes/PredictionLC": "0.1",  # Added by Azzam
                "Odom/Strategy": "0",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                "Odom/FilteringStrategy": "1",
                # "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
                # "OdomF2M/BundleAdjustment": "0",
                # "OdomF2M/ScanMaxSize": "20000",
                "Reg/Strategy": "1",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam
                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "1.5",  # Added by Azzam
                "Icp/MaxRotation": "0.7",  # Added by Azzam
                "Icp/RangeMin": "0.0",  # Added by Azzam
                "Icp/RangeMax": "25.0",  # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0",  # Added by Azzam
                "Icp/Iterations": "30",  # Added by Azzam
                "Icp/PointToPlane": "True",  # Added by Azzam
                "Icp/VoxelSize": "0.05",  # Added by Azzam
                "Icp/PointToPlaneMinComplexity": "0.09",  # to be more robust to long corridors with low geometry
                "Icp/PointToPlaneLowComplexityStrategy": "1",  # to be more robust to long corridors with low geometry
                "Vis/MaxDepth": "20.0",
                "Vis/MinInliers": "15",
                "Grid/Sensor": "0",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "150.0",  # Added by Azzam
                "Grid/UpdateRate": "1.0",  # Update map every 1 second (default is often higher)
                "Grid/CellSize": "0.3",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
                "GridGlobal/UpdateError": "0.04",  # Added by Azzam
                "Grid/RayTracing": "False",  # Added by Azzam
                "use_sim_time": False,
                # "Threads": 12, # Added by Azzam,
                # "database_path": "/root/.ros/rtabmap.db", # Untuk 2 jalur
                "database_path": "/home/ist/map_baru_bagus.db", # Untuk 1 jalur
            }
        ],
        remappings=[
            ("scan", "/scan"),
            ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        # prefix='nice -n -9 chrt -f 80',
        respawn=True,
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        namespace="slam",
        output="screen",
        remappings=[("scan", "/scan"), ("odom", "/odom")],
        respawn=True,
        arguments=[
            "-configuration_directory",
            path_config,
            "-configuration_basename",
            "cartographer_config.lua",
            "-load_state_filename",
            "/home/ist/test_map/garasi_icar.pbstream",
            "-load_frozen_state",
            "true",
        ],
    )

    cartographer_mapping = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        namespace="slam",
        output="screen",
        remappings=[("scan", "/scan"), ("odom", "/odom")],
        respawn=True,
        arguments=[
            "-configuration_directory",
            path_config,
            "-configuration_basename",
            "cartographer_config.lua",
        ],
    )

    cartographer_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node",
        namespace="slam",
        output="screen",
        respawn=True,
        arguments=["-resolution", "0.05"],
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
                "history_length": 50.0,
                "frequency": 50.0,
                "publish_tf": False,
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
                # "odom0_noise_covariance":    [0.0004, 0.0,    0.0,
                #                             0.0,    0.0004, 0.0,
                #                             0.0,    0.0,    0.05],
                "pose0": "localization_pose",  # Ini untuk rtabmap
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
                # "pose0_noise_covariance":    [0.25,   0.0,    0.0,
                #                             0.0,    0.25,   0.0,
                #                             0.0,    0.0,    0.25],
                # Process noise = how fast the filter can change
                # "process_noise_covariance":  [5e-5,   0.0,    0.0,
                #                             0.0,    5e-5,   0.0,
                #                             0.0,    0.0,    0.0001],
                # "odom1": "/slam_vo/odom",
                # # fmt: off
                # "odom1_config": [
                #     True,True,False,
                #     False,False,True,
                #     False,False,False,
                #     False,False,False,
                #     False,False,False,
                # ],
                # # fmt: on
                # "odom1_differential": True,
                # "odom1_relative": True,
                # "odom2": "/slam_icp/odom",
                # # fmt: off
                # "odom2_config": [
                #     True,True,False,
                #     False,False,True,
                #     False,False,False,
                #     False,False,False,
                #     False,False,False,
                # ],
                # # fmt: on
                # "odom2_differential": True,
                # "odom2_relative": True,
            }
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        ##        prefix='nice -n -15 chrt -f 69',
        respawn=True,
    )

    rgbd_odom_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        namespace="slam_vo",
        parameters=[
            {
                "frame_id": "base_link",
                "subscribe_depth": True,
                "subscribe_imu": True,
                "use_sim_time": False,
                "publish_tf": False,
                "approx_sync": False,
                "Rtabmap/DetectionRate": "25.0",  # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11",  # Routine period untuk cek loop closure
                "Mem/STMSize": "25",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  #
                "Mem/InitWMWithAllNodes": "True",  #
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                "Mem/NotLinkedNodesKept": "False",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure
                "Odom/Strategy": "1",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                "Reg/Strategy": "0",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam
                "Kp/MaxFeatures": "2000",
                "Kp/MaxDepth": "8.0",
                "Kp/DetectorStrategy": "4",  # 0 = SURF, 1 = SIFT, 2 = ORB
                "Vis/MaxDepth": "8.0",
                "Vis/MinInliers": "10",
                "use_sim_time": False,
                "Threads": 10,  # Added by Azzam
            }  # Added by Azzam
        ],
        remappings=[
            ("scan", "/scan"),
            ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
            ("imu", "/hardware/imu"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    icp_odom_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        name="icp_odometry",
        output="screen",
        namespace="slam_icp",
        parameters=[
            {
                "frame_id": "base_link",
                "subscribe_scan": True,
                "subscribe_depth": False,
                "subscribe_imu": True,
                "use_sim_time": False,
                "publish_tf": False,
                "approx_sync": False,
                "Rtabmap/DetectionRate": "25.0",  # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11",  # Routine period untuk cek loop closure
                "Mem/STMSize": "25",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  #
                "Mem/InitWMWithAllNodes": "True",  #
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                "Mem/NotLinkedNodesKept": "False",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure
                "Odom/Strategy": "1",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.3",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "1.0",  # Added by Azzam
                "Icp/MaxRotation": "0.5",  # Added by Azzam
                "Icp/RangeMin": "0.0",  # Added by Azzam
                "Icp/RangeMax": "25.0",  # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0",  # Added by Azzam
                "Icp/Iterations": "30",  # Added by Azzam
                "Icp/PointToPlane": "False",  # Added by Azzam, False karena butuh cepat
                "Icp/VoxelSize": "0.05",  # Added by Azzam
                "Reg/Strategy": "1",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam
                "use_sim_time": False,
                "Threads": 10,  # Added by Azzam
            }  # Added by Azzam
        ],
        remappings=[
            ("scan", "/scan"),
            ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
            ("imu", "/hardware/imu"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    test_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "yaml_filename": "/home/wildan/Desktop/map_baru/map_baru_bagus.yaml",
            }
        ],
        remappings=[("map", "/slam/map")],
    )

    test_map_server_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
        remappings=[("map", "/slam/map")],
    )

    # return LaunchDescription(
    #     [
    #         # DeclareLaunchArgument('auto_start', default_value='true'),
    #         # hokuyo1_lidar_driver,
    #         # hokuyo1_lidar_configure,
    #         # hokuyo1_lidar_activate,
    #         # hokuyo2_lidar_driver,
    #         # hokuyo2_lidar_configure,
    #         # hokuyo2_lidar_activate,


    #         # beckhoff,
    #         # CANbus_HAL,

    #         # tf_base_link_to_lidar2_link,
    #         # tf_base_link_to_body_link,
    #         # tf_base_link_to_lidar1_link,
    #         # tf_base_link_to_imu_link,
    #         # tf_base_link_to_lidar_kanan_link,
    #         # tf_base_link_to_lidar_kiri_link,
    #         # tf_base_link_to_camera_link,
    #         # bpearl_lidar_kanan,
    #         # bpearl_lidar_kiri,
    #         rs2_cam_main,


    #         # rviz2,
    #         # CANbus_HAL_socket_can0,
    #         # CANbus_HAL_socket_can1,
    #         # rosapi_node,
    #         # rosbridge_server,
    #         # web_video_server,
    #         # master,
    #         # ui_server,
    #         # upload_server,
    #         # test_map_server,
    #         # test_map_server_lifecycle
    #         # all_obstacle_filter,
    #         # camera_obstacle_detector,
    #         # lidar_obstacle_filter,
    #         # forklift_detector

    #         # rtabmap_slam_robust,
    #     ]
    # )

    # ==============================================================================

    return LaunchDescription(
        [
            # ascamera_multi,
            # TimerAction(
            #     period=8.0,
            #     actions=[
            #         lane_detection,
            #         aruco_detection,
            #     ],
            # ),
            # =============================================================================
            pose_estimator,
            tf_base_link_to_lidar_kanan_link,
            tf_base_link_to_lidar_kiri_link,
            tf_base_link_to_lidar2_link,
            tf_base_link_to_body_link,
            tf_base_link_to_imu_link,
            tf_base_link_to_lidar1_link,
            tf_base_link_to_camera_link,
            # # =============================================================================
            rs2_cam_main,
            # # =============================================================================
            TimerAction(
                period=8.0,
                actions=[
                    bpearl_lidar_kanan,
                    bpearl_lidar_kiri,
                ],
            ),
            TimerAction(
                period=1.5,
                actions=[
                    DeclareLaunchArgument("auto_start", default_value="true"),
                    hokuyo1_lidar_driver,
                    hokuyo1_lidar_activate,
                    hokuyo1_lidar_configure,
                ],
            ),

            # obstacle_filter,
            # camera_obstacle_detector,
            lidar_obstacle_filter,
            all_obstacle_filter,
            # forklift_detector,
            # forklift_detector_vision,
            # # =============================================================================
            # TimerAction(
            #     period=1.0,
            #     actions=[
            #         # wit_ros2_imu,
            #         # imu_serial,
            #     ],
            # ),
            beckhoff,
            # CANbus_HAL,
            CANbus_HAL_socket_can0,  # Untuk PC tanpa embedded CAN
            CANbus_HAL_socket_can1,  # Untuk PC tanpa embedded CAN
            # # =============================================================================
            rosapi_node,
            rosbridge_server,
            web_video_server,
            master,
            ui_server,
            upload_server,
            # # =============================================================================
            TimerAction(
                period=0.5,
                actions=[
                    # rtabmap_slam_rtabmap3,
                    rtabmap_slam_robust,
                    # icp_odom_node,
                    # rgbd_odom_node,
                    ekf_node,
                ],
            ),
            # TimerAction(
            #     period=300.0,
            #     actions=[
            #         ekf_node,
            #     ],
            # ),
            # rviz2,
        ]
    )
