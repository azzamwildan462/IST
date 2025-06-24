import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction, DeclareLaunchArgument


from ament_index_python.packages import get_package_share_directory

path_config_buffer = os.getenv('AMENT_PREFIX_PATH', '')
path_config_buffer_split = path_config_buffer.split(":")
ws_path = path_config_buffer_split[0] + "/../../"
path_config = ws_path + "src/ros2_utils/configs/"

def generate_launch_description():
    
    SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
    SetEnvironmentVariable(name='CYCLONEDDS_URI', value='file://' + path_config + 'cyclonedds.xml'),

    # =============================================================================================

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # fmt: off
        arguments=["-d",os.path.join(path_config,"robot.rviz"),
                   "--ros-args","--log-level","error",]
        # fmt: on
    )

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

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi_node',
        output='screen',
        respawn=True,
    )

    rs2_cam_main = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="rs2_cam_main",
        parameters=[
            {
                "camera_name": "camera",
                "camera_namespace": "",
                "enable_accel": True,
                "enable_gyro": True,
                "enable_depth": True,
                "enable_color": True,
                "enable_sync": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "pointcloud.enable": True,
                "enable_pointcloud": True,
                # "rgb_camera.profile": "640x360x30",
                # "depth_module.profile": "640x360x30",
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
        prefix='nice -n -20',
    )

    wit_ros2_imu = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        parameters=[{'port': '/dev/imu_usb'},
                    {"baud": 115200}],
        remappings=[('/imu/data_raw', '/hardware/imu')],
        output="screen"

    )

    # =============================================================================================

    rtabmap_slam_rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="slam",
        remappings=[
            ('rgb/image', '/camera/rs2_cam_main/color/image_raw'),
            ('rgb/camera_info', '/camera/rs2_cam_main/color/camera_info'),
            ('depth/image', '/camera/rs2_cam_main/aligned_depth_to_color/image_raw')
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    # =============================================================================================

    wifi_control = Node(
        package="communication",
        executable="wifi_control",
        name="wifi_control",
        parameters=[
            {
                "hotspot_ssid": "gh_template",
                "hotspot_password": "gh_template",
            },
        ],
        output="screen",
        respawn=True,
    )

    telemetry = Node(
        package="communication",
        executable="telemetry.py",
        name="telemetry",
        parameters=[{
            "INFLUXDB_URL": "http://172.30.37.21:8086",
            "INFLUXDB_USERNAME": "awm462",
            "INFLUXDB_PASSWORD": "wildan462",
            "INFLUXDB_ORG": "awmawm",
            "INFLUXDB_BUCKET": "ujiCoba",
            "ROBOT_NAME": "gh_template",
        }],
        output="screen",
        respawn=True,
    )

    # =============================================================================================

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

    # =============================================================================================

    master = Node(
        package='master',
        executable='master',
        name='master',
        output='screen',
        respawn=True,
        parameters= [
            {
                "profile_max_acceleration" : 60.0,
                "profile_max_decceleration" : 90.0,
                "profile_max_velocity" : 1.0, 
                "profile_max_accelerate_jerk" : 900.0,
                "profile_max_decelerate_jerk" : 1200.0,
                "profile_max_braking" : 3.0,
                "profile_max_braking_acceleration" : 2000.0,
                "profile_max_braking_jerk" : 3000.0,
                "profile_max_steering_rad" : 0.3, 
                "wheelbase" : 0.27,
                "default_lookahead": 0.32,
            }
        ],
        prefix='nice -n -8'
    )

    # =============================================================================================

    CANbus_HAL = Node(
        package='hardware',
        executable='CANbus_HAL',
        name='CANbus_HAL',
        output='screen',
        parameters=[
            {
                "routine_period_ms": 10,
                "max_can_recv_error_counter": 10,
                "can_timeout_us": 50000,
                "bitrate": 1000000,
            }
        ],
        respawn=True,
        # prefix='nice -n -10',
        # prefix='nice -n -20 chrt -f 98'
    )

    motor_main = Node(
        package='hardware',
        executable='motor_main',
        name='motor_main',
        output='screen',
        respawn=True,
        parameters=[
            {
                "routine_period_ms": 20,
                "k_p_wheel": 3900.0,
                "k_i_wheel": 0.02,
                "wheel_radius": 0.035,
                "encoder_ppr": 147906.25,
                "steering_dutyCycle2rad" : 0.010835,
                "offset_sudut_steering_rad" : 0.70845,
                # "offset_sudut_steering_rad" : 0.1,
                "encoder_to_meter" : 0.000001600758,
            }
        ],
        # prefix='nice -n -10',
    )

    keyboard_input = Node(
        package='hardware',
        executable='keyboard_input',
        name='keyboard_input',
        output='screen',
        respawn=True,
        prefix=['xterm -e'],
    )

    # =============================================================================================

    pose_estimator = Node(
        package='world_model',
        executable='pose_estimator',
        name='pose_estimator',
        output='screen',
        parameters=[{
            "encoder_to_meter" : 0.0000015651508,
        }],
        respawn=True,
        prefix='nice -n -9'
    )

    # =============================================================================================

    vision_capture = Node(
        package='vision',
        executable='vision_capture',
        name='vision_capture',
        output='screen',
        respawn=True,
        parameters=[
            {
                "model_path": "/home/iris/model.onnx",
            }
        ],
        prefix='nice -n -8'
    )

    onnx_inference_node = Node(
        package='vision',
        executable='onnx_inference_node',
        name='onnx_inference_node',
        output='screen',
        respawn=True,
        parameters=[
            {
                "model_path": "/home/iris/model.onnx",
                "use_temporal_model" : 1, 
            }
        ],

        prefix='nice -n -8'
        
    )

    detection = Node(
        package='vision',
        executable='detection',
        name='detection',
        output='screen',
        parameters=[
            {
                "publish_pointcloud2": True,
            }
        ],
        respawn=True,
        prefix='nice -n -8'
    )

    # =============================================================================================

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
                'scan_max_range': 2.0,  # LiDAR max range (meters)
                'scan_voxel_size': 0.05,  # Downsampling resolution (meters)
                "qos_scan": 1,
                "wait_for_transform": 2.0,

                "odom_tf_linear_variance": 0.01,
                "odom_tf_angular_variance": 0.01,
                # "odom_tf_linear_variance": 0.0000000001,
                # "odom_tf_angular_variance": 0.0000000001,
                "publish_tf": False,
                "publish_map": True,
                "approx_sync": False,
                "use_saved_map": False,
                "sync_queue_size": 30,
                "topic_queue_size": 10,

                "Rtabmap/DetectionRate": "25.0", # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11", # Routine period untuk cek loop closure

                "Mem/STMSize": "25",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  # 
                "Mem/InitWMWithAllNodes": "True",  # 
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                'Mem/NotLinkedNodesKept': "False",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure

                "Kp/MaxFeatures": "2000",
                "Kp/MaxDepth": "2.0",
                "Kp/DetectorStrategy": "4",  # 0 = SURF, 1 = SIFT, 2 = ORB

                "RGBD/Enabled": "True",
                "RGBD/OptimizeFromGraphEnd": "False", # True agar robot tidak lompat 
                "RGBD/NeighborLinkRefining": "True",  # Added from documentation
                "RGBD/AngularUpdate": "0.01",  # Added from documentation
                "RGBD/LinearUpdate": "0.01",  # Added from documentation
                "RGBD/OptimizeMaxError": "5.0",  # Added from documentation
                "RGBD/InvertedReg": "False",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "5",
                "RGBD/ProximityMaxGraphDepth": "50",
                "RGBD/ProximityByTime": "False",
                "RGBD/ProximityBySpace": "True",  # Added from documentation
                "RGBD/LoopClosureReextractFeatures": "True",  # Added from documentation
                "RGBD/LocalBundleOnLoopClosure": "True",  # Added from documentation

                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Iterations": "80",  # Added by Azzam
                "Optimizer/Epsilon": "0.00001",  # Added by Azzam
                "Optimizer/Robust": "False",  # Added by Azzam
                'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
                "Optimizer/VarianceIgnored": "True",
                "GTSAM/Incremental": "True",

                "Bayes/PredictionMargin": "0", # Added by Azzam
                "Bayes/FullPredictionUpdate": "False", # Added by Azzam
                "Bayes/PredictionLC": "0.1", # Added by Azzam

                "Odom/Strategy": "1",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                # "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
                # "OdomF2M/BundleAdjustment": "0",
                # "OdomF2M/ScanMaxSize": "20000",

                "Reg/Strategy": "0",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam

                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "0.3", # Added by Azzam
                "Icp/MaxRotation": "0.1", # Added by Azzam
                "Icp/RangeMin": "0.0", # Added by Azzam
                "Icp/RangeMax": "2.0", # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0", # Added by Azzam
                "Icp/Iterations": "30", # Added by Azzam
                "Icp/PointToPlane": "True", # Added by Azzam
                "Icp/VoxelSize": "0.05", # Added by Azzam
                'Icp/PointToPlaneMinComplexity':'0.23', # to be more robust to long corridors with low geometry
                'Icp/PointToPlaneLowComplexityStrategy':'2', # to be more robust to long corridors with low geometry

                "Vis/MaxDepth": "2.0",
                "Vis/MinInliers": "8",

                "Grid/Sensor": "0",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "5.0",  # Added by Azzam
                'Grid/UpdateRate': "1.0",  # Update map every 1 second (default is often higher)
                'Grid/CellSize': "0.01",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
                "GridGlobal/UpdateError": "0.04", # Added by Azzam
                "Grid/RayTracing": "False", # Added by Azzam

                "use_sim_time": False,
                "Threads": 10, # Added by Azzam
        
            }
        ],
        remappings=[
            ("odom", "/slam_vo/odom"),
            ('rgb/image', '/camera/rs2_cam_main/color/image_raw'),
            ('rgb/camera_info', '/camera/rs2_cam_main/color/camera_info'),
            ('depth/image', '/camera/rs2_cam_main/aligned_depth_to_color/image_raw'),
            ('scan', '/detection/pointcloud_laser_scan'),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
        # prefix='nice -n -9 chrt -f 90',
        respawn=True,
    )

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        parameters=[{"use_mag": False}],
        remappings=[
            ("/imu/data_raw", "/camera/rs2_cam_main/imu"),
            ("/imu/data", "/hardware/imu"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
        respawn=True,
    )

    rgbd_odom_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        namespace="slam_vo",
        parameters=[
            {"frame_id": "base_link"},
            {'guess_frame_id':'odom'},
            {'subscribe_depth':True},
            {"subscribe_imu": True},
            {"approx_sync": False},
            {"Vis/MinInliers": "8"},
            {
                "use_sim_time": False,
                "Odom/Strategy": "1",
                "Odom/ResetCountdown": "0",

                "Rtabmap/DetectionRate": "25.0", # Added by Azzam
                "Rtabmap/CreateIntermediateNodes": "True",
                "Rtabmap/LoopThr": "0.11", # Routine period untuk cek loop closure

                "Mem/STMSize": "25",  # Short-term memory size
                "Mem/IncrementalMemory": "False",  # 
                "Mem/InitWMWithAllNodes": "True",  # 
                "Mem/RehearsalSimilaritys": "0.9",  #
                "Mem/UseOdomFeatures": "True",  #
                'Mem/NotLinkedNodesKept': "False",  # Keep unlinked nodes
                # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure

                "Kp/MaxFeatures": "2000",
                "Kp/MaxDepth": "2.0",
                "Kp/DetectorStrategy": "4",  # 0 = SURF, 1 = SIFT, 2 = ORB

                "RGBD/Enabled": "True",
                "RGBD/OptimizeFromGraphEnd": "False", # True agar robot tidak lompat 
                "RGBD/NeighborLinkRefining": "True",  # Added from documentation
                "RGBD/AngularUpdate": "0.01",  # Added from documentation
                "RGBD/LinearUpdate": "0.01",  # Added from documentation
                "RGBD/OptimizeMaxError": "5.0",  # Added from documentation
                "RGBD/InvertedReg": "False",  # Added from documentation
                "RGBD/ProximityPathMaxNeighbors": "5",
                "RGBD/ProximityMaxGraphDepth": "50",
                "RGBD/ProximityByTime": "False",
                "RGBD/ProximityBySpace": "True",  # Added from documentation
                "RGBD/LoopClosureReextractFeatures": "True",  # Added from documentation
                "RGBD/LocalBundleOnLoopClosure": "True",  # Added from documentation

                "Optimizer/Strategy": "2",  # Added by Azzam
                "Optimizer/Iterations": "80",  # Added by Azzam
                "Optimizer/Epsilon": "0.00001",  # Added by Azzam
                "Optimizer/Robust": "False",  # Added by Azzam
                'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
                "Optimizer/VarianceIgnored": "True",
                "GTSAM/Incremental": "True",

                "Bayes/PredictionMargin": "0", # Added by Azzam
                "Bayes/FullPredictionUpdate": "False", # Added by Azzam
                "Bayes/PredictionLC": "0.1", # Added by Azzam

                "Odom/Strategy": "1",  # Added by Azzam
                "Odom/ResetCountdown": "0",  # Added by Azzam
                "Odom/Holonomic": "False",  # Added by Azzam
                "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
                "Odom/AlignWithGround": "True",  # Added by Azzam
                # "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
                # "OdomF2M/BundleAdjustment": "0",
                # "OdomF2M/ScanMaxSize": "20000",

                "Reg/Strategy": "0",  # Added by Azzam
                "Reg/Force3DoF": "True",  # Added by Azzam

                "Icp/Strategy": "1",  # Added by Azzam
                "Icp/MaxTranslation": "0.3", # Added by Azzam
                "Icp/MaxRotation": "0.1", # Added by Azzam
                "Icp/RangeMin": "0.0", # Added by Azzam
                "Icp/RangeMax": "2.0", # Added by Azzam
                "Icp/MaxCorrespondenceDistance": "1.0", # Added by Azzam
                "Icp/Iterations": "30", # Added by Azzam
                "Icp/PointToPlane": "True", # Added by Azzam
                "Icp/VoxelSize": "0.05", # Added by Azzam
                'Icp/PointToPlaneMinComplexity':'0.23', # to be more robust to long corridors with low geometry
                'Icp/PointToPlaneLowComplexityStrategy':'2', # to be more robust to long corridors with low geometry

                "Vis/MaxDepth": "2.0",
                "Vis/MinInliers": "3",

                "Grid/Sensor": "0",  # Added to suppress warning
                "Grid/RangeMin": "0.0",  # Added by Azzam
                "Grid/RangeMax": "100.0",  # Added by Azzam
                'Grid/UpdateRate': "1.0",  # Update map every 1 second (default is often higher)
                'Grid/CellSize': "0.01",  # Increase cell size to reduce map density
                "Grid/FromDepth": "False",  # Added from documentation
                "Grid/IncrementalMapping": "True",  # Added from documentation
                "Grid/Scan2dUnknownSpaceFilled": "False",  # Added by Azzam
                "GridGlobal/UpdateError": "0.04", # Added by Azzam
                "Grid/RayTracing": "False", # Added by Azzam

                "use_sim_time": False,
                "Threads": 10, # Added by Azzam

            
            } # Added by Azzam
        ],
        remappings=[
            ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
            ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
            ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
            ("imu", "/hardware/imu"),
            # ("odom", "vo")
        ],
        arguments=["--ros-args", "--log-level", "warn"],
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
                "frequency": 50.0,
                "odom0": "/odom",
                # fmt: off
                "odom0_config": [
                    True,True,False,
                    False,False,False,
                    False,False,False,
                    False,False,False,
                    False,False,False,
                ],
                # fmt: on
                "odom0_differential": True,
                "odom0_relative": True,

                "odom1": "/slam_vo/odom",
                # fmt: off
                "odom1_config": [
                    True,True,False,
                    False,False,True,
                    False,False,False,
                    False,False,False,
                    False,False,False,
                ],
                # fmt: on
                "odom1_differential": True,
                "odom1_relative": True,

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

    # =============================================================================================

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

    tf_base_link_to_body_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_body_link",
        # fmt: off
        arguments=["0.175","0.00","0.165","0.00","0.00","0.00","base_link","body_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_imu_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_imu_link",
        # fmt: off
        arguments=["0.19","0.00","0.00","0.00","0.00","0.00","base_link","imu_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    tf_base_link_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_link_to_camera_link",
        # fmt: off
        arguments=["0.25","0.00","0.16225","0.00","0.3","0.00","base_link","camera_link",
            "--ros-args","--log-level","error",],
        # fmt: on
        respawn=True,
    )

    # =============================================================================================

    # return LaunchDescription(
    #     [
    #         tf_base_link_to_body_link,
    #         tf_base_link_to_imu_link,
    #         tf_base_link_to_camera_link,

    #         rs2_cam_main,
    #         detection,
    #         rtabmap_slam_rtabmap3,

    #     ]
    # )

    return LaunchDescription(
        [
            rosapi_node, 
            ui_server, 
            rosbridge_server, 
            web_video_server,

            # =============================================================================================

            # tf_map_empty, # sementara
            # pose_estimator,
            tf_base_link_to_body_link,
            tf_base_link_to_imu_link,
            tf_base_link_to_camera_link,

            # =============================================================================================

            # vision_capture,
            detection,  
            onnx_inference_node,

            # =============================================================================================

            # wit_ros2_imu,

            # =============================================================================================

            # keyboard_input,
            master,

            # =============================================================================================

            rs2_cam_main,
            # rgbd_odom_node,
            # imu_filter_madgwick_node,

            # ekf_node,


            # =============================================================================================

            CANbus_HAL,
            motor_main, 

            # =============================================================================================

            # TimerAction(
            #     period=0.5,
            #     actions=[
            #         rtabmap_slam_rtabmap3,
            #         ekf_node,
            #     ],
            # ),

            # =============================================================================================
            rviz2,

        ]
    )
