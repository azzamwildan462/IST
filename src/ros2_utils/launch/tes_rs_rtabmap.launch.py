# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True,
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                                  'align_depth.enable': 'true',
                                  'enable_sync': 'true',
                                  'rgb_camera.profile': '640x360x30'}.items(),
        ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
    ])



    # rgbd_odom_node = Node(
    #     package="rtabmap_odom",
    #     executable="rgbd_odometry",
    #     name="rgbd_odometry",
    #     output="screen",
    #     namespace="slam_vo",
    #     parameters=[
    #         {"frame_id": "base_link"},
    #         {'guess_frame_id':'odom'},
    #         {'subscribe_depth':True},
    #         {"subscribe_imu": True},
    #         {"approx_sync": False},
    #         {"Vis/MinInliers": "8"},
    #         {
    #             "use_sim_time": False,
    #             "Odom/Strategy": "1",
    #             "Odom/ResetCountdown": "0",

    #             "Rtabmap/DetectionRate": "25.0", # Added by Azzam
    #             "Rtabmap/CreateIntermediateNodes": "True",
    #             "Rtabmap/LoopThr": "0.11", # Routine period untuk cek loop closure

    #             "Mem/STMSize": "25",  # Short-term memory size
    #             "Mem/IncrementalMemory": "False",  # 
    #             "Mem/InitWMWithAllNodes": "True",  # 
    #             "Mem/RehearsalSimilaritys": "0.9",  #
    #             "Mem/UseOdomFeatures": "True",  #
    #             'Mem/NotLinkedNodesKept': "False",  # Keep unlinked nodes
    #             # "Mem/UseOdomFeatures": "False", # (percobaan) untuk disable odometry untuk mencari loop closure

    #             "Odom/Strategy": "1",  # Added by Azzam
    #             "Odom/ResetCountdown": "0",  # Added by Azzam
    #             "Odom/Holonomic": "False",  # Added by Azzam
    #             "Odom/ScanKeyFrameThr": "0.9",  # Added by Azzam, semakin kecil semakin sering lidar update
    #             "Odom/AlignWithGround": "True",  # Added by Azzam
    #             # "OdomF2M/ScanSubtractAngle": "0.0",  # Added by Azzam
    #             # "OdomF2M/BundleAdjustment": "0",
    #             # "OdomF2M/ScanMaxSize": "20000",

    #             "Reg/Strategy": "0",  # Added by Azzam
    #             "Reg/Force3DoF": "True",  # Added by Azzam

    #             "Icp/Strategy": "1",  # Added by Azzam
    #             "Icp/MaxTranslation": "0.3", # Added by Azzam
    #             "Icp/MaxRotation": "0.1", # Added by Azzam
    #             "Icp/RangeMin": "0.0", # Added by Azzam
    #             "Icp/RangeMax": "2.0", # Added by Azzam
    #             "Icp/MaxCorrespondenceDistance": "1.0", # Added by Azzam
    #             "Icp/Iterations": "30", # Added by Azzam
    #             "Icp/PointToPlane": "True", # Added by Azzam
    #             "Icp/VoxelSize": "0.05", # Added by Azzam
    #             'Icp/PointToPlaneMinComplexity':'0.23', # to be more robust to long corridors with low geometry
    #             'Icp/PointToPlaneLowComplexityStrategy':'2', # to be more robust to long corridors with low geometry

    #             "Vis/MaxDepth": "2.0",
    #             "Vis/MinInliers": "3",

    #             "use_sim_time": False,
    #             "Threads": 10, # Added by Azzam

            
    #         } # Added by Azzam
    #     ],
    #     remappings=[
    #         ("rgb/image", "/camera/rs2_cam_main/color/image_raw"),
    #         ("depth/image", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw"),
    #         ("rgb/camera_info", "/camera/rs2_cam_main/color/camera_info"),
    #         ("imu", "/hardware/imu"),
    #         # ("odom", "vo")
    #     ],
    #     arguments=["--ros-args", "--log-level", "warn"],
    # )