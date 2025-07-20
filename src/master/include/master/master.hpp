#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_interface/msg/point_array.hpp"
#include "ros2_interface/msg/terminal_array.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/help_marker.hpp"
#include "ros2_utils/pid.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "rtabmap_msgs/msg/info.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vector"
#include "boost/filesystem.hpp"
#include <boost/algorithm/string.hpp>
#include "boost/thread/mutex.hpp"
#include "fstream"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/filters/crop_box.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/transforms.hpp"

#define FSM_GLOBAL_INIT 0
#define FSM_GLOBAL_PREOP 1
#define FSM_GLOBAL_SAFEOP 2
#define FSM_GLOBAL_OP_3 3
#define FSM_GLOBAL_OP_4 4
#define FSM_GLOBAL_OP_5 5
#define FSM_GLOBAL_OP_2 6
#define FSM_GLOBAL_RECORD_ROUTE 7
#define FSM_GLOBAL_MAPPING 8

#define FSM_LOCAL_PRE_FOLLOW_LANE 0
#define FSM_LOCAL_FOLLOW_LANE 1
#define FSM_LOCAL_MENUNGGU_STATION_1 2
#define FSM_LOCAL_MENUNGGU_STATION_2 3
#define FSM_LOCAL_MENUNGGU_STOP 4

#define TRANSMISSION_AUTO 0
#define TRANSMISSION_NEUTRAL 1
#define TRANSMISSION_FORWARD 3
#define TRANSMISSION_REVERSE 5

#define ARUCO_TERMINAL_1 1
#define ARUCO_TERMINAL_2 2
#define ARUCO_TERMINAL_3 3
#define ARUCO_START_BELOKAN 4
#define ARUCO_START_LURUS 5

#define IN_TR_FORWARD 0b01
#define IN_TR_REVERSE 0b10
#define IN_BRAKE_ACTIVE 0b100
#define IN_EPS_nFAULT 0b1000
#define IN_MASK_BUMPER 0b11110000
// #define IN_START_OP3_HANDLER 0b100000000
// #define IN_STOP_OP3_HANDLER 0b1000000000
#define IN_START_OP3_HANDLER 0b1000000000
#define IN_STOP_OP3_HANDLER 0b100000000
// #define IN_START_GAS_MANUAL 0b10000000000
#define IN_TRIM_KECEPATAN 0b10000000000000
#define IN_START_OP3 (0b100000 << 16)
#define IN_STOP_OP3 (0b100 << 16)
#define IN_START_GAS_MANUAL (0b1000 << 16)
#define IN_MANUAL_MUNDUR (0b100 << 16)
#define IN_MANUAL_MAJU (0b10 << 16)
#define IN_NEXT_TERMINAL (0b10000 << 16)
#define IN_SYSTEM_FULL_ENABLE (0b01 << 16)

#define TERMINAL_TYPE_STOP 0x01
#define TERMINAL_TYPE_BELOKAN 0x02
#define TERMINAL_TYPE_STOP1 0x04
#define TERMINAL_TYPE_STOP2 0x08
#define TERMINAL_TYPE_STOP3 0x0C
#define TERMINAL_TYPE_STOP4 0x10
#define TERMINAL_TYPE_LURUS 0x20

#define EMERGENCY_LIDAR_DEPAN_DETECTED 0b010
#define EMERGENCY_CAMERA_OBS_DETECTED 0b100
#define EMERGENCY_GYRO_ANOMALY_DETECTED 0b1000
#define EMERGENCY_ICP_SCORE_TERLALU_BESAR 0b10000
#define EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR 0b100000
#define EMERGENCY_STOP_KARENA_OBSTACLE 0b1000000
#define STATUS_TOWING_CONNECTED 0b01

// using namespace std::chrono_literals;

typedef struct
{
    float x;
    float y;
    float theta;
    float fb_velocity;
    float fb_steering;
    float arah;
} waypoint_t;

typedef struct
{
    float x;
    float y;
} point_t;

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_global_fsm;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_local_fsm;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_to_ui;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_actuator;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_transmission_master;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pose_offset;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_waypoints;
    rclcpp::Publisher<ros2_interface::msg::TerminalArray>::SharedPtr pub_terminals;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_obs_find;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_pose_filtered;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_slam_status;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_camera_obs_emergency;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_master_status_emergency;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_master_status_klik_terminal_terakhir;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_CAN_eps_encoder;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_beckhoff_sensor;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_obs_find;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kiri;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_tengah;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kanan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_icp_pose_estimate;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_beckhoff;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_pose_estimator;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_obstacle_filter;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_can;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_lane_detection;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_aruco_detection;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_encoder_meter;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_pressed;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_ui_control_btn;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_ui_control_velocity_and_steering;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_CAN_eps_mode_fb;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_beckhoff_digital_input;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_aruco_nearest_marker_id;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_depan_scan;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_belakang_scan;
    rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr sub_rtabmap_info;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_localization_pose;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_camera_pcl;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_icp_score;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_detected_forklift_contour;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_detected_forklift;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_gyro_counter;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_record_route_mode;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_terminal; // Aktif -> add terminal, InActive -> save terminal
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_rm_terminal;  // Aktif -> add terminal, InActive -> save terminal

    std::shared_ptr<rclcpp::SyncParametersClient> lidar_obstacle_param_client;

    rclcpp::CallbackGroup::SharedPtr cloud_cb_group_, timer_cb_group_;

    // Configs
    // ===============================================================================================
    bool use_ekf_odometry = false;
    float profile_max_acceleration = 30;
    float profile_max_decceleration = 60;
    float profile_max_velocity = 1.5; // m/s (1 m/s == 3.6 km/h)
    float profile_max_accelerate_jerk = 100;
    float profile_max_decelerate_jerk = 1000;
    float profile_max_braking = 3;
    float profile_max_braking_acceleration = 2000;
    float profile_max_braking_jerk = 3000;
    float max_obs_find_value = 100;
    float profile_max_steering_rad = 0.785; // Ini adalah arah roda depannya
    std::vector<double> pid_terms;
    std::string waypoint_file_path;
    std::string terminal_file_path;
    float wheelbase = 0.75;
    int metode_following = 0;
    bool enable_obs_detection = false;
    bool enable_obs_detection_camera = false;
    float timeout_terminal_1 = 10;
    float timeout_terminal_2 = 10;
    bool transform_map2odom = false;
    float toribay_ready_threshold = 0.5;
    bool use_filtered_pose = false;
    float threshold_icp_score = 100.0;
    bool debug_motion = false;

    std::vector<double> complementary_terms = {0.30, 0.03, 0.01, 0.9};

    float offset_sudut_steering = 0;

    // Vars
    // ===============================================================================================
    HelpLogger logger;
    HelpMarker marker;
    MachineState local_fsm;
    MachineState global_fsm;
    PID pid_vx;

    int16_t error_code_beckhoff = 0;
    int16_t error_code_pose_estimator = 0;
    int16_t error_code_obstacle_filter = 0;
    int16_t error_code_can = 0;
    int16_t error_code_lane_detection = 0;
    int16_t error_code_aruco_detection = 0;

    uint8_t status_perjalanan = 0;

    float actuation_ax = 0;
    float actuation_ay = 0;
    float actuation_az = 0;

    float actuation_vx = 0;
    float actuation_vy = 0;
    float actuation_wz = 0; // Ini posisi

    float target_velocity = 0;

    float target_velocity_joy_x = 0;
    float target_velocity_joy_y = 0;
    float target_velocity_joy_wz = 0;

    ros2_interface::msg::PointArray lane_kiri;
    ros2_interface::msg::PointArray lane_tengah;
    ros2_interface::msg::PointArray lane_kanan;
    int16_t aruco_nearest_marker_id = -1;

    float fb_encoder_meter = 0;
    float fb_final_pose_xyo[3];
    float fb_filtered_final_pose_xyo[3];
    float fb_final_vel_dxdydo[3];
    float fb_steering_angle = 0;
    uint8_t fb_eps_mode = 0;

    uint32_t fb_beckhoff_digital_input = 0;

    float dt = 0.02;

    float obs_find = 0;
    float obs_find_baru = 0;

    int16_t transmission_joy_master = 0;

    rclcpp::Time current_time;
    rclcpp::Time last_time_CANbus;
    rclcpp::Time last_time_beckhoff;
    rclcpp::Time last_time_pose_estimator;
    rclcpp::Time last_time_obstacle_filter;
    rclcpp::Time last_time_joy;
    rclcpp::Time last_time_key_pressed;
    rclcpp::Time last_time_ui_control_btn;
    rclcpp::Time last_time_error_code_aruco_detection;
    rclcpp::Time last_time_error_code_lane_detection;
    rclcpp::Time time_start_operation;
    rclcpp::Time time_start_follow_lane;
    rclcpp::Time last_time_kamera_pcl;

    std::vector<waypoint_t> waypoints;
    ros2_interface::msg::TerminalArray terminals;
    std::vector<point_t> area_special;

    sensor_msgs::msg::LaserScan lidar_depan_points;
    boost::mutex mutex_lidar_depan_points;
    sensor_msgs::msg::LaserScan lidar_belakang_points;
    bool is_rtabmap_ready = false;
    bool is_pose_corrected = false;
    bool prev_is_rtabmap_ready = false;
    float map2odom_offset_x = 0;
    float map2odom_offset_y = 0;
    float map2odom_offset_theta = 0;
    tf2::Transform manual_map2odom_tf;
    bool is_toribay_ready = false;
    float lidar_obs_scan_thr = 1.5;

    geometry_msgs::msg::TransformStamped tf_lidar_base;
    std::unique_ptr<tf2_ros::Buffer> tf_lidar_base_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_lidar_base_listener;

    float camera_scan_min_x_, camera_scan_max_x_, camera_scan_min_y_, camera_scan_max_y_;
    int camera_scan_obs_result = 0;

    uint8_t slam_status = 0;
    bool tf_is_initialized = false;

    float icp_score = 999999;

    int8_t detected_forklift_number = -1;
    int8_t detected_forklift_number_filtered = -1;
    float detected_forklift_contour = 0;

    float dx_icp = 0;
    float dy_icp = 0;
    float dth_icp = 0;
    float icp_mag = 0;

    uint8_t gyro_counter = 0;

    int16_t master_status_emergency = 0;
    int16_t status_klik_terminal_terakhir = -1;

    Master();
    ~Master();

    // ROS
    // ===============================================================================================
    void callback_tim_50hz();
    void callback_sub_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
    void callback_sub_CAN_eps_encoder(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_beckhoff_sensor(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_lane_kiri(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_lane_tengah(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_lane_kanan(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_obs_find(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_sub_icp_pose_estimate(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_sub_error_code_beckhoff(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_pose_estimator(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_obstacle_filter(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_can(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_error_code_lane_detection(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_error_code_aruco_detection(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_encoder_meter(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_ui_control_btn(const std_msgs::msg::UInt16::SharedPtr msg);
    void callback_sub_ui_control_velocity_and_steering(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_aruco_nearest_marker_id(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_CAN_eps_mode_fb(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_sub_beckhoff_digital_input(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void callback_sub_lidar_depan_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void callback_sub_lidar_belakang_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void callback_sub_rtabmap_info(const rtabmap_msgs::msg::Info::SharedPtr msg);
    void callback_sub_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void callback_sub_localization_pose(const geometry_msgs::msg::PoseWithCovarianceStamped msg);
    void callback_sub_camera_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void callback_sub_icp_score(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_detected_forklift_contour(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_detected_forklift(const std_msgs::msg::Int8::SharedPtr msg);
    void callback_sub_gyro_counter(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_srv_set_record_route_mode(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_set_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);
    void callback_srv_rm_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response);

    // Process
    // ===============================================================================================
    void process_marker();
    void process_local_fsm();
    void process_transmitter();
    void process_record_route();
    void process_add_terminal();
    void process_load_waypoints();
    void process_save_waypoints();
    void process_load_terminals();
    void process_save_terminals();

    // Motion
    // ===============================================================================================
    void manual_motion(float vx, float vy, float wz);
    void follow_lane(float vx, float vy, float wz);
    void follow_lane_gas_manual(float vx, float vy, float wz);
    void follow_lane_steer_manual(float vx, float vy, float wz);
    void follow_waypoints(float vx, float vy, float wz, float lookahead_distance, bool is_loop = false);
    void follow_waypoints_gas_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop = false);
    void follow_waypoints_steer_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop = false);
    void fusion_follow_lane_waypoints(float vx, float vy, float wz, float lookahead_distance, bool is_loop = false);
    void fusion_follow_lane_waypoints_gas_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop = false);
    void fusion_follow_lane_waypoints_steer_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop = false);
    void wp2velocity_steering(float lookahead_distance, float *pvelocity, float *psteering, bool is_loop = false);
    void lane2velocity_steering(float *pvelocity, float *psteering, float *pconfidence);
    float pythagoras(float x1, float y1, float x2, float y2);
    float obstacle_influence(float gain);
    float local_obstacle_influence(float obs_scan_r, float gain = 0.5);

    // Misc
    // ===============================================================================================
    geometry_msgs::msg::Point get_point(double x, double y, double z);
    geometry_msgs::msg::Quaternion get_quat(double r, double p, double y);
    std::vector<geometry_msgs::msg::Point> get_path(float x0, float y0, float x1, float y1, float resolution);
    geometry_msgs::msg::Point get_near(float x, float y, std::vector<geometry_msgs::msg::Point> ps);
    void set_initialpose(float x, float y, float yaw);
    void set_pose_offset(float x, float y, float yaw);
    void set_lidar_obstacle_filter_param(double scan_range, double min_y, double max_y, double obstacle_error_tolerance);
};

#endif // MASTER_HPP