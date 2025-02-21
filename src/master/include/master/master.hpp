#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_interface/msg/point_array.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/help_marker.hpp"
#include "ros2_utils/pid.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#define FSM_GLOBAL_INIT 0
#define FSM_GLOBAL_PREOP 1
#define FSM_GLOBAL_SAFEOP 2
#define FSM_GLOBAL_OP_3 3
#define FSM_GLOBAL_OP_4 4
#define FSM_GLOBAL_OP_5 5
#define FSM_GLOBAL_OP_2 6

#define FSM_LOCAL_PRE_FOLLOW_LANE 0
#define FSM_LOCAL_FOLLOW_LANE 1
#define FSM_LOCAL_MENUNGGU_STATION_1 2
#define FSM_LOCAL_MENUNGGU_STATION_2 3

#define TRANSMISSION_AUTO 0
#define TRANSMISSION_NEUTRAL 1
#define TRANSMISSION_FORWARD 3
#define TRANSMISSION_REVERSE 5

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
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_CAN_eps_encoder;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_beckhoff_sensor;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_obs_find;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kiri;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_tengah;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kanan;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kiri_single_cam;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kanan_single_cam;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_hasil_perhitungan_kiri;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_hasil_perhitungan_kanan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_aruco_kanan_detected;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_aruco_kiri_detected;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_beckhoff;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_lidar;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_cam_kiri;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_cam_kanan;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_pose_estimator;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_obstacle_filter;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_aruco_kiri;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_aruco_kanan;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_error_code_can;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_encoder_meter;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_pressed;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_ui_control_btn;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_ui_control_velocity_and_steering;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_aruco_marker_id_kanan;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_aruco_marker_id_kiri;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_CAN_eps_mode_fb;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_beckhoff_digital_input;

    // Configs
    // ===============================================================================================
    bool use_ekf_odometry = false;
    float profile_max_acceleration = 30;
    float profile_max_decceleration = 60;
    float profile_max_velocity = 2.5; // m/s (1 m/s == 3.6 km/h)
    float profile_max_accelerate_jerk = 100;
    float profile_max_decelerate_jerk = 1000;
    float profile_max_braking = 3;
    float profile_max_braking_acceleration = 2000;
    float profile_max_braking_jerk = 3000;
    float max_obs_find_value = 100;
    float profile_max_steering_rad = 0.785; // Ini adalah arah roda depannya

    // Vars
    // ===============================================================================================
    HelpLogger logger;
    HelpMarker marker;
    MachineState local_fsm;
    MachineState global_fsm;
    PID pid_vx;

    int16_t error_code_beckhoff = 0;
    int16_t error_code_lidar = 0;
    int16_t error_code_cam_kiri = 0;
    int16_t error_code_cam_kanan = 0;
    int16_t error_code_pose_estimator = 0;
    int16_t error_code_obstacle_filter = 0;
    int16_t error_code_aruco_kiri = 0;
    int16_t error_code_aruco_kanan = 0;
    int16_t error_code_can = 0;

    float cam_kiri_pid_output = 0;
    float cam_kiri_pid_setpoint = 0;
    float cam_kiri_pid_fb = 0;
    float cam_kiri_velocity_gain = 0;
    float cam_kanan_pid_output = 0;
    float cam_kanan_pid_setpoint = 0;
    float cam_kanan_pid_fb = 0;
    float cam_kanan_velocity_gain = 0;

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
    ros2_interface::msg::PointArray lane_kiri_single_cam;
    ros2_interface::msg::PointArray lane_kanan_single_cam;

    bool aruco_kiri_detected = false;
    bool aruco_kanan_detected = false;
    int16_t aruco_kiri_marker_id = -1;
    int16_t aruco_kanan_marker_id = -1;

    float fb_encoder_meter = 0;
    float fb_final_pose_xyo[3];
    float fb_final_vel_dxdydo[3];
    float fb_steering_angle = 0;
    uint8_t fb_eps_mode = 0;

    /**
     * Start Stop button bit 0
     * Transmisi R-N-F bit 1-2-3
     */
    uint8_t fb_beckhoff_digital_input[2] = {0};

    float dt = 0.02;

    float obs_find = 0;

    int16_t transmission_joy_master = 0;

    rclcpp::Time current_time;
    rclcpp::Time last_time_CANbus;
    rclcpp::Time last_time_beckhoff;
    rclcpp::Time last_time_lidar;
    rclcpp::Time last_time_cam_kiri;
    rclcpp::Time last_time_cam_kanan;
    rclcpp::Time last_time_pose_estimator;
    rclcpp::Time last_time_obstacle_filter;
    rclcpp::Time last_time_aruco_kiri;
    rclcpp::Time last_time_aruco_kanan;
    rclcpp::Time last_time_joy;
    rclcpp::Time last_time_key_pressed;
    rclcpp::Time last_time_ui_control_btn;
    rclcpp::Time time_start_operation;
    rclcpp::Time time_start_follow_lane;

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
    void callback_sub_hasil_perhitungan_kiri(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_hasil_perhitungan_kanan(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_lane_kiri_single_cam(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_lane_kanan_single_cam(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_aruco_kiri_detected(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_sub_aruco_kanan_detected(const std_msgs::msg::Bool::SharedPtr msg);
    void callback_sub_error_code_beckhoff(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_lidar(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_cam_kiri(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_cam_kanan(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_pose_estimator(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_obstacle_filter(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_aruco_kiri(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_aruco_kanan(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_error_code_can(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_encoder_meter(const std_msgs::msg::Float32::SharedPtr msg);
    void callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_ui_control_btn(const std_msgs::msg::UInt16::SharedPtr msg);
    void callback_sub_ui_control_velocity_and_steering(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void callback_sub_aruco_marker_id_kanan(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_aruco_marker_id_kiri(const std_msgs::msg::Int16::SharedPtr msg);
    void callback_sub_CAN_eps_mode_fb(const std_msgs::msg::UInt8::SharedPtr msg);
    void callback_sub_beckhoff_digital_input(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    // Process
    // ===============================================================================================
    void process_marker();
    void process_local_fsm();
    void process_transmitter();

    // Motion
    // ===============================================================================================
    void manual_motion(float vx, float vy, float wz);
    void follow_lane(float vx, float vy, float wz);
    void follow_lane_2_cam(float vx, float vy, float wz);
    void follow_lane_2_cam_gas_manual(float vx, float vy, float wz);
    void follow_lane_2_cam_steer_manual(float vx, float vy, float wz);
    float obstacle_influence(float gain);

    // Misc
    // ===============================================================================================
    geometry_msgs::msg::Point get_point(double x, double y, double z);
    geometry_msgs::msg::Quaternion get_quat(double r, double p, double y);
    std::vector<geometry_msgs::msg::Point> get_path(float x0, float y0, float x1, float y1, float resolution);
    geometry_msgs::msg::Point get_near(float x, float y, std::vector<geometry_msgs::msg::Point> ps);
    void set_initialpose(float x, float y, float yaw);
    void set_pose_offset(float x, float y, float yaw);
};

#endif // MASTER_HPP