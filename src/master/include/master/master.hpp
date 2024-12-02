#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/help_marker.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "ros2_utils/pid.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ros2_interface/msg/point_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"

#define FSM_GLOBAL_INIT 0
#define FSM_GLOBAL_PREOP 1
#define FSM_GLOBAL_SAFEOP 2
#define FSM_GLOBAL_OP 3

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_global_fsm;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_to_ui;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kiri;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_tengah;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kanan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;

    // Configs
    // ===============================================================================================
    bool use_ekf_odometry = false;
    float profile_max_acceleration = 100;
    float profile_max_decceleration = 100;
    float profile_max_velocity = 10;
    float profile_max_accelerate_jerk = 1000;
    float profile_max_decelerate_jerk = 1000;
    float profile_max_braking = 80;
    float profile_max_braking_acceleration = 2000;
    float profile_max_braking_jerk = 3000;

    HelpLogger logger;
    HelpMarker marker;
    MachineState local_fsm;
    MachineState global_fsm;
    PID pid_vx;

    float actuation_ax = 0;
    float actuation_ay = 0;
    float actuation_az = 0;

    float actuation_vx = 0;
    float actuation_vy = 0;
    float actuation_wz = 0; // Ini posisi

    ros2_interface::msg::PointArray lane_kiri;
    ros2_interface::msg::PointArray lane_tengah;
    ros2_interface::msg::PointArray lane_kanan;

    float fb_final_pose_xyo[3];
    float fb_final_vel_dxdydo[3];
    float fb_steering_angle = 0;

    float dt = 0.02;

    Master();
    ~Master();

    // ROS
    // ===============================================================================================
    void callback_tim_50hz();
    void callback_sub_lane_kiri(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_lane_tengah(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_lane_kanan(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Process
    // ===============================================================================================
    void process_marker();
    void process_local_fsm();
    void process_transmitter();

    // Motion
    // ===============================================================================================
    void manual_motion(float vx, float vy, float wz);
    void follow_lane(float vx, float vy, float wz);
    float obstacle_influence(float gain);

    // Misc
    // ===============================================================================================
    geometry_msgs::msg::Point get_point(double x, double y, double z);
    geometry_msgs::msg::Quaternion get_quat(double r, double p, double y);
    std::vector<geometry_msgs::msg::Point> get_path(float x0, float y0, float x1, float y1, float resolution);
    geometry_msgs::msg::Point get_near(float x, float y, std::vector<geometry_msgs::msg::Point> ps);
    void set_initialpose(float x, float y, float yaw);
};

#endif // MASTER_HPP