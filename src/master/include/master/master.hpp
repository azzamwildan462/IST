#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/help_marker.hpp"
#include "ros2_utils/simple_fsm.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ros2_interface/msg/point_array.hpp"

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kiri;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_tengah;
    rclcpp::Subscription<ros2_interface::msg::PointArray>::SharedPtr sub_lane_kanan;

    HelpLogger logger;
    HelpMarker marker;
    MachineState fsm;

    float actuation_vx = 0;
    float actuation_vy = 0;
    float actuation_wz = 0;

    ros2_interface::msg::PointArray lane_kiri;
    ros2_interface::msg::PointArray lane_tengah;
    ros2_interface::msg::PointArray lane_kanan;

    Master();
    ~Master();

    // ROS
    // ===============================================================================================
    void callback_tim_50hz();
    void callback_sub_lane_kiri(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_lane_tengah(const ros2_interface::msg::PointArray::SharedPtr msg);
    void callback_sub_lane_kanan(const ros2_interface::msg::PointArray::SharedPtr msg);

    // Process
    // ===============================================================================================
    void process_marker();
    void process_fsm();

    // Motion
    // ===============================================================================================
    void manual_motion(float vx, float vy, float wz);
    float obstacle_influence(float gain);
    void follow_lane(float vx, float vy, float wz);

    // Misc
    // ===============================================================================================
    geometry_msgs::msg::Point get_point(double x, double y, double z);
    geometry_msgs::msg::Quaternion get_quat(double r, double p, double y);
    std::vector<geometry_msgs::msg::Point> get_path(float x0, float y0, float x1, float y1, float resolution);
    geometry_msgs::msg::Point get_near(float x, float y, std::vector<geometry_msgs::msg::Point> ps);
    void set_initialpose(float x, float y, float yaw);
};

#endif // MASTER_HPP