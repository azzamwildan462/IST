#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class PoseEstimator : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_encoder_meter;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_encoder;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gyro;

    //----TransformBroadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Configs (static)
    // =======================================================
    float encoder_to_meter = 1.0;

    uint16_t encoder[2] = {0, 0};
    uint16_t prev_encoder[2] = {0, 0};
    float gyro = 0;
    float prev_gyro = 0;

    float final_pose_xyo[3] = {0, 0, 0};
    float final_vel_dxdydo[3] = {0, 0, 0};

    int32_t sensor_left_encoder = 0;
    int32_t sensor_right_encoder = 0;

    // Vars
    // =======================================================
    int16_t error_code = 0;

    PoseEstimator()
        : Node("master")
    {
        RCLCPP_INFO(this->get_logger(), "PoseEstimator init");
        this->declare_parameter("encoder_to_meter", 1.0);
        this->get_parameter("encoder_to_meter", encoder_to_meter);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&PoseEstimator::callback_tim_50hz, this));

        //----Publisher
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("error_code", 10);
        pub_encoder_meter = this->create_publisher<std_msgs::msg::Float32>("encoder_meter", 1);

        sub_encoder = this->create_subscription<std_msgs::msg::Int32>(
            "/can/encoder", 1, std::bind(&PoseEstimator::callback_sub_encoder, this, std::placeholders::_1));
        sub_gyro = this->create_subscription<sensor_msgs::msg::Imu>(
            "/hardware/imu", 1, std::bind(&PoseEstimator::callback_sub_gyro, this, std::placeholders::_1));
    }

    void callback_sub_encoder(const std_msgs::msg::Int32::SharedPtr msg)
    {
        sensor_left_encoder = msg->data * 0.5;
        sensor_right_encoder = msg->data * 0.5;

        std_msgs::msg::Float32 msg_encoder_meter;
        msg_encoder_meter.data = msg->data * encoder_to_meter * 50;
        pub_encoder_meter->publish(msg_encoder_meter);
    }

    void callback_tim_50hz()
    {
        static rclcpp::Time time_old = this->now();
        static rclcpp::Time time_now = this->now();
        time_old = time_now;
        time_now = this->now();
        double dt = (time_now - time_old).seconds();

        if (dt < FLT_EPSILON)
        {
            error_code = 1;
            return;
        }

        /* Not used */
        // int16_t d_left_encoder = encoder[0] - prev_encoder[0];
        // int16_t d_right_encoder = encoder[1] - prev_encoder[1];
        //======================================================

        float d_gyro = gyro - prev_gyro;
        memcpy(prev_encoder, encoder, sizeof(prev_encoder));
        prev_gyro = gyro;

        final_vel_dxdydo[0] = (sensor_left_encoder + sensor_right_encoder) / 2.0 * cosf(final_pose_xyo[2]) * encoder_to_meter / dt;
        final_vel_dxdydo[1] = (sensor_left_encoder + sensor_right_encoder) / 2.0 * sinf(final_pose_xyo[2]) * encoder_to_meter / dt;
        final_vel_dxdydo[2] = d_gyro;

        while (final_vel_dxdydo[2] > M_PI)
            final_vel_dxdydo[2] -= 2 * M_PI;
        while (final_vel_dxdydo[2] < -M_PI)
            final_vel_dxdydo[2] += 2 * M_PI;

        final_pose_xyo[0] += final_vel_dxdydo[0] * dt;
        final_pose_xyo[1] += final_vel_dxdydo[1] * dt;
        final_pose_xyo[2] += final_vel_dxdydo[2] * dt;

        while (final_pose_xyo[2] > M_PI)
            final_pose_xyo[2] -= 2 * M_PI;
        while (final_pose_xyo[2] < -M_PI)
            final_pose_xyo[2] += 2 * M_PI;

        float yaw_deg = final_pose_xyo[2] * 180.0 / M_PI;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_deg);

        nav_msgs::msg::Odometry msg_odom;
        msg_odom.header.stamp = time_now;
        msg_odom.header.frame_id = "odom";
        msg_odom.child_frame_id = "base_link";
        msg_odom.pose.pose.position.x = final_pose_xyo[0];
        msg_odom.pose.pose.position.y = final_pose_xyo[1];
        msg_odom.pose.pose.orientation.x = q.x();
        msg_odom.pose.pose.orientation.y = q.y();
        msg_odom.pose.pose.orientation.z = q.z();
        msg_odom.pose.pose.orientation.w = q.w();
        msg_odom.pose.covariance[0] = 1e-2;
        msg_odom.pose.covariance[7] = 1e-2;
        msg_odom.pose.covariance[14] = 1e6;
        msg_odom.pose.covariance[21] = 1e6;
        msg_odom.pose.covariance[28] = 1e6;
        msg_odom.pose.covariance[35] = 1e-2;
        msg_odom.twist.twist.linear.x = final_vel_dxdydo[0];
        msg_odom.twist.twist.linear.y = final_vel_dxdydo[1];
        msg_odom.twist.twist.angular.z = final_vel_dxdydo[2];
        msg_odom.twist.covariance[0] = 1e-2;
        msg_odom.twist.covariance[7] = 1e-2;
        msg_odom.twist.covariance[14] = 1e6;
        msg_odom.twist.covariance[21] = 1e6;
        msg_odom.twist.covariance[28] = 1e6;
        msg_odom.twist.covariance[35] = 1e-2;
        pub_odom->publish(msg_odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = time_now;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = final_pose_xyo[0];
        tf.transform.translation.y = final_pose_xyo[1];
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(tf);

        std_msgs::msg::Int16 msg_error_code;
        msg_error_code.data = error_code;
        pub_error_code->publish(msg_error_code);
    }

    void callback_sub_gyro(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // get_angle from quartenion
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        gyro = yaw;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_pose_estimator = std::make_shared<PoseEstimator>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_pose_estimator);
    executor.spin();

    return 0;
}