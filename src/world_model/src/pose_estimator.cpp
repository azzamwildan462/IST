#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"

class PoseEstimator : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

    //----TransformBroadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    float final_pose_xyo[3] = {0, 0, 0};
    float prev_final_pose_xyo[3] = {0, 0, 0};
    float final_vel_dxdydo[3] = {0, 0, 0};

    PoseEstimator() : Node("master")
    {
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&PoseEstimator::callback_tim_50hz, this));

        //----Publisher
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // final_pose_xyo[0] = 0.5;
        // final_pose_xyo[1] = 2.0;
        // final_pose_xyo[2] = 0.5;
    }

    void callback_tim_50hz()
    {
        static rclcpp::Time time_old = this->now();
        static rclcpp::Time time_now = this->now();
        time_old = time_now;
        time_now = this->now();
        double dt = (time_now - time_old).seconds();

        // Kalkulasi odometry disini
        // Hadrcode, dia akan membenarkan sendiri
        {
            static uint8_t c = 0;
            static rclcpp::Time last_time_change = this->now();

            if ((time_now - last_time_change).seconds() > 45)
            {
                c++;
                if (c > 2)
                {
                    c = 0;
                }
                last_time_change = time_now;
            }

            if (c == 0)
            {
                final_pose_xyo[0] = -0.99;
                final_pose_xyo[1] = -2;
                final_pose_xyo[2] = 0.0;
            }
            else if (c == 1)
            {
                final_pose_xyo[0] = -1;
                final_pose_xyo[1] = 1;
                final_pose_xyo[2] = -0.0;
            }
            else if (c == 2)
            {
                final_pose_xyo[0] = -1;
                final_pose_xyo[1] = -2;
                final_pose_xyo[2] = -0.0;
            }
        }

        final_vel_dxdydo[0] = (final_pose_xyo[0] - prev_final_pose_xyo[0]) / dt;
        final_vel_dxdydo[1] = (final_pose_xyo[1] - prev_final_pose_xyo[1]) / dt;
        final_vel_dxdydo[2] = (final_pose_xyo[2] - prev_final_pose_xyo[2]) / dt;
        memcpy(prev_final_pose_xyo, final_pose_xyo, sizeof(final_pose_xyo));

        tf2::Quaternion q;
        q.setRPY(0, 0, final_pose_xyo[2]);

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
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_pose_estimator = std::make_shared<PoseEstimator>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_pose_estimator);
    executor.spin();

    return 0;
}