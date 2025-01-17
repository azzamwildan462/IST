#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

class ICP : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_final_pose;

    HelpLogger logger;

    // Configs (static)
    // =======================================================

    // Vars
    // =======================================================
    int16_t error_code = 0;
    float final_pose[3] = {0, 0, 0};

    ICP()
        : Node("ICP")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ICP::callback_tim_50hz, this));

        //----Publisher

        //----Subscriber
        sub_final_pose = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&ICP::callback_sub_final_pose, this, std::placeholders::_1));

        logger.info("ICP initialized");
    }

    void callback_sub_final_pose(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        final_pose[0] = msg->pose.pose.position.x;
        final_pose[1] = msg->pose.pose.position.y;

        // get_angle from quartenion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;

        m.getRPY(roll, pitch, yaw);

        final_pose[2] = yaw;
    }

    void callback_tim_50hz()
    {
    }

    void run_icp()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

        // Fill in the CloudIn data
        cloud_in->width = 5;
        cloud_in->height = 1;
        cloud_in->is_dense = false;
        cloud_in->points.resize(cloud_in->width * cloud_in->height);

        for (size_t i = 0; i < cloud_in->points.size(); ++i)
        {
            cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }

        *cloud_out = *cloud_in;

        for (size_t i = 0; i < cloud_in->points.size(); ++i)
            cloud_out->points[i].x += 0.7f;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        if (icp.hasConverged())
        {
            RCLCPP_INFO(this->get_logger(), "ICP converged");
            RCLCPP_INFO(this->get_logger(), "The score is %f", icp.getFitnessScore());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "ICP did not converge");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_icp = std::make_shared<ICP>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_icp);
    executor.spin();

    return 0;
}