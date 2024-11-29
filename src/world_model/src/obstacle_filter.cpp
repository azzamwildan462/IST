#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ros2_utils/help_logger.hpp"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"

using namespace std::chrono_literals;

class ObstacleFilter : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points;

    //-----Transform listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;

    // Transform
    // ---------
    bool tf_is_initialized = false;
    geometry_msgs::msg::TransformStamped tf_lidar_map;

    // Point cloud
    // -----------
    pcl::PointCloud<pcl::PointXYZ> points_lidar;
    pcl::PointCloud<pcl::PointXYZ> points_base;

    HelpLogger logger;

    ObstacleFilter() : Node("obstacle_filter")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ObstacleFilter::callback_tim_50hz, this));

        //----Subscriber
        sub_lidar_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar1/livox/lidar", 10, std::bind(&ObstacleFilter::callback_sub_lidar_points, this, std::placeholders::_1));

        //-----Tranform listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

        while (!tf_is_initialized)
        {
            rclcpp::sleep_for(1s);
            try
            {
                tf_lidar_map = tf_buffer->lookupTransform("map", "lidar1_link", tf2::TimePointZero);
                tf_is_initialized = true;
            }
            catch (tf2::TransformException &ex)
            {
                logger.error("%s", ex.what());
                tf_is_initialized = false;
            }
        }
    }

    void callback_sub_lidar_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            tf_lidar_map = tf_buffer->lookupTransform("map", "lidar1_link", tf2::TimePointZero);
            tf_is_initialized = true;
        }
        catch (tf2::TransformException &ex)
        {
            logger.error("%s", ex.what());
            tf_is_initialized = false;
        }

        if (!tf_is_initialized)
        {
            return;
        }

        // Store the point cloud data
        pcl::fromROSMsg(*msg, points_lidar);

        // Transform the point cloud data to the base link frame
        if (points_lidar.empty())
        {
            points_base.clear();
        }
        else
        {
            pcl_ros::transformPointCloud(points_lidar, points_base, tf_lidar_map);
        }
    }

    void callback_tim_50hz()
    {
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_obstacle_filter = std::make_shared<ObstacleFilter>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_obstacle_filter);
    executor.spin();

    return 0;
}