#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ros2_utils/help_logger.hpp"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>

using namespace std::chrono_literals;

class ObstacleFilter : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points;

    //-----Transform listener
    std::unique_ptr<tf2_ros::Buffer> tf_lidar_map_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_lidar_map_listener;

    std::unique_ptr<tf2_ros::Buffer> tf_lidar_base_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_lidar_base_listener;

    // Transform
    // ---------
    bool tf_is_initialized = false;
    geometry_msgs::msg::TransformStamped tf_lidar_map;
    geometry_msgs::msg::TransformStamped tf_lidar_base;

    // Point cloud
    // -----------
    pcl::PointCloud<pcl::PointXYZ> points_lidar;
    pcl::PointCloud<pcl::PointXYZ> points_lidar2map;
    pcl::PointCloud<pcl::PointXYZ> points_lidar2base;

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
        tf_lidar_map_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_lidar_map_listener = std::make_unique<tf2_ros::TransformListener>(*tf_lidar_map_buffer);

        tf_lidar_base_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_lidar_base_listener = std::make_unique<tf2_ros::TransformListener>(*tf_lidar_base_buffer);

        while (!tf_is_initialized)
        {
            rclcpp::sleep_for(1s);
            try
            {
                tf_lidar_map = tf_lidar_map_buffer->lookupTransform("map", "lidar1_link", tf2::TimePointZero);
                tf_lidar_base = tf_lidar_base_buffer->lookupTransform("base_link", "lidar1_link", tf2::TimePointZero);
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
            tf_lidar_map = tf_lidar_map_buffer->lookupTransform("map", "lidar1_link", tf2::TimePointZero);
            tf_lidar_base = tf_lidar_base_buffer->lookupTransform("base_link", "lidar1_link", tf2::TimePointZero);
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

        // Transform the point cloud data to the map frame
        if (points_lidar.empty())
        {
            points_lidar2map.clear();
            points_lidar2base.clear();
        }
        else
        {
            pcl_ros::transformPointCloud(points_lidar, points_lidar2map, tf_lidar_map);
            pcl_ros::transformPointCloud(points_lidar, points_lidar2base, tf_lidar_base);
        }

        // DBSCAN clustering
        dbscan_clustering(points_lidar2map.makeShared(), 0.5, 5);
    }

    void callback_tim_50hz()
    {
    }

    void dbscan_clustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float epsilon, int minPts)
    {
        // Initialize cluster labels (-1 for noise, 0 for unvisited)
        std::vector<int> labels(cloud->size(), 0);
        int clusterID = 0;

        // KD-Tree for fast neighbor search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        for (size_t i = 0; i < cloud->size(); ++i)
        {
            if (labels[i] != 0)
                continue; // Already processed or noise

            // Find neighbors within epsilon radius
            std::vector<int> neighbors;
            std::vector<float> distances;
            if (tree->radiusSearch(cloud->points[i], epsilon, neighbors, distances) < minPts)
            {
                labels[i] = -1; // Mark as noise
                continue;
            }

            // Create a new cluster
            clusterID++;
            labels[i] = clusterID;
            std::vector<int> clusterQueue(neighbors);

            while (!clusterQueue.empty())
            {
                int currentPoint = clusterQueue.back();
                clusterQueue.pop_back();

                if (labels[currentPoint] == -1)
                {
                    labels[currentPoint] = clusterID; // Noise becomes part of the cluster
                }

                if (labels[currentPoint] != 0)
                    continue; // Skip already visited points
                labels[currentPoint] = clusterID;

                // Find neighbors for the current point
                std::vector<int> subNeighbors;
                std::vector<float> subDistances;
                if (tree->radiusSearch(cloud->points[currentPoint], epsilon, subNeighbors, subDistances) >= minPts)
                {
                    clusterQueue.insert(clusterQueue.end(), subNeighbors.begin(), subNeighbors.end());
                }
            }
        }

        // Print clusters
        logger.info("Number of clusters: %d", clusterID);
        for (int i = 1; i <= clusterID; ++i)
        {
            logger.info("Cluster %d points:", i);
            for (size_t j = 0; j < labels.size(); ++j)
            {
                if (labels[j] == i)
                {
                    logger.info("(%f, %f, %f)", cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
                }
            }
        }
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