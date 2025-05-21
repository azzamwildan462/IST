#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

using namespace std::chrono_literals;

class ObstacleFilter : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_points;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_laserscan;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_master_actuator;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_obs_find;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_lidar_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_filtered_lidar_points_pcl1;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;

    //-----Transform listener
    std::unique_ptr<tf2_ros::Buffer> tf_lidar_map_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_lidar_map_listener;

    std::unique_ptr<tf2_ros::Buffer> tf_lidar_base_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_lidar_base_listener;

    //----Configs
    bool use_raw_lidar = true;
    float scan_yaw_start = -0.785398;
    float scan_yaw_end = 0.785398;
    float scan_r_max = 2.00;
    std::string lidar_frame_id = "lidar1_link";
    std::string lidar_topic = "/scan";
    bool publish_filtered_lidar = false;
    bool use_pointcloud2 = false;
    float scan_box_x_min = 0.0;
    float scan_box_y_min = -0.5;
    float scan_box_x_max = 2.5;
    float scan_box_y_max = 0.5;
    bool use_scan_box = false;

    //----Variables
    int16_t error_code = 0;
    float target_steering_angle = 0;

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
    pcl::PointCloud<pcl::PointXYZ> points_lidar2base_filtered;

    float scan_yaw_target = 0;
    float scan_yaw_minmax = 0;
    float scan_range_decimal = 0;

    HelpLogger logger;

    ObstacleFilter()
        : Node("obstacle_filter")
    {
        this->declare_parameter("use_raw_lidar", true);
        this->get_parameter("use_raw_lidar", use_raw_lidar);

        this->declare_parameter("scan_yaw_start", -0.785398);
        this->get_parameter("scan_yaw_start", scan_yaw_start);

        this->declare_parameter("scan_yaw_end", 0.785398);
        this->get_parameter("scan_yaw_end", scan_yaw_end);

        this->declare_parameter("scan_r_max", 2.00);
        this->get_parameter("scan_r_max", scan_r_max);

        this->declare_parameter("lidar_frame_id", "lidar1_link");
        this->get_parameter("lidar_frame_id", lidar_frame_id);

        this->declare_parameter("publish_filtered_lidar", false);
        this->get_parameter("publish_filtered_lidar", publish_filtered_lidar);

        this->declare_parameter("lidar_topic", "/scan");
        this->get_parameter("lidar_topic", lidar_topic);

        this->declare_parameter("use_pointcloud2", false);
        this->get_parameter("use_pointcloud2", use_pointcloud2);

        this->declare_parameter("scan_box_x_min", 0.0);
        this->get_parameter("scan_box_x_min", scan_box_x_min);

        this->declare_parameter("scan_box_x_max", 2.5);
        this->get_parameter("scan_box_x_max", scan_box_x_max);

        this->declare_parameter("scan_box_y_min", -0.5);
        this->get_parameter("scan_box_y_min", scan_box_y_min);

        this->declare_parameter("scan_box_y_max", 0.5);
        this->get_parameter("scan_box_y_max", scan_box_y_max);

        this->declare_parameter("use_scan_box", false);
        this->get_parameter("use_scan_box", use_scan_box);

        scan_yaw_target = scan_yaw_start * 0.5 + scan_yaw_end * 0.5;
        scan_range_decimal = 1 / scan_r_max;

        while (scan_yaw_target > M_PI)
            scan_yaw_target -= 2 * M_PI;
        while (scan_yaw_target < -M_PI)
            scan_yaw_target += 2 * M_PI;

        scan_yaw_minmax = fabsf(scan_yaw_end - scan_yaw_start) * 0.5;

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ObstacleFilter::callback_tim_50hz, this));

        //----Subscriber
        if (use_pointcloud2)
        {
            sub_lidar_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lidar_topic, 1, std::bind(&ObstacleFilter::callback_sub_lidar_points, this, std::placeholders::_1));
        }
        else
        {
            sub_lidar_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>(
                lidar_topic, 1, std::bind(&ObstacleFilter::callback_sub_lidar_laserscan, this, std::placeholders::_1));
        }
        sub_master_actuator = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/master/actuator", 1, std::bind(&ObstacleFilter::callback_sub_master_actuator, this, std::placeholders::_1));
        //----Publisher
        pub_obs_find = this->create_publisher<std_msgs::msg::Float32>("/obstacle_filter/obs_find", 1);
        pub_filtered_lidar_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_filter/filtered_lidar", 1);
        pub_filtered_lidar_points_pcl1 = this->create_publisher<sensor_msgs::msg::PointCloud>("/obstacle_filter/filtered_lidar_pcl", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("/obstacle_filter/error_code", 1);

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
                // tf_lidar_map = tf_lidar_map_buffer->lookupTransform("map", "lidar1_link", tf2::TimePointZero);
                tf_lidar_base = tf_lidar_base_buffer->lookupTransform("base_link", lidar_frame_id, tf2::TimePointZero);
                tf_is_initialized = true;
            }
            catch (tf2::TransformException &ex)
            {
                logger.error("%s", ex.what());
                tf_is_initialized = false;
            }
        }

        logger.info("ObstacleFilter init success, lidar on %s (%d)", lidar_topic.c_str(), use_pointcloud2);
    }

    void callback_sub_master_actuator(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        target_steering_angle = msg->data[1];
    }

    void callback_sub_lidar_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process Laserscan
        float obs_find = 0;
        float cos_steering_angle = cosf(target_steering_angle);
        float sin_steering_angle = sinf(target_steering_angle);
        float dynamic_scan_box_x_max = scan_box_x_max * cos_steering_angle - scan_box_y_min * sin_steering_angle;
        float dynamic_scan_box_y_max = scan_box_x_max * sin_steering_angle + scan_box_y_min * cos_steering_angle;
        sensor_msgs::msg::PointCloud msg_filtered_scan;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max)
            {
                float angle = msg->angle_min + i * msg->angle_increment;
                float delta_a = scan_yaw_target - angle;
                while (delta_a > M_PI)
                    delta_a -= 2 * M_PI;
                while (delta_a < -M_PI)
                    delta_a += 2 * M_PI;

                // Ketika masuk lingkaran scan
                if (std::fabs(delta_a) < scan_yaw_minmax && range < scan_r_max)
                {
                    float point_x = range * cosf(angle);
                    float point_y = range * sinf(angle);

                    // Ketika masuk scan box
                    if (use_scan_box && point_x > scan_box_x_min && point_x < dynamic_scan_box_x_max && point_y > scan_box_y_min && point_y < dynamic_scan_box_y_max)
                    {
                        geometry_msgs::msg::Point32 pt;
                        pt.x = point_x;
                        pt.y = point_y;
                        msg_filtered_scan.points.push_back(pt);
                        obs_find += scan_range_decimal * (scan_r_max - range);
                    }
                    else if (!use_scan_box)
                    {
                        geometry_msgs::msg::Point32 pt;
                        pt.x = point_x;
                        pt.y = point_y;
                        msg_filtered_scan.points.push_back(pt);
                        obs_find += scan_range_decimal * (scan_r_max - range);
                    }
                }
            }
        }

        // Publish the obstacle detection result
        std_msgs::msg::Float32 msg_obs_find;
        msg_obs_find.data = obs_find;
        pub_obs_find->publish(msg_obs_find);

        if (publish_filtered_lidar)
        {
            pub_filtered_lidar_points_pcl1->publish(msg_filtered_scan);
        }
    }

    void callback_sub_lidar_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // try
        // {
        //     tf_lidar_map = tf_lidar_map_buffer->lookupTransform("map", "lidar1_link", tf2::TimePointZero);
        //     tf_lidar_base = tf_lidar_base_buffer->lookupTransform("base_link", "lidar1_link", tf2::TimePointZero);
        //     tf_is_initialized = true;
        // }
        // catch (tf2::TransformException &ex)
        // {
        //     logger.error("%s", ex.what());
        //     tf_is_initialized = false;
        // }

        if (!tf_is_initialized)
        {
            error_code = 1;
            return;
        }

        // Store the point cloud data
        pcl::fromROSMsg(*msg, points_lidar);

        // Transform the point cloud data to the map frame
        if (points_lidar.empty())
        {
            // points_lidar2map.clear();
            error_code = 2;
            points_lidar2base.clear();
        }
        else
        {
            // pcl_ros::transformPointCloud(points_lidar, points_lidar2map, tf_lidar_map);
            pcl_ros::transformPointCloud(points_lidar, points_lidar2base, tf_lidar_base);
        }

        // Crop all points within -0.35 < x < 0.35 and -0.35 < y < 0.35
        static pcl::CropBox<pcl::PointXYZ> crop_box;
        crop_box.setInputCloud(points_lidar2base.makeShared());
        crop_box.setMin(Eigen::Vector4f(-0.35, -0.35, -100, 1));
        crop_box.setMax(Eigen::Vector4f(0.35, 0.35, 100, 1));
        crop_box.setNegative(true);
        crop_box.filter(points_lidar2base_filtered);

        // Publish the filtered point cloud
        if (publish_filtered_lidar)
        {
            sensor_msgs::msg::PointCloud2 msg_filtered_lidar;
            pcl::toROSMsg(points_lidar2base_filtered, msg_filtered_lidar);
            msg_filtered_lidar.header = msg->header;
            pub_filtered_lidar_points->publish(msg_filtered_lidar);
        }

        // Scan obstacle
        float obs_find = 0;
        for (auto p : points_lidar2base_filtered.points)
        {
            // Terhadap lidar
            float dx = p.x - tf_lidar_base.transform.translation.x;
            float dy = p.y - tf_lidar_base.transform.translation.y;
            float r = sqrtf(dx * dx + dy * dy);
            float a = atan2f(dy, dx);

            // Filter
            float delta_a = scan_yaw_target - a;
            while (delta_a > M_PI)
                delta_a -= 2 * M_PI;
            while (delta_a < -M_PI)
                delta_a += 2 * M_PI;

            if (fabsf(delta_a) < scan_yaw_minmax && r < scan_r_max)
            {
                obs_find = obs_find + 1 * fmaxf(0.3, (1 - r / scan_r_max));
            }
        }

        std_msgs::msg::Float32 msg_obs_find;
        msg_obs_find.data = obs_find;
        pub_obs_find->publish(msg_obs_find);
    }

    void callback_tim_50hz()
    {
        std_msgs::msg::Int16 msg_error_code;
        msg_error_code.data = error_code;
        pub_error_code->publish(msg_error_code);
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