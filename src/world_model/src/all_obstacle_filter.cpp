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
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class AllObstacleFilter : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_scan_box;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_kanan_points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_kiri_points;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_camera_points;

    rclcpp::CallbackGroup::SharedPtr cb_lidar_kanan_points;
    rclcpp::CallbackGroup::SharedPtr cb_lidar_kiri_points;
    rclcpp::CallbackGroup::SharedPtr cb_camera_points;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_result_all_obstacle;

    //----Configs
    float scan_box_x_min = 0.0;
    float scan_box_y_min = -0.5;
    float scan_box_x_max = 2.5;
    float scan_box_y_max = 0.5;
    float scan_box_z_min = -1.0;
    float scan_box_z_max = 1.0;
    std::string lidar_kanan_topic = "/lidar_kanan_points";
    std::string lidar_kiri_topic = "/lidar_kiri_points";
    std::string camera_topic = "/camera/rs2_cam_main/depth/color/points";
    std::string lidar_kanan_frame_id = "lidar_kanan_link";
    std::string lidar_kiri_frame_id = "lidar_kiri_link";
    std::string camera_frame_id = "camera_depth_optical_frame";
    float exclude_lidar_x_min = -0.1;
    float exclude_lidar_y_min = -0.1;
    float exclude_lidar_z_min = -0.1;
    float exclude_lidar_x_max = 0.1;
    float exclude_lidar_y_max = 0.1;
    float exclude_lidar_z_max = 0.1;
    float scan_toribay_x_min = -0.0;
    float scan_toribay_y_min = 1.5;
    float scan_toribay_z_min = 2.5;
    float scan_toribay_x_max = 2.5;
    float scan_toribay_y_max = 2.5;
    float scan_toribay_z_max = 3.0;
    float scan_toribay_real_wtf_x_min = -0.0;
    float scan_toribay_real_wtf_y_min = 0.5;
    float scan_toribay_real_wtf_z_min = 2.5;
    float scan_toribay_real_wtf_x_max = 0.0;
    float scan_toribay_real_wtf_y_max = 0.5;
    float scan_toribay_real_wtf_z_max = 3.0;
    float gain_lidar = 10.0;

    //----Variables
    float result_lidar_kanan = 0.0;
    float result_lidar_kiri = 0.0;
    float result_camera = 0.0;
    float result_toribay_kanan = 0.0;
    float result_toribay_kiri = 0.0;
    float result_toribay_real_wtf_kanan = 0.0;
    float result_toribay_real_wtf_kiri = 0.0;
    geometry_msgs::msg::PointStamped scan_box_kiri_belakang_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_box_kanan_depan_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_box_kiri_belakang_lidar_kiri;
    geometry_msgs::msg::PointStamped scan_box_kanan_depan_lidar_kiri;
    geometry_msgs::msg::PointStamped scan_box_kiri_belakang_camera;
    geometry_msgs::msg::PointStamped scan_box_kanan_depan_camera;

    geometry_msgs::msg::PointStamped scan_toribay_kiri_belakang_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_toribay_kanan_depan_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_toribay_kiri_belakang_lidar_kiri;
    geometry_msgs::msg::PointStamped scan_toribay_kanan_depan_lidar_kiri;

    geometry_msgs::msg::PointStamped scan_toribay_real_wtf_kiri_belakang_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_toribay_real_wtf_kanan_depan_lidar_kanan;
    geometry_msgs::msg::PointStamped scan_toribay_real_wtf_kiri_belakang_lidar_kiri;
    geometry_msgs::msg::PointStamped scan_toribay_real_wtf_kanan_depan_lidar_kiri;

    // Transform
    // ---------
    std::unique_ptr<tf2_ros::Buffer> tf_base2_lidar_kanan_buffer;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_lidar_kiri_buffer;
    std::unique_ptr<tf2_ros::Buffer> tf_base2_camera_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_kanan_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_lidar_kiri_listener;
    std::unique_ptr<tf2_ros::TransformListener> tf_base2_camera_listener;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_kanan;
    geometry_msgs::msg::TransformStamped tf_base2_lidar_kiri;
    geometry_msgs::msg::TransformStamped tf_base2_camera;

    bool is_tf_initialized = false;

    HelpLogger logger;

    AllObstacleFilter()
        : Node("obstacle_filter")
    {
        this->declare_parameter("scan_box_x_min", 0.0);
        this->get_parameter("scan_box_x_min", scan_box_x_min);

        this->declare_parameter("scan_box_x_max", 2.5);
        this->get_parameter("scan_box_x_max", scan_box_x_max);

        this->declare_parameter("scan_box_y_min", -0.5);
        this->get_parameter("scan_box_y_min", scan_box_y_min);

        this->declare_parameter("scan_box_y_max", 0.5);
        this->get_parameter("scan_box_y_max", scan_box_y_max);

        this->declare_parameter("scan_box_z_min", 0.3);
        this->get_parameter("scan_box_z_min", scan_box_z_min);

        this->declare_parameter("scan_box_z_max", 2.0);
        this->get_parameter("scan_box_z_max", scan_box_z_max);

        this->declare_parameter("lidar_kanan_topic", "/lidar_kanan_points");
        this->get_parameter("lidar_kanan_topic", lidar_kanan_topic);

        this->declare_parameter("lidar_kiri_topic", "/lidar_kiri_points");
        this->get_parameter("lidar_kiri_topic", lidar_kiri_topic);

        this->declare_parameter("camera_topic", "/camera/rs2_cam_main/depth/color/points");
        this->get_parameter("camera_topic", camera_topic);

        this->declare_parameter("lidar_kanan_frame_id", "lidar_kanan_link");
        this->get_parameter("lidar_kanan_frame_id", lidar_kanan_frame_id);

        this->declare_parameter("lidar_kiri_frame_id", "lidar_kiri_link");
        this->get_parameter("lidar_kiri_frame_id", lidar_kiri_frame_id);

        this->declare_parameter("camera_frame_id", "camera_depth_optical_frame");
        this->get_parameter("camera_frame_id", camera_frame_id);

        this->declare_parameter("exclude_lidar_x_min", -0.1);
        this->get_parameter("exclude_lidar_x_min", exclude_lidar_x_min);

        this->declare_parameter("exclude_lidar_y_min", -0.1);
        this->get_parameter("exclude_lidar_y_min", exclude_lidar_y_min);

        this->declare_parameter("exclude_lidar_z_min", -0.1);
        this->get_parameter("exclude_lidar_z_min", exclude_lidar_z_min);

        this->declare_parameter("exclude_lidar_x_max", 0.1);
        this->get_parameter("exclude_lidar_x_max", exclude_lidar_x_max);

        this->declare_parameter("exclude_lidar_y_max", 0.1);
        this->get_parameter("exclude_lidar_y_max", exclude_lidar_y_max);

        this->declare_parameter("exclude_lidar_z_max", 0.1);
        this->get_parameter("exclude_lidar_z_max", exclude_lidar_z_max);

        this->declare_parameter("scan_toribay_x_min", -1.0);
        this->get_parameter("scan_toribay_x_min", scan_toribay_x_min);

        this->declare_parameter("scan_toribay_y_min", -0.5);
        this->get_parameter("scan_toribay_y_min", scan_toribay_y_min);

        this->declare_parameter("scan_toribay_z_min", 0.7);
        this->get_parameter("scan_toribay_z_min", scan_toribay_z_min);

        this->declare_parameter("scan_toribay_x_max", -0.3);
        this->get_parameter("scan_toribay_x_max", scan_toribay_x_max);

        this->declare_parameter("scan_toribay_y_max", 0.5);
        this->get_parameter("scan_toribay_y_max", scan_toribay_y_max);

        this->declare_parameter("scan_toribay_z_max", 2.0);
        this->get_parameter("scan_toribay_z_max", scan_toribay_z_max);

        this->declare_parameter("scan_toribay_real_wtf_x_min", -1.3);
        this->get_parameter("scan_toribay_real_wtf_x_min", scan_toribay_real_wtf_x_min);

        this->declare_parameter("scan_toribay_real_wtf_y_min", -0.5);
        this->get_parameter("scan_toribay_real_wtf_y_min", scan_toribay_real_wtf_y_min);

        this->declare_parameter("scan_toribay_real_wtf_z_min", 0.7);
        this->get_parameter("scan_toribay_real_wtf_z_min", scan_toribay_real_wtf_z_min);

        this->declare_parameter("scan_toribay_real_wtf_x_max", -0.3);
        this->get_parameter("scan_toribay_real_wtf_x_max", scan_toribay_real_wtf_x_max);

        this->declare_parameter("scan_toribay_real_wtf_y_max", 0.5);
        this->get_parameter("scan_toribay_real_wtf_y_max", scan_toribay_real_wtf_y_max);

        this->declare_parameter("scan_toribay_real_wtf_z_max", 2.0);
        this->get_parameter("scan_toribay_real_wtf_z_max", scan_toribay_real_wtf_z_max);

        this->declare_parameter("gain_lidar", 10.0);
        this->get_parameter("gain_lidar", gain_lidar);

        //----Logger
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        //----Transform listener
        tf_base2_lidar_kanan_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_base2_lidar_kiri_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_base2_camera_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_base2_lidar_kanan_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_lidar_kanan_buffer, this);
        tf_base2_lidar_kiri_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_lidar_kiri_buffer, this);
        tf_base2_camera_listener = std::make_unique<tf2_ros::TransformListener>(*tf_base2_camera_buffer, this);

        while (!is_tf_initialized)
        {
            rclcpp::sleep_for(1s);
            logger.warn("Waiting for transforms to be available...");
            try
            {
                tf_base2_lidar_kanan = tf_base2_lidar_kanan_buffer->lookupTransform(lidar_kanan_frame_id, "base_link", tf2::TimePointZero);
                tf_base2_lidar_kiri = tf_base2_lidar_kiri_buffer->lookupTransform(lidar_kiri_frame_id, "base_link", tf2::TimePointZero);
                tf_base2_camera = tf_base2_camera_buffer->lookupTransform(camera_frame_id, "base_link", tf2::TimePointZero);
                is_tf_initialized = true;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
                is_tf_initialized = false;
            }
        }

        {
            geometry_msgs::msg::PointStamped point_batas_kiri_belakang;
            geometry_msgs::msg::PointStamped point_batas_kanan_depan;

            point_batas_kiri_belakang.point.x = scan_toribay_x_min;
            point_batas_kiri_belakang.point.y = scan_toribay_y_max;
            point_batas_kiri_belakang.point.z = scan_toribay_z_min;
            point_batas_kiri_belakang.header.frame_id = "base_link";
            point_batas_kiri_belakang.header.stamp = this->now();

            point_batas_kanan_depan.point.x = scan_toribay_x_max;
            point_batas_kanan_depan.point.y = scan_toribay_y_min;
            point_batas_kanan_depan.point.z = scan_toribay_z_max;
            point_batas_kanan_depan.header.frame_id = "base_link";
            point_batas_kanan_depan.header.stamp = this->now();

            try
            {
                tf2::doTransform(point_batas_kiri_belakang, scan_toribay_kiri_belakang_lidar_kanan, tf_base2_lidar_kanan);
                tf2::doTransform(point_batas_kanan_depan, scan_toribay_kanan_depan_lidar_kanan, tf_base2_lidar_kanan);
                tf2::doTransform(point_batas_kiri_belakang, scan_toribay_kiri_belakang_lidar_kiri, tf_base2_lidar_kiri);
                tf2::doTransform(point_batas_kanan_depan, scan_toribay_kanan_depan_lidar_kiri, tf_base2_lidar_kiri);
            }
            catch (const tf2::TransformException &ex)
            {
                logger.error("Transform failed: %s", ex.what());
                return;
            }
        }

        {
            geometry_msgs::msg::PointStamped point_batas_kiri_belakang;
            geometry_msgs::msg::PointStamped point_batas_kanan_depan;

            point_batas_kiri_belakang.point.x = scan_toribay_real_wtf_x_min;
            point_batas_kiri_belakang.point.y = scan_toribay_real_wtf_y_max;
            point_batas_kiri_belakang.point.z = scan_toribay_real_wtf_z_min;
            point_batas_kiri_belakang.header.frame_id = "base_link";
            point_batas_kiri_belakang.header.stamp = this->now();

            point_batas_kanan_depan.point.x = scan_toribay_real_wtf_x_max;
            point_batas_kanan_depan.point.y = scan_toribay_real_wtf_y_min;
            point_batas_kanan_depan.point.z = scan_toribay_real_wtf_z_max;
            point_batas_kanan_depan.header.frame_id = "base_link";
            point_batas_kanan_depan.header.stamp = this->now();

            try
            {
                tf2::doTransform(point_batas_kiri_belakang, scan_toribay_real_wtf_kiri_belakang_lidar_kanan, tf_base2_lidar_kanan);
                tf2::doTransform(point_batas_kanan_depan, scan_toribay_real_wtf_kanan_depan_lidar_kanan, tf_base2_lidar_kanan);
                tf2::doTransform(point_batas_kiri_belakang, scan_toribay_real_wtf_kiri_belakang_lidar_kiri, tf_base2_lidar_kiri);
                tf2::doTransform(point_batas_kanan_depan, scan_toribay_real_wtf_kanan_depan_lidar_kiri, tf_base2_lidar_kiri);
            }
            catch (const tf2::TransformException &ex)
            {
                logger.error("Transform failed: %s", ex.what());
                return;
            }
        }

        //----Callback groups
        cb_lidar_kanan_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_lidar_kiri_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_camera_points = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_lidar_kanan_options = rclcpp::SubscriptionOptions();
        sub_lidar_kanan_options.callback_group = cb_lidar_kanan_points;
        auto sub_lidar_kiri_options = rclcpp::SubscriptionOptions();
        sub_lidar_kiri_options.callback_group = cb_lidar_kiri_points;
        auto sub_camera_options = rclcpp::SubscriptionOptions();
        sub_camera_options.callback_group = cb_camera_points;

        //----Subscriber
        sub_lidar_kanan_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_kanan_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_kanan_points, this, std::placeholders::_1), sub_lidar_kanan_options);
        sub_lidar_kiri_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_kiri_topic, 1, std::bind(&AllObstacleFilter::callback_sub_lidar_kiri_points, this, std::placeholders::_1), sub_lidar_kiri_options);
        sub_camera_points = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            camera_topic, 1, std::bind(&AllObstacleFilter::callback_sub_camera_points, this, std::placeholders::_1), sub_camera_options);
        sub_scan_box = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/master/obstacle_scan_box", 1, std::bind(&AllObstacleFilter::callback_sub_scan_box, this, std::placeholders::_1));

        //----Publisher
        pub_result_all_obstacle = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/all_obstacle_filter/result_all_obstacle", 1);

        //----Timer
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&AllObstacleFilter::callback_tim_routine, this));

        logger.info("AllObstacleFilter init success");
    }

    void callback_sub_scan_box(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 4)
        {
            scan_box_x_min = msg->data[0];
            scan_box_y_min = msg->data[1];
            scan_box_x_max = msg->data[2];
            scan_box_y_max = msg->data[3];
        }
    }

    void callback_sub_lidar_kanan_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        static uint16_t counter_pembagi = 0;
        if (counter_pembagi++ <= 5)
        {
            (void)msg;
            return;
        }
        counter_pembagi = 0;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // exclude cover lidar
        pcl::CropBox<pcl::PointXYZ> crop_box_exclude;
        pcl::PointCloud<pcl::PointXYZ> points_excluded;
        crop_box_exclude.setInputCloud(cloud.makeShared());
        crop_box_exclude.setMin(Eigen::Vector4f(exclude_lidar_x_min, exclude_lidar_y_min, exclude_lidar_z_min, 1));
        crop_box_exclude.setMax(Eigen::Vector4f(exclude_lidar_x_max, exclude_lidar_y_max, exclude_lidar_z_max, 1));
        crop_box_exclude.setNegative(true);
        crop_box_exclude.filter(points_excluded);

        float min_x = fminf(scan_box_kiri_belakang_lidar_kanan.point.x, scan_box_kanan_depan_lidar_kanan.point.x);
        float max_x = fmaxf(scan_box_kiri_belakang_lidar_kanan.point.x, scan_box_kanan_depan_lidar_kanan.point.x);
        float min_y = fminf(scan_box_kiri_belakang_lidar_kanan.point.y, scan_box_kanan_depan_lidar_kanan.point.y);
        float max_y = fmaxf(scan_box_kiri_belakang_lidar_kanan.point.y, scan_box_kanan_depan_lidar_kanan.point.y);
        float min_z = fminf(scan_box_kiri_belakang_lidar_kanan.point.z, scan_box_kanan_depan_lidar_kanan.point.z);
        float max_z = fmaxf(scan_box_kiri_belakang_lidar_kanan.point.z, scan_box_kanan_depan_lidar_kanan.point.z);

        // Crop box filter based on scan box cloud lidar kanan
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> points_cropped;
        crop_box_filter.setInputCloud(points_excluded.makeShared());
        crop_box_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
        crop_box_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
        crop_box_filter.setNegative(false); // Keep points inside the box
        crop_box_filter.filter(points_cropped);

        result_lidar_kanan = points_cropped.size();

        // Scan toribay
        min_x = fminf(scan_toribay_kiri_belakang_lidar_kanan.point.x, scan_toribay_kanan_depan_lidar_kanan.point.x);
        max_x = fmaxf(scan_toribay_kiri_belakang_lidar_kanan.point.x, scan_toribay_kanan_depan_lidar_kanan.point.x);
        min_y = fminf(scan_toribay_kiri_belakang_lidar_kanan.point.y, scan_toribay_kanan_depan_lidar_kanan.point.y);
        max_y = fmaxf(scan_toribay_kiri_belakang_lidar_kanan.point.y, scan_toribay_kanan_depan_lidar_kanan.point.y);
        min_z = fminf(scan_toribay_kiri_belakang_lidar_kanan.point.z, scan_toribay_kanan_depan_lidar_kanan.point.z);
        max_z = fmaxf(scan_toribay_kiri_belakang_lidar_kanan.point.z, scan_toribay_kanan_depan_lidar_kanan.point.z);

        // Crop box filter based on scan toribay cloud lidar kanan
        pcl::CropBox<pcl::PointXYZ> crop_box_toribay_filter;
        pcl::PointCloud<pcl::PointXYZ> points_toribay_cropped;
        crop_box_toribay_filter.setInputCloud(cloud.makeShared());
        crop_box_toribay_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
        crop_box_toribay_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
        crop_box_toribay_filter.setNegative(false); // Keep points inside the box
        crop_box_toribay_filter.filter(points_toribay_cropped);

        result_toribay_kanan = points_toribay_cropped.size();

        // Scan toribay real wtf
        min_x = fminf(scan_toribay_real_wtf_kiri_belakang_lidar_kanan.point.x, scan_toribay_real_wtf_kanan_depan_lidar_kanan.point.x);
        max_x = fmaxf(scan_toribay_real_wtf_kiri_belakang_lidar_kanan.point.x, scan_toribay_real_wtf_kanan_depan_lidar_kanan.point.x);
        min_y = fminf(scan_toribay_real_wtf_kiri_belakang_lidar_kanan.point.y, scan_toribay_real_wtf_kanan_depan_lidar_kanan.point.y);
        max_y = fmaxf(scan_toribay_real_wtf_kiri_belakang_lidar_kanan.point.y, scan_toribay_real_wtf_kanan_depan_lidar_kanan.point.y);
        min_z = fminf(scan_toribay_real_wtf_kiri_belakang_lidar_kanan.point.z, scan_toribay_real_wtf_kanan_depan_lidar_kanan.point.z);
        max_z = fmaxf(scan_toribay_real_wtf_kiri_belakang_lidar_kanan.point.z, scan_toribay_real_wtf_kanan_depan_lidar_kanan.point.z);

        // Scan toribay real wtf
        pcl::CropBox<pcl::PointXYZ> crop_box_toribay_real_wtf_filter;
        pcl::PointCloud<pcl::PointXYZ> points_toribay_real_wtf_cropped;
        crop_box_toribay_real_wtf_filter.setInputCloud(cloud.makeShared());
        crop_box_toribay_real_wtf_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
        crop_box_toribay_real_wtf_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
        crop_box_toribay_real_wtf_filter.setNegative(false); // Keep points inside the box
        crop_box_toribay_real_wtf_filter.filter(points_toribay_real_wtf_cropped);

        result_toribay_real_wtf_kanan = points_toribay_real_wtf_cropped.size();
    }

    void callback_sub_lidar_kiri_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        static uint16_t counter_pembagi = 0;
        if (counter_pembagi++ <= 5)
        {
            (void)msg;
            return;
        }
        counter_pembagi = 0;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // exclude cover lidar
        pcl::CropBox<pcl::PointXYZ> crop_box_exclude;
        pcl::PointCloud<pcl::PointXYZ> points_excluded;
        crop_box_exclude.setInputCloud(cloud.makeShared());
        crop_box_exclude.setMin(Eigen::Vector4f(exclude_lidar_x_min, exclude_lidar_y_min, exclude_lidar_z_min, 1));
        crop_box_exclude.setMax(Eigen::Vector4f(exclude_lidar_x_max, exclude_lidar_y_max, exclude_lidar_z_max, 1));
        crop_box_exclude.setNegative(true);
        crop_box_exclude.filter(points_excluded);

        float min_x = fminf(scan_box_kiri_belakang_lidar_kiri.point.x, scan_box_kanan_depan_lidar_kiri.point.x);
        float max_x = fmaxf(scan_box_kiri_belakang_lidar_kiri.point.x, scan_box_kanan_depan_lidar_kiri.point.x);
        float min_y = fminf(scan_box_kiri_belakang_lidar_kiri.point.y, scan_box_kanan_depan_lidar_kiri.point.y);
        float max_y = fmaxf(scan_box_kiri_belakang_lidar_kiri.point.y, scan_box_kanan_depan_lidar_kiri.point.y);
        float min_z = fminf(scan_box_kiri_belakang_lidar_kiri.point.z, scan_box_kanan_depan_lidar_kiri.point.z);
        float max_z = fmaxf(scan_box_kiri_belakang_lidar_kiri.point.z, scan_box_kanan_depan_lidar_kiri.point.z);

        // Crop box filter based on scan box cloud lidar kiri
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> points_cropped;
        crop_box_filter.setInputCloud(points_excluded.makeShared());
        crop_box_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
        crop_box_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
        crop_box_filter.setNegative(false); // Keep points inside the box
        crop_box_filter.filter(points_cropped);

        result_lidar_kiri = points_cropped.size();

        // Scan toribay
        min_x = fminf(scan_toribay_kiri_belakang_lidar_kiri.point.x, scan_toribay_kanan_depan_lidar_kiri.point.x);
        max_x = fmaxf(scan_toribay_kiri_belakang_lidar_kiri.point.x, scan_toribay_kanan_depan_lidar_kiri.point.x);
        min_y = fminf(scan_toribay_kiri_belakang_lidar_kiri.point.y, scan_toribay_kanan_depan_lidar_kiri.point.y);
        max_y = fmaxf(scan_toribay_kiri_belakang_lidar_kiri.point.y, scan_toribay_kanan_depan_lidar_kiri.point.y);
        min_z = fminf(scan_toribay_kiri_belakang_lidar_kiri.point.z, scan_toribay_kanan_depan_lidar_kiri.point.z);
        max_z = fmaxf(scan_toribay_kiri_belakang_lidar_kiri.point.z, scan_toribay_kanan_depan_lidar_kiri.point.z);

        // Crop box filter based on scan toribay cloud lidar kiri
        pcl::CropBox<pcl::PointXYZ> crop_box_toribay_filter;
        pcl::PointCloud<pcl::PointXYZ> points_toribay_cropped;
        crop_box_toribay_filter.setInputCloud(cloud.makeShared());
        crop_box_toribay_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
        crop_box_toribay_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
        crop_box_toribay_filter.setNegative(false); // Keep points inside the box
        crop_box_toribay_filter.filter(points_toribay_cropped);

        result_toribay_kiri = points_toribay_cropped.size();

        // Scan toribay real wtf
        min_x = fminf(scan_toribay_real_wtf_kiri_belakang_lidar_kiri.point.x, scan_toribay_real_wtf_kanan_depan_lidar_kiri.point.x);
        max_x = fmaxf(scan_toribay_real_wtf_kiri_belakang_lidar_kiri.point.x, scan_toribay_real_wtf_kanan_depan_lidar_kiri.point.x);
        min_y = fminf(scan_toribay_real_wtf_kiri_belakang_lidar_kiri.point.y, scan_toribay_real_wtf_kanan_depan_lidar_kiri.point.y);
        max_y = fmaxf(scan_toribay_real_wtf_kiri_belakang_lidar_kiri.point.y, scan_toribay_real_wtf_kanan_depan_lidar_kiri.point.y);
        min_z = fminf(scan_toribay_real_wtf_kiri_belakang_lidar_kiri.point.z, scan_toribay_real_wtf_kanan_depan_lidar_kiri.point.z);
        max_z = fmaxf(scan_toribay_real_wtf_kiri_belakang_lidar_kiri.point.z, scan_toribay_real_wtf_kanan_depan_lidar_kiri.point.z);

        // Scan toribay real wtf
        pcl::CropBox<pcl::PointXYZ> crop_box_toribay_real_wtf_filter;
        pcl::PointCloud<pcl::PointXYZ> points_toribay_real_wtf_cropped;
        crop_box_toribay_real_wtf_filter.setInputCloud(cloud.makeShared());
        crop_box_toribay_real_wtf_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
        crop_box_toribay_real_wtf_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
        crop_box_toribay_real_wtf_filter.setNegative(false); // Keep points inside the box
        crop_box_toribay_real_wtf_filter.filter(points_toribay_real_wtf_cropped);

        result_toribay_real_wtf_kiri = points_toribay_real_wtf_cropped.size();
    }

    void callback_sub_camera_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        static uint16_t counter_pembagi = 0;
        if (counter_pembagi++ <= 5)
        {
            (void)msg;
            return;
        }
        counter_pembagi = 0;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        float min_x = fminf(scan_box_kiri_belakang_camera.point.x, scan_box_kanan_depan_camera.point.x);
        float max_x = fmaxf(scan_box_kiri_belakang_camera.point.x, scan_box_kanan_depan_camera.point.x);
        float min_y = fminf(scan_box_kiri_belakang_camera.point.y, scan_box_kanan_depan_camera.point.y);
        float max_y = fmaxf(scan_box_kiri_belakang_camera.point.y, scan_box_kanan_depan_camera.point.y);
        float min_z = fminf(scan_box_kiri_belakang_camera.point.z, scan_box_kanan_depan_camera.point.z);
        float max_z = fmaxf(scan_box_kiri_belakang_camera.point.z, scan_box_kanan_depan_camera.point.z);

        // Crop box filter based on scan box cloud camera
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        pcl::PointCloud<pcl::PointXYZ> points_cropped;
        crop_box_filter.setInputCloud(cloud.makeShared());
        crop_box_filter.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
        crop_box_filter.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
        crop_box_filter.setNegative(false); // Keep points inside the box
        crop_box_filter.filter(points_cropped);

        result_camera = points_cropped.size();
    }

    void callback_tim_routine()
    {
        if (!is_tf_initialized)
        {
            logger.warn("Waiting for transforms to be initialized...");
            return;
        }

        geometry_msgs::msg::PointStamped point_batas_kiri_belakang;
        geometry_msgs::msg::PointStamped point_batas_kanan_depan;

        point_batas_kiri_belakang.point.x = scan_box_x_min;
        point_batas_kiri_belakang.point.y = scan_box_y_max;
        point_batas_kiri_belakang.point.z = scan_box_z_min;
        point_batas_kiri_belakang.header.frame_id = "base_link";
        point_batas_kiri_belakang.header.stamp = this->now();

        point_batas_kanan_depan.point.x = scan_box_x_max;
        point_batas_kanan_depan.point.y = scan_box_y_min;
        point_batas_kanan_depan.point.z = scan_box_z_max;
        point_batas_kanan_depan.header.frame_id = "base_link";
        point_batas_kanan_depan.header.stamp = this->now();

        try
        {
            tf2::doTransform(point_batas_kiri_belakang, scan_box_kiri_belakang_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_batas_kanan_depan, scan_box_kanan_depan_lidar_kanan, tf_base2_lidar_kanan);
            tf2::doTransform(point_batas_kiri_belakang, scan_box_kiri_belakang_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_batas_kanan_depan, scan_box_kanan_depan_lidar_kiri, tf_base2_lidar_kiri);
            tf2::doTransform(point_batas_kiri_belakang, scan_box_kiri_belakang_camera, tf_base2_camera);
            tf2::doTransform(point_batas_kanan_depan, scan_box_kanan_depan_camera, tf_base2_camera);
        }
        catch (const tf2::TransformException &ex)
        {
            logger.error("Transform failed: %s", ex.what());
            return;
        }
        // Publish results
        std_msgs::msg::Float32MultiArray result_msg;
        result_msg.data.push_back(result_lidar_kiri * gain_lidar);
        result_msg.data.push_back(result_lidar_kanan * gain_lidar);
        result_msg.data.push_back(result_camera);
        result_msg.data.push_back(result_toribay_kiri);
        result_msg.data.push_back(result_toribay_kanan);
        result_msg.data.push_back(result_toribay_real_wtf_kiri);
        result_msg.data.push_back(result_toribay_real_wtf_kanan);
        pub_result_all_obstacle->publish(result_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_all_obstacle_filter = std::make_shared<AllObstacleFilter>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_all_obstacle_filter);
    executor.spin();

    return 0;
}