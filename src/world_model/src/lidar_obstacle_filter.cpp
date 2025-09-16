#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/opencv.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/float32.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/transforms.hpp"

#include <pcl/common/transforms.h>
#include <Eigen/Geometry> // for Affine3f, AngleAxisf

#include <yaml-cpp/yaml.h>

class LidarObstacleFilter : public rclcpp::Node
{
public:
    HelpLogger logger;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_obs_find_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_icp_score_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_camera_filtered_pcl_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_icp_estimate;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_filtered;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_camera_pcl;

    rclcpp::CallbackGroup::SharedPtr callback_group_lidar;
    rclcpp::CallbackGroup::SharedPtr callback_group_camera;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::shared_ptr<tf2_ros::Buffer> tf_lidar_map_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_lidar_map_listener;
    laser_geometry::LaserProjection projector_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_voxelized_;

    geometry_msgs::msg::TransformStamped tf_camera_map;
    std::unique_ptr<tf2_ros::Buffer> tf_camera_map_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_camera_map_listener;

    bool tf_is_initialized = false;

    std::string pgm_path_;
    std::string yaml_path_;
    double resolution_, origin_x_, origin_y_;
    double scan_min_y_, scan_max_y_, obstacle_tolerance_;
    double scan_range_;

    float final_pose_x, final_pose_y, final_pose_theta;
    float odom_x, odom_y, odom_theta;
    float obs_find = 0;

    float icp_max_range = 10.0;

    pcl::PointXYZ point_batas_kiri_bawah;  // Kirinya mobil
    pcl::PointXYZ point_batas_kanan_bawah; // Kanannya mobil
    pcl::PointXYZ point_batas_kiri_atas;
    pcl::PointXYZ point_batas_kanan_atas;

    float dx_icp;
    float dy_icp;
    float dth_icp;

    float icp_estimate_x = 0;
    float icp_estimate_y = 0;
    float icp_estimate_th = 0;

    LidarObstacleFilter()
        : Node("lidar_obstacle_filter")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Logger failed to initialize");
            rclcpp::shutdown();
        }

        // Declare parameters
        declare_parameter<std::string>("pgm_path", "map.pgm");
        declare_parameter<std::string>("yaml_path", "");
        declare_parameter<double>("resolution", 0.05);
        declare_parameter<double>("origin_x", 0.0);
        declare_parameter<double>("origin_y", 0.0);
        declare_parameter<double>("scan_min_y", -0.5);
        declare_parameter<double>("scan_max_y", 0.5);
        declare_parameter<double>("scan_range", 5.0);
        declare_parameter<double>("obstacle_error_tolerance", 0.4);
        declare_parameter<double>("icp_max_range", 10.0);

        get_parameter("pgm_path", pgm_path_);
        get_parameter("yaml_path", yaml_path_);
        get_parameter("resolution", resolution_);
        get_parameter("origin_x", origin_x_);
        get_parameter("origin_y", origin_y_);
        get_parameter("scan_min_y", scan_min_y_);
        get_parameter("scan_max_y", scan_max_y_);
        get_parameter("scan_range", scan_range_);
        get_parameter("obstacle_error_tolerance", obstacle_tolerance_);
        get_parameter("icp_max_range", icp_max_range);

        if (yaml_path_ != "")
        {
            try
            {
                YAML::Node config = YAML::LoadFile(yaml_path_);
                if (config["resolution"])
                    resolution_ = config["resolution"].as<double>();
                if (config["origin"])
                {
                    auto origin = config["origin"].as<std::vector<double>>();
                    if (origin.size() >= 2)
                    {
                        origin_x_ = origin[0];
                        origin_y_ = origin[1];
                    }
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
            }
        }

        tf_camera_map_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_camera_map_listener = std::make_unique<tf2_ros::TransformListener>(*tf_camera_map_buffer);

        // while (!tf_is_initialized)
        // {
        //     sleep(1);
        //     try
        //     {
        //         tf_camera_map = tf_camera_map_buffer->lookupTransform("map", "camera_depth_optical_frame", tf2::TimePointZero);
        //         tf_is_initialized = true;
        //     }
        //     catch (tf2::TransformException &ex)
        //     {
        //         logger.error("Lidar Obstacle Filter tunggu");
        //         tf_is_initialized = false;
        //     }
        // }

        pub_obs_find_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_obstacle_filter/obs_find", 1);
        pub_icp_score_ = this->create_publisher<std_msgs::msg::Float32>("/lidar_obstacle_filter/icp_score", 1);
        pub_camera_filtered_pcl_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_obstacle_filter/camera_filtered_pcl", 1);
        pub_icp_estimate = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_obstacle_filter/icp_estimate", 10);

        tf_lidar_map_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10));
        tf_lidar_map_listener = std::make_shared<tf2_ros::TransformListener>(*tf_lidar_map_buffer);

        auto sub_lidar_opts = rclcpp::SubscriptionOptions();
        sub_lidar_opts.callback_group = callback_group_lidar;

        auto sub_camera_opts = rclcpp::SubscriptionOptions();
        sub_camera_opts.callback_group = callback_group_camera;

        // Register callback for dynamic parameter updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&LidarObstacleFilter::callback_params_change, this, std::placeholders::_1));

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1,
                                                                           std::bind(&LidarObstacleFilter::callback_scan, this, std::placeholders::_1), sub_lidar_opts);
        sub_odom_filtered = this->create_subscription<nav_msgs::msg::Odometry>("/slam/odometry/filtered", 1,
                                                                               std::bind(&LidarObstacleFilter::callback_sub_odom_filtered, this, std::placeholders::_1));
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                      std::bind(&LidarObstacleFilter::callback_sub_odom, this, std::placeholders::_1));
        // sub_camera_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/camera/rs2_cam_main/depth/color/points", 1,
        //     std::bind(&LidarObstacleFilter::callback_sub_camera_pcl, this, std::placeholders::_1), sub_camera_opts);

        load_map_pointcloud();
        logger.info("LidarObstacleFilter node initialized %.2f %.2f %.2f", resolution_, origin_x_, origin_y_);
    }

    // Update internal ROI values
    void update_params(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "scan_min_y")
                scan_min_y_ = p.as_double();
            if (p.get_name() == "scan_max_y")
                scan_max_y_ = p.as_double();
            if (p.get_name() == "obstacle_error_tolerance")
                obstacle_tolerance_ = p.as_double();
            if (p.get_name() == "scan_range")
                scan_range_ = p.as_double();
        }
        RCLCPP_INFO(this->get_logger(), "Updated params: x[%0.2f,%0.2f], y[%0.2f,%0.2f]",
                    scan_min_y_, scan_max_y_, obstacle_tolerance_, scan_range_);
    }

    // Parameter update callback
    rcl_interfaces::msg::SetParametersResult callback_params_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        update_params(params);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        return result;
    }

    void callback_sub_camera_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!tf_is_initialized)
        {
            logger.error("TF is not initialized yet");
            return;
        }

        static pcl::PointCloud<pcl::PointXYZ> points_camera_;
        static pcl::PointCloud<pcl::PointXYZ> points_lidar2map_;
        static pcl::PointCloud<pcl::PointXYZ> points_camera_filtered_;

        pcl::fromROSMsg(*msg, points_camera_);

        if (points_camera_.empty())
        {
            points_lidar2map_.clear();
            points_camera_filtered_.clear();
        }
        else
        {
            pcl_ros::transformPointCloud(points_camera_, points_lidar2map_, tf_camera_map);

            pcl::VoxelGrid<pcl::PointXYZ> voxel_;
            voxel_.setInputCloud(points_lidar2map_.makeShared());
            voxel_.filter(points_lidar2map_);

            pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>());
            hull->points.push_back(point_batas_kiri_bawah);
            hull->points.push_back(point_batas_kanan_bawah);
            hull->points.push_back(point_batas_kanan_atas);
            hull->points.push_back(point_batas_kiri_atas);

            std::vector<pcl::Vertices> polygons(1);
            polygons[0].vertices = {0, 1, 2, 3};

            pcl::CropHull<pcl::PointXYZ> crop_hull;
            crop_hull.setInputCloud(points_lidar2map_.makeShared());
            crop_hull.setHullCloud(hull);
            crop_hull.setHullIndices(polygons);
            crop_hull.setDim(2);
            crop_hull.setCropOutside(true);
            crop_hull.filter(points_camera_filtered_);

            sensor_msgs::msg::PointCloud2 filtered_msg;
            pcl::toROSMsg(points_camera_filtered_, filtered_msg);
            filtered_msg.header = msg->header;
            pub_camera_filtered_pcl_->publish(filtered_msg);
        }
    }

    void callback_sub_odom_filtered(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        final_pose_x = msg->pose.pose.position.x;
        final_pose_y = msg->pose.pose.position.y;

        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        final_pose_theta = yaw;

        // logger.info("Current pose: x=%.2f, y=%.2f, theta=%.2f", final_pose_x, final_pose_y, final_pose_theta);
    }
    void callback_sub_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_x = msg->pose.pose.position.x;
        odom_y = msg->pose.pose.position.y;

        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        odom_theta = yaw;

        // logger.info("Current pose: x=%.2f, y=%.2f, theta=%.2f", odom_x, odom_y, odom_theta);
    }

    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        static uint16_t counter_pembagi = 1;
        if (counter_pembagi++ <= 4)
        {
            (void)msg;
            return;
        }
        counter_pembagi = 1;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        try
        {
            projector_.transformLaserScanToPointCloud("base_link", *msg, cloud_msg, *tf_lidar_map_buffer);
        }
        catch (tf2::TransformException &ex)
        {
            // logger.warn("TF error: %s", ex.what());
            return;
        }

        // projector_.projectLaser(*msg, cloud_msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *scan_cloud);

        /* Downsampling */
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(scan_cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // Adjust leaf size as needed
        voxel_filter.filter(*scan_cloud);

        // logger.info("Scan cloud size after downsampling: %zu points", scan_cloud->points.size());

        if (scan_cloud->empty())
        {
            logger.warn("Scan cloud is empty, skipping processing");
            return;
        }

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        float cos_t = std::cos(final_pose_theta);
        float sin_t = std::sin(final_pose_theta);
        // rotation about Z
        T(0, 0) = cos_t;
        T(0, 1) = -sin_t;
        T(1, 0) = sin_t;
        T(1, 1) = cos_t;
        // translation
        T(0, 3) = final_pose_x;
        T(1, 3) = final_pose_y;
        // leave T(2,3)=0 (no z offset)

        // 2) Apply it
        pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*scan_cloud, *world_cloud, T);

        // for (const auto &pt : world_cloud->points)
        // {
        //     logger.info("(%.2f %.2f) Point: x=%.2f, y=%.2f", final_pose_x, final_pose_y, pt.x, pt.y);
        // }

        // logger.info("%.2f %.2f %.2f || %.2f %.2f %.2f",
        //             final_pose_x, final_pose_y, final_pose_theta,
        //             odom_x, odom_y, odom_theta);

        // pcl::PointXYZ p_in;
        // p_in.x = odom_x;
        // p_in.y = odom_y;
        // p_in.z = 0.0f; // Assuming z is 0 for 2D lidar

        // float c = std::cos(final_pose_theta);
        // float s = std::sin(final_pose_theta);

        // pcl::PointXYZ p_out;
        // p_out.x = c * p_in.x - s * p_in.y + final_pose_x;
        // p_out.y = s * p_in.x + c * p_in.y + final_pose_y;
        // p_out.z = p_in.z; // unchanged

        // float theta_transformed = final_pose_theta + odom_theta;
        // while (theta_transformed > M_PI)
        //     theta_transformed -= 2 * M_PI;
        // while (theta_transformed < -M_PI)
        //     theta_transformed += 2 * M_PI;

        /* Membuat scan area */
        point_batas_kiri_bawah.x = final_pose_x + scan_max_y_ * cos(final_pose_theta + M_PI / 2);
        point_batas_kiri_bawah.y = final_pose_y + scan_max_y_ * sin(final_pose_theta + M_PI / 2);
        point_batas_kanan_bawah.x = final_pose_x + scan_min_y_ * cos(final_pose_theta + M_PI / 2);
        point_batas_kanan_bawah.y = final_pose_y + scan_min_y_ * sin(final_pose_theta + M_PI / 2);

        point_batas_kiri_atas.x = point_batas_kiri_bawah.x + scan_range_ * cos(final_pose_theta);
        point_batas_kiri_atas.y = point_batas_kiri_bawah.y + scan_range_ * sin(final_pose_theta);
        point_batas_kanan_atas.x = point_batas_kanan_bawah.x + scan_range_ * cos(final_pose_theta);
        point_batas_kanan_atas.y = point_batas_kanan_bawah.y + scan_range_ * sin(final_pose_theta);

        // logger.info("Scan area points: "
        //             "Kiri Bawah(%.2f, %.2f), "
        //             "Kanan Bawah(%.2f, %.2f), "
        //             "Kanan Atas(%.2f, %.2f), "
        //             "Kiri Atas(%.2f, %.2f)",
        //             point_batas_kiri_bawah.x, point_batas_kiri_bawah.y,
        //             point_batas_kanan_bawah.x, point_batas_kanan_bawah.y,
        //             point_batas_kanan_atas.x, point_batas_kanan_atas.y,
        //             point_batas_kiri_atas.x, point_batas_kiri_atas.y);

        pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>());
        hull->points.push_back(point_batas_kiri_bawah);
        hull->points.push_back(point_batas_kanan_bawah);
        hull->points.push_back(point_batas_kanan_atas);
        hull->points.push_back(point_batas_kiri_atas);

        std::vector<pcl::Vertices> polygons(1);
        polygons[0].vertices = {0, 1, 2, 3};

        /* Crop pcl lidar berdasarkan area scan */
        pcl::CropHull<pcl::PointXYZ> crop_hull;
        crop_hull.setInputCloud(world_cloud);
        crop_hull.setHullCloud(hull);
        crop_hull.setHullIndices(polygons);
        crop_hull.setDim(2);
        crop_hull.setCropOutside(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop_hull.filter(*filtered_cloud);

        // logger.info("Filtered cloud size: %zu points", filtered_cloud->points.size());

        /* Filter obstacle dari map */
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_obs_from_map(new pcl::PointCloud<pcl::PointXYZ>());

        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);
        for (const auto &pt : filtered_cloud->points)
        {
            // int kdtree_search = kdtree_map_.nearestKSearch(pt, 1, indices, sqr_distances);
            if (kdtree_map_.nearestKSearch(pt, 1, indices, sqr_distances) > 0)
            {
                float dist_sq = sqr_distances[0];
                // logger.info("Processing point: x=%.2f, y=%.2f %.2f", pt.x, pt.y, dist_sq);
                if (dist_sq > obstacle_tolerance_)
                    filtered_obs_from_map->points.push_back(pt);
            }
        }

        // logger.info("Filtered obstacles from map size: %zu points", filtered_obs_from_map->points.size());

        float obs_scan_r_decimal = 1 / scan_range_;
        float obs_find_local = 0;
        for (const auto &pt : filtered_obs_from_map->points)
        {
            // logger.info("Point: x=%.2f, y=%.2f", pt.x, pt.y);
            float dx = pt.x - final_pose_x;
            float dy = pt.y - final_pose_y;
            float distance = sqrtf(dx * dx + dy * dy);

            obs_find_local = obs_scan_r_decimal * (scan_range_ - distance);
        }

        /* ICP */
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(world_cloud);
        icp.setInputTarget(map_cloud_voxelized_);
        icp.setMaxCorrespondenceDistance(0.5f); // Secara teori ini 10x dari voxel grid
        icp.setMaximumIterations(30);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-4);

        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud);

        float icp_score = icp.getFitnessScore();

        /**
         * Get final matrix transformation, this matrix is used to transfrom measured points to reference points,
         * so we use this transformation to transform our robot pose (from odometry) into new pose based on ICP
         */
        Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

        /* Get Robot's translation and rotation based on ICP */
        Eigen::Vector4f point(final_pose_x, final_pose_y, 0, 1);
        Eigen::Vector4f transformed_point = transformation_matrix * point;
        dth_icp = atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));
        dx_icp = transformed_point[0] - final_pose_x;
        dy_icp = transformed_point[1] - final_pose_y;
        if (dth_icp > M_PI)
            dth_icp -= 2 * M_PI;
        else if (dth_icp < -M_PI)
            dth_icp += 2 * M_PI;

        icp_estimate_x = final_pose_x * 0.95 + (final_pose_x + dx_icp) * 0.05;
        icp_estimate_y = final_pose_y * 0.95 + (final_pose_y + dy_icp) * 0.05;
        icp_estimate_th = final_pose_theta * 0.975 + (final_pose_theta + dth_icp) * 0.025;

        tf2::Quaternion q;
        q.setRPY(0, 0, icp_estimate_th);

        nav_msgs::msg::Odometry msg_odom;
        msg_odom.header.stamp = msg->header.stamp;
        msg_odom.header.frame_id = "odom_icp";
        msg_odom.child_frame_id = "base_link";
        msg_odom.pose.pose.position.x = icp_estimate_x;
        msg_odom.pose.pose.position.y = icp_estimate_y;
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
        msg_odom.twist.twist.linear.x = dx_icp;
        msg_odom.twist.twist.linear.y = dy_icp;
        msg_odom.twist.twist.angular.z = dth_icp;
        msg_odom.twist.covariance[0] = 1e-2;
        msg_odom.twist.covariance[7] = 1e-2;
        msg_odom.twist.covariance[14] = 1e6;
        msg_odom.twist.covariance[21] = 1e6;
        msg_odom.twist.covariance[28] = 1e6;
        msg_odom.twist.covariance[35] = 1e-2;
        pub_icp_estimate->publish(msg_odom);

        std_msgs::msg::Float32 msg_obs_find;
        msg_obs_find.data = obs_find_local;
        pub_obs_find_->publish(msg_obs_find);

        std_msgs::msg::Float32 msg_icp_score;
        msg_icp_score.data = icp_score;
        pub_icp_score_->publish(msg_icp_score);
    }

    bool isInsideConvexQuad(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &poly)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            const auto &a = poly[i];
            const auto &b = poly[(i + 1) % 4];
            Eigen::Vector2f edge = b - a;
            Eigen::Vector2f to_point = pt - a;
            float cross = edge.x() * to_point.y() - edge.y() * to_point.x();
            if (cross < 0)
                return false; // Outside for CCW
        }
        return true;
    }

    void load_map_pointcloud()
    {
        logger.info("Loading map from PGM file: %s", pgm_path_.c_str());
        cv::Mat map = cv::imread(pgm_path_, cv::IMREAD_UNCHANGED);
        if (map.empty() || map.type() != CV_8UC1)
        {
            logger.error("Failed to load PGM map or invalid format");
            return;
        }

        map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        logger.info("%d %d", map.rows, map.cols);

        for (int y = 0; y < map.rows; ++y)
        {
            for (int x = 0; x < map.cols; ++x)
            {
                uint8_t pixel = map.at<uchar>(y, x);
                if (pixel < 100)
                { // occupied
                    // logger.info("Adding point at (%d, %d)", x, y);
                    float wx = origin_x_ + x * resolution_;
                    float wy = origin_y_ + (map.rows - y - 1) * resolution_;
                    map_cloud_->points.emplace_back(wx, wy, 0.0f);
                }
            }
        }

        map_cloud_->width = map_cloud_->points.size();
        map_cloud_->height = 1;
        map_cloud_->is_dense = true;

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        map_cloud_voxelized_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        voxel_filter.setInputCloud(map_cloud_);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // Adjust voxel size as needed
        voxel_filter.filter(*map_cloud_voxelized_);
        map_cloud_ = map_cloud_voxelized_;

        logger.info("Map loaded with %d points", map_cloud_->points.size());

        kdtree_map_ = pcl::KdTreeFLANN<pcl::PointXYZ>();
        kdtree_map_.setInputCloud(map_cloud_voxelized_);

        logger.info("Loaded map with %s occupied points", std::to_string(map_cloud_voxelized_->points.size()).c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarObstacleFilter>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
