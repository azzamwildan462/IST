// File: src/obstacle_detector.cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_utils/help_logger.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/kdtree/kdtree.h>

using std::placeholders::_1;

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector()
        : Node("obstacle_detector"),
          logger_(),
          octree_(0.05f),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        if (!logger_.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
            return;
        }

        // Declare dynamic ROI parameters (in base_link frame)
        this->declare_parameter("roi_min_x", 0.2);
        this->declare_parameter("roi_max_x", 2.0);
        this->declare_parameter("roi_min_y", -1.0);
        this->declare_parameter("roi_max_y", 1.0);

        // Initialize ROI values
        updateRoiValues(this->get_parameters({"roi_min_x", "roi_max_x", "roi_min_y", "roi_max_y"}));

        // Register callback for dynamic parameter updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ObstacleDetector::onParameterUpdate, this, _1));

        // Load static map once (in map frame)
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/slam/cloud_map", rclcpp::QoS(1),
            std::bind(&ObstacleDetector::mapCallback, this, _1));

        // Subscribe to camera cloud, transform into map frame continuously
        camera_sub_.subscribe(this, "/camera/rs2_cam_main/depth/color/points");
        tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            camera_sub_, tf_buffer_, "map", 10,
            this->get_node_logging_interface(), this->get_node_clock_interface());
        tf_filter_->registerCallback(
            std::bind(&ObstacleDetector::cloudCallback, this, _1));

        // Publisher for obstacle count
        count_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/obstacle_count", 10);

        // Processing timer (20 ms)
        tim_routine_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ObstacleDetector::callbackRoutine, this));

        logger_.info("ObstacleDetector initialized: dynamic ROI parameters enabled");
    }

private:
    // Update internal ROI values
    void updateRoiValues(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params)
        {
            if (p.get_name() == "roi_min_x")
                roi_min_x_ = p.as_double();
            if (p.get_name() == "roi_max_x")
                roi_max_x_ = p.as_double();
            if (p.get_name() == "roi_min_y")
                roi_min_y_ = p.as_double();
            if (p.get_name() == "roi_max_y")
                roi_max_y_ = p.as_double();
        }
        RCLCPP_INFO(this->get_logger(), "Updated ROI: x[%0.2f,%0.2f], y[%0.2f,%0.2f]",
                    roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_);
    }

    // Parameter update callback
    rcl_interfaces::msg::SetParametersResult onParameterUpdate(
        const std::vector<rclcpp::Parameter> &params)
    {
        updateRoiValues(params);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        return result;
    }

    void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        // Load static map into octree (map frame)
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(pcl_pc2, *map_cloud_);
        octree_.setInputCloud(map_cloud_);
        octree_.addPointsFromInputCloud();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        // Store live cloud in map frame
        latest_cloud_ = msg;
    }

    void callbackRoutine()
    {
        if (!latest_cloud_)
            return;

        // Convert ROS cloud to PCL (map frame)
        pcl::PCLPointCloud2 pcl_pc2_map;
        pcl_conversions::toPCL(*latest_cloud_, pcl_pc2_map);
        auto cloud_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromPCLPointCloud2(pcl_pc2_map, *cloud_map);

        // Downsample map-frame cloud
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_map);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        auto cloud_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        vg.filter(*cloud_ds);

        // Transform downsampled cloud into base_link frame for ROI
        sensor_msgs::msg::PointCloud2 ros_ds;
        pcl::PCLPointCloud2 pcl_ds;
        pcl::toPCLPointCloud2(*cloud_ds, pcl_ds);
        pcl_conversions::fromPCL(pcl_ds, ros_ds);
        sensor_msgs::msg::PointCloud2 ros_bl;
        try
        {
            tf_buffer_.transform(ros_ds, ros_bl, "base_link", tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform to base_link failed: %s", ex.what());
            return;
        }

        // Convert to PCL for ROI
        pcl::PCLPointCloud2 pcl_pc2_bl;
        pcl_conversions::toPCL(ros_bl, pcl_pc2_bl);
        auto cloud_bl = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromPCLPointCloud2(pcl_pc2_bl, *cloud_bl);

        // ROI filter in base_link frame
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_bl);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(roi_min_x_, roi_max_x_);
        auto cloud_roi_x = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pass.filter(*cloud_roi_x);
        pass.setInputCloud(cloud_roi_x);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(roi_min_y_, roi_max_y_);
        auto cloud_roi = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pass.filter(*cloud_roi);

        // Transform ROI cloud back to map frame
        pcl::PCLPointCloud2 pcl_roi;
        pcl::toPCLPointCloud2(*cloud_roi, pcl_roi);
        sensor_msgs::msg::PointCloud2 ros_roi;
        pcl_conversions::fromPCL(pcl_roi, ros_roi);
        sensor_msgs::msg::PointCloud2 ros_roi_map;
        try
        {
            tf_buffer_.transform(ros_roi, ros_roi_map, "map", tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform ROI to map frame failed: %s", ex.what());
            return;
        }

        // Convert back to PCL for change detection
        pcl::PCLPointCloud2 pcl_pc2_roi_map;
        pcl_conversions::toPCL(ros_roi_map, pcl_pc2_roi_map);
        auto cloud_roi_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromPCLPointCloud2(pcl_pc2_roi_map, *cloud_roi_map);

        // Detect changes vs static map octree
        octree_.switchBuffers();
        octree_.setInputCloud(cloud_roi_map);
        octree_.addPointsFromInputCloud();
        std::vector<int> new_idxs;
        octree_.getPointIndicesFromNewVoxels(new_idxs);

        // Publish obstacle count
        std_msgs::msg::Int32 count_msg;
        count_msg.data = static_cast<int>(new_idxs.size());
        count_pub_->publish(count_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> camera_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_pub_;
    rclcpp::TimerBase::SharedPtr tim_routine_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    HelpLogger logger_;
    double roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetector>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
