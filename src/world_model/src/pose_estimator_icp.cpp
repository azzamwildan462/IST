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
#include <tf2_eigen/tf2_eigen.hpp> // not just tf2_geometry_msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Transform.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "pcl/io/pcd_io.h"
#include "std_msgs/msg/u_int8.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include "pcl_ros/transforms.hpp"

using namespace std::chrono_literals;

class PoseEstimatorICP : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_encoder_meter;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_encoder;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gyro;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_offset;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_fb_transmission;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_CAN_eps_encoder;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_filtered;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_laserscan;

    //----TransformBroadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    std::unique_ptr<tf2_ros::Buffer> tf_lidar_base_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_lidar_base_listener;
    geometry_msgs::msg::TransformStamped tf_lidar_base;

    HelpLogger logger;

    // Configs (static)
    // =======================================================
    float encoder_to_meter = 1.0;
    int timer_period = 20; // ms
    std::string lidar_frame_id = "lidar1_link";
    std::string lidar_topic = "/scan";
    std::string map_file = "/path/to/your/map.pcd"; // Path to the map file

    float icp_minimal_travel = 0.5;              // minimal travel distance to perform ICP
    float icp_max_correspondence_distance = 0.1; // max correspondence distance for ICP
    float icp_max_translation_distance = 0.1;    // max translation distance for ICP
    float icp_max_linear_correction = 0.5;       // max linear correction for ICP
    float icp_max_angular_correction = 0.5;      // max angular correction for ICP
    float icp_gain = 0.005;                      // gain factor for ICP
    int icp_max_iterations = 50;                 // max iterations for ICP
    float icp_transformation_epsilon = 1e-8;     // transformation epsilon for ICP
    float threshold_icp_score = 20.0;            // threshold for ICP score to consider a valid alignment

    uint16_t encoder[2] = {0, 0};
    uint16_t prev_encoder[2] = {0, 0};
    float gyro = 0;
    float prev_gyro = 0;

    float final_pose_xyo[3] = {0, 0, 0};
    float prev_final_pose_xyo[3] = {0, 0, 0};
    float final_pose_xyo_before_icp[3] = {0, 0, 0};
    float final_vel_dxdydo[3] = {0, 0, 0};
    float d_icp_dxdydo[3] = {0, 0, 0}; // delta dx, dy, do (delta orientation) from ICP

    int32_t sensor_left_encoder = 0;
    int32_t sensor_right_encoder = 0;

    rclcpp::Time last_time_gyro_update;
    rclcpp::Time current_time;

    uint8_t fb_transmission = 0;

    float steering_position = 0;
    float offset_sudut_steering = 0;
    float d_gyro_steering = 0;
    float wheelbase = 1.00;

    int gyro_type = 0;

    float pose_filtered[3] = {0, 0, 0};

    float imu_z_velocity = 0;

    float delta_theta_filtered = 0;  // error theta sekarang terhadap filtered
    float delta_vTheta_filtered = 0; // error theta sekarang terhadap filtered

    bool is_gyro_recvd = false;

    bool tf_is_initialized = false;

    pcl::PointCloud<pcl::PointXYZ> points_lidar;
    pcl::PointCloud<pcl::PointXYZ> points_lidar2map;
    pcl::PointCloud<pcl::PointXYZ> points_lidar2base;
    pcl::PointCloud<pcl::PointXYZ> points_lidar2base_aligned;
    pcl::PointCloud<pcl::PointXYZ> map_points;

    float icp_score = 999.0f; // Score of the ICP alignment

    //=======================================================
    // Vars
    // =========================================================
    int16_t error_code = 0;

    PoseEstimatorICP()
        : Node("PoseEstimatorICP")
    {
        this->declare_parameter("encoder_to_meter", 1.0);
        this->get_parameter("encoder_to_meter", encoder_to_meter);

        this->declare_parameter("offset_sudut_steering", 0.0);
        this->get_parameter("offset_sudut_steering", offset_sudut_steering);

        this->declare_parameter("gyro_type", 0);
        this->get_parameter("gyro_type", gyro_type);

        this->declare_parameter("wheelbase", 1.0);
        this->get_parameter("wheelbase", wheelbase);

        this->declare_parameter("timer_period", 20);
        this->get_parameter("timer_period", timer_period);

        this->declare_parameter("lidar_frame_id", "lidar1_link");
        this->get_parameter("lidar_frame_id", lidar_frame_id);

        this->declare_parameter("lidar_topic", "/scan");
        this->get_parameter("lidar_topic", lidar_topic);

        this->declare_parameter("icp_minimal_travel", 0.5);
        this->get_parameter("icp_minimal_travel", icp_minimal_travel);

        this->declare_parameter("icp_max_correspondence_distance", 0.1);
        this->get_parameter("icp_max_correspondence_distance", icp_max_correspondence_distance);

        this->declare_parameter("icp_max_translation_distance", 0.1);
        this->get_parameter("icp_max_translation_distance", icp_max_translation_distance);

        this->declare_parameter("icp_max_linear_correction", 0.5);
        this->get_parameter("icp_max_linear_correction", icp_max_linear_correction);

        this->declare_parameter("icp_max_angular_correction", 0.5);
        this->get_parameter("icp_max_angular_correction", icp_max_angular_correction);

        this->declare_parameter("icp_gain", 0.005);
        this->get_parameter("icp_gain", icp_gain);

        this->declare_parameter("map_file", "/path/to/your/map.pcd");
        this->get_parameter("map_file", map_file);

        this->declare_parameter("icp_max_iterations", 50);
        this->get_parameter("icp_max_iterations", icp_max_iterations);

        this->declare_parameter("icp_transformation_epsilon", 1e-8);
        this->get_parameter("icp_transformation_epsilon", icp_transformation_epsilon);

        this->declare_parameter("threshold_icp_score", 20.0);
        this->get_parameter("threshold_icp_score", threshold_icp_score);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (!load_map_points(map_file))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map points");
            rclcpp::shutdown();
        }

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_lidar_base_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_lidar_base_listener = std::make_unique<tf2_ros::TransformListener>(*tf_lidar_base_buffer);

        while (!tf_is_initialized)
        {
            rclcpp::sleep_for(1s);
            try
            {
                tf_lidar_base = tf_lidar_base_buffer->lookupTransform("base_link", lidar_frame_id, tf2::TimePointZero);
                tf_is_initialized = true;
            }
            catch (tf2::TransformException &ex)
            {
                logger.error("%s", ex.what());
                tf_is_initialized = false;
            }
        }

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(timer_period), std::bind(&PoseEstimatorICP::callback_tim_50hz, this));

        //----Publisher
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("error_code", 10);
        pub_encoder_meter = this->create_publisher<std_msgs::msg::Float32>("encoder_meter", 1);

        sub_encoder = this->create_subscription<std_msgs::msg::Int32>(
            "/can/encoder", 1, std::bind(&PoseEstimatorICP::callback_sub_encoder, this, std::placeholders::_1));
        sub_gyro = this->create_subscription<sensor_msgs::msg::Imu>(
            "/hardware/imu", 1, std::bind(&PoseEstimatorICP::callback_sub_gyro, this, std::placeholders::_1));
        sub_pose_offset = this->create_subscription<nav_msgs::msg::Odometry>(
            "/master/pose_offset", 1, std::bind(&PoseEstimatorICP::callback_sub_pose_offset, this, std::placeholders::_1));
        sub_fb_transmission = this->create_subscription<std_msgs::msg::UInt8>(
            "/can/fb_transmission", 1, std::bind(&PoseEstimatorICP::callback_sub_fb_transmission, this, std::placeholders::_1));
        sub_odometry_filtered = this->create_subscription<nav_msgs::msg::Odometry>(
            "/slam/odometry/filtered", 1, std::bind(&PoseEstimatorICP::callback_sub_odom_filtered, this, std::placeholders::_1));
        sub_lidar_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 1, std::bind(&PoseEstimatorICP::callback_sub_lidar_laserscan, this, std::placeholders::_1));

        logger.info("PoseEstimatorICP initialized");
    }

    bool load_map_points(const std::string &map_file)
    {
        // Load map points from a PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file, map_points) == -1)
        {
            logger.error("Failed to load map points from %s", map_file.c_str());
            return false;
        }
        logger.info("Loaded %zu map points from %s", map_points.size(), map_file.c_str());

        return true;
    }

    void callback_sub_lidar_laserscan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (sqrtf(final_pose_xyo[0] * final_pose_xyo[0] + final_pose_xyo[1] * final_pose_xyo[1]) < icp_minimal_travel)
        {
            // logger.info("Skipping ICP due to minimal travel distance");
            memcpy(prev_final_pose_xyo, final_pose_xyo, sizeof(final_pose_xyo));
            return;
        }
        memcpy(prev_final_pose_xyo, final_pose_xyo, sizeof(final_pose_xyo));

        // Convert LaserScan to PointCloud
        pcl::PointCloud<pcl::PointXYZ> points_lidar2base_temp;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max)
                continue;

            float angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0.0; // Assuming 2D laser scan
            points_lidar2base_temp.push_back(point);
        }

        // Transform PointCloud to base_link frame
        geometry_msgs::msg::TransformStamped tf_msg;
        try
        {
            tf_msg = tf_lidar_base_buffer->lookupTransform(
                "base_link",    // target_frame
                lidar_frame_id, // source_frame
                tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }
        Eigen::Affine3d transform;
        tf2::fromMsg(tf_msg.transform, transform);
        Eigen::Affine3f transform_f = transform.cast<float>();

        pcl::transformPointCloud(points_lidar2base_temp, points_lidar2base, transform_f);

        // Perform ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(points_lidar2base.makeShared());
        icp.setInputTarget(map_points.makeShared());
        icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
        icp.setMaximumIterations(icp_max_iterations);
        icp.setTransformationEpsilon(icp_transformation_epsilon);
        icp.align(points_lidar2base_aligned);

        icp_score = icp.getFitnessScore();

        std_msgs::msg::Float32 msg_icp_score;
        msg_icp_score.data = icp_score;
        pub_encoder_meter->publish(msg_icp_score);

        Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

        Eigen::Vector4f pose_now(prev_final_pose_xyo[0], prev_final_pose_xyo[1], 0.0, 1.0);
        Eigen::Vector4f pose_transformed = transformation_matrix * pose_now;
        d_icp_dxdydo[0] = pose_transformed[0] - prev_final_pose_xyo[0];
        d_icp_dxdydo[1] = pose_transformed[1] - prev_final_pose_xyo[1];
        d_icp_dxdydo[2] = atan2f(transformation_matrix(1, 0), transformation_matrix(0, 0));
        while (d_icp_dxdydo[2] > M_PI)
            d_icp_dxdydo[2] -= 2 * M_PI;
        while (d_icp_dxdydo[2] < -M_PI)
            d_icp_dxdydo[2] += 2 * M_PI;
    }

    void callback_sub_odom_filtered(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose_filtered[0] = msg->pose.pose.position.x;
        pose_filtered[1] = msg->pose.pose.position.y;

        // get_angle from quartenion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        pose_filtered[2] = yaw;
        delta_theta_filtered = pose_filtered[2] - final_pose_xyo[2];

        while (delta_theta_filtered > M_PI)
            delta_theta_filtered -= 2 * M_PI;
        while (delta_theta_filtered < -M_PI)
            delta_theta_filtered += 2 * M_PI;

        float v_yaw = msg->twist.twist.angular.z;
        delta_vTheta_filtered = v_yaw - final_vel_dxdydo[2];

        while (delta_vTheta_filtered > M_PI)
            delta_vTheta_filtered -= 2 * M_PI;
        while (delta_vTheta_filtered < -M_PI)
            delta_vTheta_filtered += 2 * M_PI;
    }

    void callback_sub_CAN_eps_encoder(const std_msgs::msg::Float32::SharedPtr msg)
    {
        steering_position = msg->data + offset_sudut_steering;
    }

    void callback_sub_fb_transmission(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        fb_transmission = msg->data;
    }

    void callback_sub_pose_offset(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        final_pose_xyo[0] = msg->pose.pose.position.x;
        final_pose_xyo[1] = msg->pose.pose.position.y;

        // get_angle from quartenion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        final_pose_xyo[2] = yaw;
    }

    void callback_sub_encoder(const std_msgs::msg::Int32::SharedPtr msg)
    {
        sensor_left_encoder = msg->data * 0.5;
        sensor_right_encoder = msg->data * 0.5;

        std_msgs::msg::Float32 msg_encoder_meter;

        if (fb_transmission == 5)
            msg_encoder_meter.data = (sensor_left_encoder + sensor_right_encoder) / 2.0 * encoder_to_meter * 1000 / timer_period * -1;
        else
            msg_encoder_meter.data = (sensor_left_encoder + sensor_right_encoder) / 2.0 * encoder_to_meter * 1000 / timer_period;

        pub_encoder_meter->publish(msg_encoder_meter);
    }

    void callback_sub_gyro(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // get_angle from quartenion
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        imu_z_velocity = msg->angular_velocity.z;

        // logger.info("%.2f %.2f %.2f %f %f %f", roll, pitch, yaw, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        gyro = yaw;
        last_time_gyro_update = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        static uint16_t cntr_awal_recvd = 0;
        if (cntr_awal_recvd++ > 50)
        {
            is_gyro_recvd = true;
            cntr_awal_recvd = 50;
        }
    }

    void callback_tim_50hz()
    {
        static rclcpp::Time time_old = this->now();
        static rclcpp::Time time_now = this->now();
        time_old = time_now;
        time_now = this->now();

        static const float dt = timer_period / 1000.0; // Convert ms to seconds

        current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        //======================================================

        float d_gyro = gyro - prev_gyro;

        rclcpp::Duration dt_gyro = current_time - last_time_gyro_update;
        static rclcpp::Duration prev_dt_gyro = dt_gyro;
        if ((prev_dt_gyro.seconds() > 0.5 && dt_gyro.seconds() <= 0.5) || !is_gyro_recvd)
        {
            logger.warn("Gyro restarted");
            d_gyro = 0;
        }
        prev_dt_gyro = dt_gyro;

        memcpy(prev_encoder, encoder, sizeof(prev_encoder));
        prev_gyro = gyro;

        while (d_gyro > M_PI)
            d_gyro -= 2 * M_PI;
        while (d_gyro < -M_PI)
            d_gyro += 2 * M_PI;

        d_gyro_steering = (sensor_left_encoder + sensor_right_encoder) / 2.0 * encoder_to_meter * tanf(steering_position) / wheelbase;

        final_vel_dxdydo[0] = (sensor_left_encoder + sensor_right_encoder) / 2.0 * cosf(final_pose_xyo[2]) * encoder_to_meter / dt;
        final_vel_dxdydo[1] = (sensor_left_encoder + sensor_right_encoder) / 2.0 * sinf(final_pose_xyo[2]) * encoder_to_meter / dt;

        if (gyro_type == 0)
            final_vel_dxdydo[2] = d_gyro;
        else if (gyro_type == 1)
            final_vel_dxdydo[2] = d_gyro_steering;

        while (final_vel_dxdydo[2] > M_PI)
            final_vel_dxdydo[2] -= 2 * M_PI;
        while (final_vel_dxdydo[2] < -M_PI)
            final_vel_dxdydo[2] += 2 * M_PI;

        final_pose_xyo[0] += final_vel_dxdydo[0] * dt;
        final_pose_xyo[1] += final_vel_dxdydo[1] * dt;
        final_pose_xyo[2] += final_vel_dxdydo[2];

        if (icp_score < threshold_icp_score)
        {
            float final_pose_corrected[3] = {0, 0, 0};
            final_pose_corrected[0] = d_icp_dxdydo[0] + final_pose_xyo[0];
            final_pose_corrected[1] = d_icp_dxdydo[1] + final_pose_xyo[1];
            final_pose_corrected[2] = d_icp_dxdydo[2] + final_pose_xyo[2];

            while (final_pose_corrected[2] > M_PI)
                final_pose_corrected[2] -= 2 * M_PI;
            while (final_pose_corrected[2] < -M_PI)
                final_pose_corrected[2] += 2 * M_PI;

            final_pose_xyo[0] = final_pose_corrected[0] * icp_gain + final_pose_xyo[0] * (1 - icp_gain);
            final_pose_xyo[1] = final_pose_corrected[1] * icp_gain + final_pose_xyo[1] * (1 - icp_gain);
            final_pose_xyo[2] = final_pose_corrected[2] * icp_gain + final_pose_xyo[2] * (1 - icp_gain);
        }

        while (final_pose_xyo[2] > M_PI)
            final_pose_xyo[2] -= 2 * M_PI;
        while (final_pose_xyo[2] < -M_PI)
            final_pose_xyo[2] += 2 * M_PI;

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
        msg_odom.twist.twist.angular.z = final_vel_dxdydo[2] / dt;
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_icp = std::make_shared<PoseEstimatorICP>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_icp);
    executor.spin();

    return 0;
}