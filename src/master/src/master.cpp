#include "master/master.hpp"
#include "rclcpp/rclcpp.hpp"

Master::Master()
    : Node("master")
{
    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        rclcpp::shutdown();
    }

    pub_initialpose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam/initialpose", 10);
    pub_global_fsm = this->create_publisher<std_msgs::msg::Int16>("/master/global_fsm", 1);
    pub_local_fsm = this->create_publisher<std_msgs::msg::Int16>("/master/local_fsm", 1);
    pub_to_ui = this->create_publisher<std_msgs::msg::Float32MultiArray>("/master/to_ui", 1);
    pub_actuator = this->create_publisher<std_msgs::msg::Float32MultiArray>("/master/actuator", 1);

    sub_obs_find = this->create_subscription<std_msgs::msg::Float32>(
        "/obstacle_filter/obs_find", 1, std::bind(&Master::callback_sub_obs_find, this, std::placeholders::_1));
    sub_beckhoff_sensor = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/beckhoff/sensors", 1, std::bind(&Master::callback_sub_beckhoff_sensor, this, std::placeholders::_1));
    sub_lane_kiri = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kiri", 1, std::bind(&Master::callback_sub_lane_kiri, this, std::placeholders::_1));
    sub_lane_kanan = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kanan", 1, std::bind(&Master::callback_sub_lane_kanan, this, std::placeholders::_1));
    sub_lane_tengah = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_tengah", 1, std::bind(&Master::callback_sub_lane_tengah, this, std::placeholders::_1));
    sub_lane_kiri_single_cam = this->create_subscription<ros2_interface::msg::PointArray>(
        "/cam_kiri/point_garis", 1, std::bind(&Master::callback_sub_lane_kiri_single_cam, this, std::placeholders::_1));
    sub_lane_kanan_single_cam = this->create_subscription<ros2_interface::msg::PointArray>(
        "/cam_kanan/point_garis", 1, std::bind(&Master::callback_sub_lane_kanan_single_cam, this, std::placeholders::_1));
    sub_aruco_kiri_detected = this->create_subscription<std_msgs::msg::Bool>(
        "/cam_kiri/aruco_detected", 1, std::bind(&Master::callback_sub_aruco_kiri_detected, this, std::placeholders::_1));
    sub_aruco_kanan_detected = this->create_subscription<std_msgs::msg::Bool>(
        "/cam_kanan/aruco_detected", 1, std::bind(&Master::callback_sub_aruco_kanan_detected, this, std::placeholders::_1));
    sub_hasil_perhitungan_kiri = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cam_kiri/hasil_kalkulasi", 1, std::bind(&Master::callback_sub_hasil_perhitungan_kiri, this, std::placeholders::_1));
    sub_hasil_perhitungan_kanan = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cam_kanan/hasil_kalkulasi", 1, std::bind(&Master::callback_sub_hasil_perhitungan_kanan, this, std::placeholders::_1));
    sub_error_code_beckhoff = this->create_subscription<std_msgs::msg::Int16>(
        "/beckhoff/error_code", 1, std::bind(&Master::callback_sub_error_code_beckhoff, this, std::placeholders::_1));
    sub_error_code_lidar = this->create_subscription<std_msgs::msg::Int16>(
        "/lidar/error_code", 1, std::bind(&Master::callback_sub_error_code_lidar, this, std::placeholders::_1));
    sub_error_code_cam_kiri = this->create_subscription<std_msgs::msg::Int16>(
        "/cam_kiri/error_code", 1, std::bind(&Master::callback_sub_error_code_cam_kiri, this, std::placeholders::_1));
    sub_error_code_cam_kanan = this->create_subscription<std_msgs::msg::Int16>(
        "/cam_kanan/error_code", 1, std::bind(&Master::callback_sub_error_code_cam_kanan, this, std::placeholders::_1));
    sub_error_code_pose_estimator = this->create_subscription<std_msgs::msg::Int16>(
        "/slam/error_code", 1, std::bind(&Master::callback_sub_error_code_pose_estimator, this, std::placeholders::_1));
    sub_error_code_obstacle_filter = this->create_subscription<std_msgs::msg::Int16>(
        "/obstacle_filter/error_code", 1, std::bind(&Master::callback_sub_error_code_obstacle_filter, this, std::placeholders::_1));
    sub_error_code_aruco_kiri = this->create_subscription<std_msgs::msg::Int16>(
        "/cam_kiri/error_code", 1, std::bind(&Master::callback_sub_error_code_aruco_kiri, this, std::placeholders::_1));
    sub_error_code_aruco_kanan = this->create_subscription<std_msgs::msg::Int16>(
        "/cam_kanan/error_code", 1, std::bind(&Master::callback_sub_error_code_aruco_kanan, this, std::placeholders::_1));
    sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&Master::callback_sub_joy, this, std::placeholders::_1));

    if (use_ekf_odometry)
        sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/slam/odometry/filtered", 1, std::bind(&Master::callback_sub_odometry, this, std::placeholders::_1));
    else
        sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&Master::callback_sub_odometry, this, std::placeholders::_1));

    //----Timer
    tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_tim_50hz, this));

    pid_vx.init(20, 3.7, 0, dt, 0, 80, 0, 60);

    logger.info("Master init success");
    global_fsm.value = FSM_GLOBAL_PREOP;
}
Master::~Master()
{
}

void Master::callback_sub_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    target_velocity_joy_x = profile_max_velocity * msg->axes[3];
    target_velocity_joy_y = 0;
    target_velocity_joy_wz = profile_max_steering_rad * msg->axes[0];
}

void Master::callback_sub_beckhoff_sensor(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    last_time_beckhoff = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    fb_steering_angle = msg->data[0];
}

void Master::callback_sub_obs_find(const std_msgs::msg::Float32::SharedPtr msg)
{
    last_time_obstacle_filter = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    obs_find = msg->data;
}

void Master::callback_sub_lane_kiri(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_kiri = *msg;
}

void Master::callback_sub_lane_tengah(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_tengah = *msg;
}

void Master::callback_sub_lane_kanan(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_kanan = *msg;
}

void Master::callback_sub_lane_kiri_single_cam(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_kiri_single_cam = *msg;
}

void Master::callback_sub_lane_kanan_single_cam(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_kanan_single_cam = *msg;
}

void Master::callback_sub_hasil_perhitungan_kiri(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    last_time_cam_kiri = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    cam_kiri_pid_output = msg->data[0];
    cam_kiri_pid_setpoint = msg->data[1];
    cam_kiri_pid_fb = msg->data[2];
    cam_kiri_velocity_gain = msg->data[3];
}

void Master::callback_sub_hasil_perhitungan_kanan(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    last_time_cam_kanan = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    cam_kanan_pid_output = msg->data[0];
    cam_kanan_pid_setpoint = msg->data[1];
    cam_kanan_pid_fb = msg->data[2];
    cam_kanan_velocity_gain = msg->data[3];
}

void Master::callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_time_pose_estimator = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    tf2::Quaternion q;
    double roll, pitch, yaw;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    fb_final_pose_xyo[0] = msg->pose.pose.position.x;
    fb_final_pose_xyo[1] = msg->pose.pose.position.y;
    fb_final_pose_xyo[2] = yaw;

    fb_final_vel_dxdydo[0] = msg->twist.twist.linear.x;
    fb_final_vel_dxdydo[1] = msg->twist.twist.linear.y;
    fb_final_vel_dxdydo[2] = msg->twist.twist.angular.z;
}

void Master::callback_sub_aruco_kiri_detected(const std_msgs::msg::Bool::SharedPtr msg)
{
    last_time_aruco_kiri = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    aruco_kiri_detected = msg->data;
}

void Master::callback_sub_aruco_kanan_detected(const std_msgs::msg::Bool::SharedPtr msg)
{
    last_time_aruco_kanan = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    aruco_kanan_detected = msg->data;
}

void Master::callback_sub_error_code_beckhoff(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_beckhoff = msg->data;
}

void Master::callback_sub_error_code_lidar(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_lidar = msg->data;
}

void Master::callback_sub_error_code_cam_kiri(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_cam_kiri = msg->data;
}

void Master::callback_sub_error_code_cam_kanan(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_cam_kanan = msg->data;
}

void Master::callback_sub_error_code_pose_estimator(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_pose_estimator = msg->data;
}

void Master::callback_sub_error_code_obstacle_filter(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_obstacle_filter = msg->data;
}

void Master::callback_sub_error_code_aruco_kiri(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_aruco_kiri = msg->data;
}

void Master::callback_sub_error_code_aruco_kanan(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_aruco_kanan = msg->data;
}

void Master::callback_tim_50hz()
{
    if (!marker.init(this->shared_from_this()))
    {
        rclcpp::shutdown();
    }

    current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    process_marker();

    switch (global_fsm.value)
    {
    /**
     * Pre-operation
     * Keadaan ini memastikan semua sistem tidak ada error
     */
    case FSM_GLOBAL_PREOP:
        if (error_code_beckhoff + error_code_cam_kiri + error_code_cam_kanan + error_code_lidar + error_code_pose_estimator + error_code_obstacle_filter + error_code_aruco_kiri + error_code_aruco_kanan == 0)
        {
            local_fsm.value = 0;
            global_fsm.value = FSM_GLOBAL_SAFEOP;
        }

        if (fabs(target_velocity_joy_x) > 0.1 || fabs(target_velocity_joy_y) > 0.1 || fabs(target_velocity_joy_wz) > 0.1)
        {
            manual_motion(target_velocity_joy_x, target_velocity_joy_y, target_velocity_joy_wz);
        }
        else
        {
            manual_motion(-20, 0, 0);
        }
        break;

    /**
     * Safe operation
     * Memastikan semua data bisa diterima
     */
    case FSM_GLOBAL_SAFEOP:
        if (error_code_beckhoff + error_code_cam_kiri + error_code_cam_kanan + error_code_lidar + error_code_pose_estimator + error_code_obstacle_filter + error_code_aruco_kiri + error_code_aruco_kanan > 0)
        {
            global_fsm.value = FSM_GLOBAL_PREOP;
        }

        {
            rclcpp::Duration dt_pose_estimator = current_time - last_time_pose_estimator;
            rclcpp::Duration dt_cam_kiri = current_time - last_time_cam_kiri;
            rclcpp::Duration dt_cam_kanan = current_time - last_time_cam_kanan;
            rclcpp::Duration dt_obstacle_filter = current_time - last_time_obstacle_filter;
            rclcpp::Duration dt_beckhoff = current_time - last_time_beckhoff;
            rclcpp::Duration dt_lidar = current_time - last_time_lidar;
            rclcpp::Duration dt_aruco_kiri = current_time - last_time_aruco_kiri;
            rclcpp::Duration dt_aruco_kanan = current_time - last_time_aruco_kanan;

            // Jika sudah berhasil menerima semua data yang diperlukan
            if (dt_pose_estimator.seconds() < 1 && dt_cam_kiri.seconds() < 1 && dt_cam_kanan.seconds() < 1 && dt_obstacle_filter.seconds() < 1 && dt_beckhoff.seconds() < 1 && dt_lidar.seconds() < 1 && dt_aruco_kiri.seconds() < 1 && dt_aruco_kanan.seconds() < 1)
            {
                target_velocity_joy_x = 0;
                target_velocity_joy_y = 0;
                target_velocity_joy_wz = 0;
                local_fsm.value = 0;
                global_fsm.value = FSM_GLOBAL_OP;
                time_start_operation = current_time;
            }
        }

        if (fabs(target_velocity_joy_x) > 0.1 || fabs(target_velocity_joy_y) > 0.1 || fabs(target_velocity_joy_wz) > 0.1)
        {
            manual_motion(target_velocity_joy_x, target_velocity_joy_y, target_velocity_joy_wz);
        }
        else
        {
            manual_motion(-20, 0, 0);
        }
        break;

    /**
     * Global operation
     * Sistem beroperasi secara otomatis
     */
    case FSM_GLOBAL_OP:
        if (error_code_beckhoff + error_code_cam_kiri + error_code_cam_kanan + error_code_lidar + error_code_pose_estimator + error_code_obstacle_filter + error_code_aruco_kiri + error_code_aruco_kanan > 0)
        {
            global_fsm.value = FSM_GLOBAL_PREOP;
        }
        process_local_fsm();
        break;
    }

    process_transmitter();
}

// ===============================================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}