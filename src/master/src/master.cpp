#include "rclcpp/rclcpp.hpp"
#include "master/master.hpp"

Master::Master() : Node("master")
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

    sub_lane_kiri = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kiri", 1, std::bind(&Master::callback_sub_lane_kiri, this, std::placeholders::_1));
    sub_lane_tengah = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_tengah", 1, std::bind(&Master::callback_sub_lane_tengah, this, std::placeholders::_1));
    sub_lane_kanan = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kanan", 1, std::bind(&Master::callback_sub_lane_kanan, this, std::placeholders::_1));
    sub_lane_kiri_single_cam = this->create_subscription<ros2_interface::msg::PointArray>(
        "/cam_kiri/point_garis", 1, std::bind(&Master::callback_sub_lane_kiri_single_cam, this, std::placeholders::_1));
    sub_lane_kanan_single_cam = this->create_subscription<ros2_interface::msg::PointArray>(
        "/cam_kanan/point_garis", 1, std::bind(&Master::callback_sub_lane_kanan_single_cam, this, std::placeholders::_1));
    sub_hasil_perhitungan_kiri = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cam_kiri/hasil_kalkulasi", 1, std::bind(&Master::callback_sub_hasil_perhitungan_kiri, this, std::placeholders::_1));
    sub_hasil_perhitungan_kanan = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cam_kanan/hasil_kalkulasi", 1, std::bind(&Master::callback_sub_hasil_perhitungan_kanan, this, std::placeholders::_1));

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
    cam_kiri_pid_output = msg->data[0];
    cam_kiri_pid_setpoint = msg->data[1];
    cam_kiri_pid_fb = msg->data[2];
    cam_kiri_velocity_gain = msg->data[3];
}

void Master::callback_sub_hasil_perhitungan_kanan(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    cam_kanan_pid_output = msg->data[0];
    cam_kanan_pid_setpoint = msg->data[1];
    cam_kanan_pid_fb = msg->data[2];
    cam_kanan_velocity_gain = msg->data[3];
}

void Master::callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
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

void Master::callback_tim_50hz()
{
    if (!marker.init(this->shared_from_this()))
    {
        rclcpp::shutdown();
    }

    process_marker();

    switch (global_fsm.value)
    {
    /**
     * Pre-operation
     * Keadaan ini memastikan semua sistem tidak ada error
     */
    case FSM_GLOBAL_PREOP:
        if (error_code_beckhoff + error_code_cam_kiri + error_code_cam_kanan + error_code_lidar == 0)
        {
            local_fsm.value = 0;
            global_fsm.value = FSM_GLOBAL_SAFEOP;
        }
        manual_motion(0, 0, 0);
        break;

    /**
     * Safe operation
     * Memastikan semua data bisa diterima
     */
    case FSM_GLOBAL_SAFEOP:
        if (error_code_beckhoff + error_code_cam_kiri + error_code_cam_kanan + error_code_lidar > 0)
        {
            global_fsm.value = FSM_GLOBAL_PREOP;
        }

        // Ini sementara saja
        if (error_code_beckhoff + error_code_cam_kiri + error_code_cam_kanan + error_code_lidar == 0)
        {
            local_fsm.value = 0;
            global_fsm.value = FSM_GLOBAL_OP;
        }
        manual_motion(0, 0, 0);

        break;

    /**
     * Global operation
     * Sistem beroperasi secara otomatis
     */
    case FSM_GLOBAL_OP:
        if (error_code_beckhoff + error_code_cam_kiri + error_code_cam_kanan + error_code_lidar > 0)
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