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

    sub_lane_kiri = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kiri", 1, std::bind(&Master::callback_sub_lane_kiri, this, std::placeholders::_1));
    sub_lane_tengah = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_tengah", 1, std::bind(&Master::callback_sub_lane_tengah, this, std::placeholders::_1));
    sub_lane_kanan = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kanan", 1, std::bind(&Master::callback_sub_lane_kanan, this, std::placeholders::_1));

    //----Timer
    tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_tim_50hz, this));

    logger.info("Master init success");
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

void Master::callback_tim_50hz()
{
    if (!marker.init(this->shared_from_this()))
    {
        rclcpp::shutdown();
    }

    process_marker();
    process_fsm();

    manual_motion(actuation_vx, actuation_vy, actuation_wz);
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