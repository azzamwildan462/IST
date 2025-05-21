/**
 * @file      ascamera_node.cpp
 * @brief     angstrong ros2 camera publisher node.
 *
 * Copyright (c) 2023 Angstrong Tech.Co.,Ltd
 *
 * @author    Angstrong SDK develop Team
 * @date      2023/03/27
 * @version   1.0

 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

#include "CameraPublisher.h"
#include "LogRedirectBuffer.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();

    // Create logger and redirect output
    auto logger = node->get_logger();
    auto buf = std::make_shared<LogRedirectBuffer>(logger);
    std::cout.rdbuf(buf.get());
    std::cerr.rdbuf(buf.get());

    RCLCPP_INFO(logger, "Hello world Angstrong Camera ROS2 Node");

    node->start();

    // Create a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add the node to the executor
    executor.add_node(node);

    // Set the loop rate (Optional: Multi-threaded executor won't strictly use loop_rate)
    rclcpp::WallRate loop_rate(100);

    // Run the executor
    while (rclcpp::ok())
    {
        executor.spin_once(); // Spins one iteration in multiple threads
        loop_rate.sleep();    // Maintain the loop rate
    }

    node->stop();

    RCLCPP_INFO(logger, "Angstrong Camera ROS2 Node Shutdown");

    rclcpp::shutdown();
    return 0;
}
