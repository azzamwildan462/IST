#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "ros2_utils/help_logger.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

class LaneDetection : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;

    // Configs (dynamic)
    // =======================================================
    int low_h;
    int low_l;
    int low_s;
    int high_h;
    int high_l;
    int high_s;

    // Configs (static)
    // =======================================================
    std::string config_path;
    bool use_dynamic_config = false;

    int error_code = 0;
    HelpLogger logger;

    LaneDetection() : Node("lane_detection")
    {
        this->declare_parameter("config_path", "");
        this->get_parameter("config_path", config_path);

        this->declare_parameter("use_dynamic_config", false);
        this->get_parameter("use_dynamic_config", use_dynamic_config);

        this->declare_parameter("low_h", 0);
        this->get_parameter("low_h", low_h);

        this->declare_parameter("low_l", 0);
        this->get_parameter("low_l", low_l);

        this->declare_parameter("low_s", 0);
        this->get_parameter("low_s", low_s);

        this->declare_parameter("high_h", 0);
        this->get_parameter("high_h", high_h);

        this->declare_parameter("high_l", 0);
        this->get_parameter("high_l", high_l);

        this->declare_parameter("high_s", 0);
        this->get_parameter("high_s", high_s);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (use_dynamic_config)
        {
            load_config();
        }

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LaneDetection::callback_tim_50hz, this));
    }

    void load_config()
    {
        YAML::Node config;
        try
        {
            config = YAML::LoadFile(config_path);
        }
        catch (const std::exception &e)
        {
            error_code = 1;
            logger.error("Failed to load config file: %s", e.what());
        }

        low_h = config["Detection"]["low_h"].as<int>();
        low_l = config["Detection"]["low_l"].as<int>();
        low_s = config["Detection"]["low_s"].as<int>();
        high_h = config["Detection"]["high_h"].as<int>();
        high_l = config["Detection"]["high_l"].as<int>();
        high_s = config["Detection"]["high_s"].as<int>();
    }

    void save_config()
    {
        YAML::Node config;
        config["Detection"]["low_h"] = low_h;
        config["Detection"]["low_l"] = low_l;
        config["Detection"]["low_s"] = low_s;
        config["Detection"]["high_h"] = high_h;
        config["Detection"]["high_l"] = high_l;
        config["Detection"]["high_s"] = high_s;

        try
        {
            std::ofstream fout(config_path);
            fout << config;
        }
        catch (const std::exception &e)
        {
            error_code = 2;
            logger.error("Failed to save config file: %s", e.what());
        }
    }

    void callback_tim_50hz()
    {
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_lane_detection = std::make_shared<LaneDetection>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_lane_detection);
    executor.spin();

    return 0;
}