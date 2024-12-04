#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "ros2_utils/help_logger.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <thread>
#include <mutex>
#include "ros2_interface/msg/point_array.hpp"
#include "ros2_interface/srv/params.hpp"

class LaneDetection : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_gray;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_bgr;
    rclcpp::Publisher<ros2_interface::msg::PointArray>::SharedPtr pub_point_kiri;
    rclcpp::Publisher<ros2_interface::msg::PointArray>::SharedPtr pub_point_kanan;
    rclcpp::Publisher<ros2_interface::msg::PointArray>::SharedPtr pub_point_tengah;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_frame_display;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_frame_binary;

    rclcpp::Service<ros2_interface::srv::Params>::SharedPtr srv_params;

    // Configs (dynamic)
    // =======================================================
    int low_h;
    int low_l;
    int low_s;
    int high_h;
    int high_l;
    int high_s;
    int gray_threshold;

    // Configs (static)
    // =======================================================
    std::string config_path;
    bool use_dynamic_config = false;
    bool use_frame_bgr = true;

    // Vars
    // =======================================================
    int error_code = 0;
    HelpLogger logger;

    std::mutex mutex_frame_gray;
    std::mutex mutex_frame_bgr;
    cv::Mat frame_gray;
    cv::Mat frame_bgr;
    bool is_frame_gray_available = false;
    bool is_frame_bgr_available = false;

    LaneDetection() : Node("lane_detection")
    {
        this->declare_parameter("config_path", "");
        this->get_parameter("config_path", config_path);

        this->declare_parameter("use_dynamic_config", false);
        this->get_parameter("use_dynamic_config", use_dynamic_config);

        this->declare_parameter("low_h", 27);
        this->get_parameter("low_h", low_h);

        this->declare_parameter("low_l", 153);
        this->get_parameter("low_l", low_l);

        this->declare_parameter("low_s", 0);
        this->get_parameter("low_s", low_s);

        this->declare_parameter("high_h", 255);
        this->get_parameter("high_h", high_h);

        this->declare_parameter("high_l", 255);
        this->get_parameter("high_l", high_l);

        this->declare_parameter("high_s", 255);
        this->get_parameter("high_s", high_s);

        this->declare_parameter("gray_threshold", 200);
        this->get_parameter("gray_threshold", gray_threshold);

        this->declare_parameter("use_frame_bgr", true);
        this->get_parameter("use_frame_bgr", use_frame_bgr);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (use_dynamic_config)
        {
            load_config();
        }

        if (use_frame_bgr)
        {
            sub_image_bgr = this->create_subscription<sensor_msgs::msg::Image>(
                "image_bgr", 1, std::bind(&LaneDetection::callback_sub_image_bgr, this, std::placeholders::_1));
        }
        else
        {
            sub_image_gray = this->create_subscription<sensor_msgs::msg::Image>(
                "image_gray", 1, std::bind(&LaneDetection::callback_sub_image_gray, this, std::placeholders::_1));
        }

        pub_point_kiri = this->create_publisher<ros2_interface::msg::PointArray>("/lane_detection/point_kiri", 1);
        pub_point_kanan = this->create_publisher<ros2_interface::msg::PointArray>("/lane_detection/point_kanan", 1);
        pub_point_tengah = this->create_publisher<ros2_interface::msg::PointArray>("/lane_detection/point_tengah", 1);

        pub_frame_display = this->create_publisher<sensor_msgs::msg::Image>("/lane_detection/frame_display", 1);
        pub_frame_binary = this->create_publisher<sensor_msgs::msg::Image>("/lane_detection/frame_binary", 1);

        srv_params = this->create_service<ros2_interface::srv::Params>(
            "/lane_detection/params", std::bind(&LaneDetection::callback_srv_params, this, std::placeholders::_1, std::placeholders::_2));

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LaneDetection::callback_tim_50hz, this));

        logger.info("LaneDetection init success");
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
        config = YAML::LoadFile(config_path);

        config["Detection"]["low_h"] = low_h;
        config["Detection"]["low_l"] = low_l;
        config["Detection"]["low_s"] = low_s;
        config["Detection"]["high_h"] = high_h;
        config["Detection"]["high_l"] = high_l;
        config["Detection"]["high_s"] = high_s;

        try
        {
            std::ofstream fout;
            fout.open(config_path, std::ios::out);
            fout << config;
            fout.close();
        }
        catch (const std::exception &e)
        {
            error_code = 2;
            logger.error("Failed to save config file: %s", e.what());
        }
    }

    void callback_srv_params(const std::shared_ptr<ros2_interface::srv::Params::Request> request,
                             std::shared_ptr<ros2_interface::srv::Params::Response> response)
    {
        if (request->req_type == 0)
        {
            response->data.push_back(low_h);
            response->data.push_back(high_h);
            response->data.push_back(low_l);
            response->data.push_back(high_l);
            response->data.push_back(low_s);
            response->data.push_back(high_s);
        }
        else if (request->req_type == 1)
        {
            low_h = request->req_data[0];
            high_h = request->req_data[1];
            low_l = request->req_data[2];
            high_l = request->req_data[3];
            low_s = request->req_data[4];
            high_s = request->req_data[5];
            if (use_dynamic_config)
            {
                save_config();
            }
            response->data.push_back(low_h);
            response->data.push_back(high_h);
            response->data.push_back(low_l);
            response->data.push_back(high_l);
            response->data.push_back(low_s);
            response->data.push_back(high_s);
        }
    }

    void callback_sub_image_gray(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        mutex_frame_gray.lock();
        try
        {
            frame_gray = cv_bridge::toCvShare(msg, "mono8")->image.clone();
            is_frame_gray_available = true;
        }
        catch (const cv_bridge::Exception &e)
        {
            error_code = 3;
            logger.error("Failed to convert image: %s", e.what());
            mutex_frame_gray.unlock();
        }
        mutex_frame_gray.unlock();
    }

    void callback_sub_image_bgr(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_frame_bgr);
        try
        {
            frame_bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            is_frame_bgr_available = true;
        }
        catch (const cv_bridge::Exception &e)
        {
            error_code = 3;
            logger.error("Failed to convert image: %s", e.what());
        }
    }

    void process_frame_bgr()
    {
        if (!is_frame_bgr_available)
        {
            error_code = 5;
            return;
        }

        cv::Mat frame_bgr_copy;
        std::lock_guard<std::mutex> lock(mutex_frame_bgr);
        if (!frame_bgr.empty())
        {
            try
            {
                frame_bgr_copy = frame_bgr.clone();
                is_frame_bgr_available = false;
            }
            catch (const cv::Exception &e)
            {
                error_code = 4;
                logger.error("Failed to clone image: %s", e.what());
            }
        }

        if (frame_bgr_copy.empty())
        {
            error_code = 5;
            return;
        }

        cv::Mat frame_hls;
        cv::cvtColor(frame_bgr_copy, frame_hls, cv::COLOR_BGR2HLS);
        cv::Mat image_hls_threshold;
        cv::inRange(frame_hls, cv::Scalar(low_h, low_l, low_s), cv::Scalar(high_h, high_l, high_s), image_hls_threshold);

        std::vector<cv::Point> point_kiri;
        std::vector<cv::Point> point_kanan;
        std::vector<cv::Point> point_tengah;
        for (int i = 0; i < image_hls_threshold.rows; i++)
        {
            bool point_kiri_ditemukan = false;
            bool point_kanan_ditemukan = false;
            int point_kiri_x = 0;
            for (int j = 0; j < image_hls_threshold.cols; j++)
            {
                if (image_hls_threshold.at<uchar>(i, j) == 255 && !point_kiri_ditemukan)
                {
                    point_kiri.push_back(cv::Point(j, i));
                    point_kiri_x = j;
                    point_kiri_ditemukan = true;
                }
                else if (image_hls_threshold.at<uchar>(i, j) == 255 && point_kiri_ditemukan && j - point_kiri_x > 50)
                {
                    point_kanan.push_back(cv::Point(j, i));
                    int point_tengah_x = (point_kiri_x + j) / 2;
                    point_tengah.push_back(cv::Point(point_tengah_x, i));
                    point_kanan_ditemukan = true;
                    break;
                }
            }

            if (!point_kanan_ditemukan && point_kiri_ditemukan)
            {
                point_kiri.pop_back();
            }
        }

        ros2_interface::msg::PointArray msg_point_kiri;
        ros2_interface::msg::PointArray msg_point_tengah;
        ros2_interface::msg::PointArray msg_point_kanan;

        for (size_t i = 0; i < point_kiri.size(); i++)
        {
            cv::circle(frame_bgr_copy, point_kiri[i], 1, cv::Scalar(0, 0, 255), 3);
            geometry_msgs::msg::Point p;
            p.x = point_kiri[i].x;
            p.y = point_kiri[i].y;
            msg_point_kiri.points.push_back(p);
        }

        for (size_t i = 0; i < point_kanan.size(); i++)
        {
            cv::circle(frame_bgr_copy, point_kanan[i], 1, cv::Scalar(0, 255, 0), 3);
            geometry_msgs::msg::Point p;
            p.x = point_kanan[i].x;
            p.y = point_kanan[i].y;
            msg_point_kanan.points.push_back(p);
        }

        for (size_t i = 0; i < point_tengah.size(); i++)
        {
            cv::circle(frame_bgr_copy, point_tengah[i], 1, cv::Scalar(255, 0, 0), 3);
            geometry_msgs::msg::Point p;
            p.x = point_tengah[i].x;
            p.y = point_tengah[i].y;
            msg_point_tengah.points.push_back(p);
        }

        pub_point_kiri->publish(msg_point_kiri);
        pub_point_kanan->publish(msg_point_kanan);
        pub_point_tengah->publish(msg_point_tengah);

        auto msg_frame_display = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr_copy).toImageMsg();
        pub_frame_display->publish(*msg_frame_display);

        auto msg_frame_binary = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_hls_threshold).toImageMsg();
        pub_frame_binary->publish(*msg_frame_binary);

        // cv::imshow("Lane Detection", frame_bgr_copy);
        // cv::waitKey(1);
    }

    void process_frame_gray()
    {
        if (!is_frame_gray_available)
        {
            error_code = 5;
            return;
        }

        cv::Mat frame_gray_copy;
        mutex_frame_gray.lock();
        try
        {
            frame_gray_copy = frame_gray.clone();
            is_frame_gray_available = false;
        }
        catch (const cv::Exception &e)
        {
            error_code = 4;
            logger.error("Failed to clone image: %s", e.what());
            mutex_frame_gray.unlock();
        }
        mutex_frame_gray.unlock();

        if (frame_gray_copy.empty())
        {
            error_code = 5;
            return;
        }

        cv::Mat frame_gray_blur;
        cv::GaussianBlur(frame_gray_copy, frame_gray_blur, cv::Size(5, 5), 0);

        cv::Mat frame_gray_binary;
        cv::threshold(frame_gray_blur, frame_gray_binary, gray_threshold, 255, cv::THRESH_BINARY);

        cv::Mat frame_gray_canny;
        cv::Canny(frame_gray_binary, frame_gray_canny, 50, 150);

        // Hough Line Transform
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(frame_gray_canny, lines, 1, CV_PI / 180, 50, 50, 10);
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            cv::line(frame_gray_copy, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
        }

        // Separate left and right lines
        std::vector<cv::Vec4i> left_lines, right_lines;
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            float slope = (float)(l[3] - l[1]) / (l[2] - l[0]);
            if (slope > 0)
            {
                right_lines.push_back(l);
            }
            else if (slope < 0)
            {
                left_lines.push_back(l);
            }
        }
    }

    void callback_tim_50hz()
    {
        if (use_frame_bgr)
        {
            process_frame_bgr();
        }
        else
        {
            process_frame_gray();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_lane_detection = std::make_shared<LaneDetection>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_lane_detection);
    executor.spin();

    return 0;
}