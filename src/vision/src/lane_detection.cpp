#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_interface/msg/point_array.hpp"
#include "ros2_interface/srv/params.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/int16.hpp"
#include <fstream>
#include <mutex>
#include <thread>
#include <yaml-cpp/yaml.h>

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
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;

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
    std::string video_path;
    bool use_dynamic_config = false;
    bool use_frame_bgr = true;
    std::string node_namespace = "";
    std::string absolute_image_topic = "";
    int lookahead_distance_y = 300;
    int erode_size = -1;
    int dilate_size = -1;
    int morph_close_size = -1;
    int jarak_minimal = 50;
    int jarak_maksimal = 100;
    int pangkal_x = 400;
    int pangkal_y = 600;
    int batas_x_kiri_scan = 200;
    int batas_x_kanan_scan = 600;
    int minimum_contour_area = 100;
    float deadzone_kanan_kiri_thr = 0.8;

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

    LaneDetection()
        : Node("lane_detection")
    {
        this->declare_parameter("config_path", "");
        this->get_parameter("config_path", config_path);

        this->declare_parameter("video_path", "");
        this->get_parameter("video_path", video_path);

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

        this->declare_parameter("absolute_image_topic", "");
        this->get_parameter("absolute_image_topic", absolute_image_topic);

        this->declare_parameter("lookahead_distance_y", 300);
        this->get_parameter("lookahead_distance_y", lookahead_distance_y);

        this->declare_parameter("erode_size", -1);
        this->get_parameter("erode_size", erode_size);

        this->declare_parameter("dilate_size", -1);
        this->get_parameter("dilate_size", dilate_size);

        this->declare_parameter("morph_close_size", -1);
        this->get_parameter("morph_close_size", morph_close_size);

        this->declare_parameter("jarak_minimal", 50);
        this->get_parameter("jarak_minimal", jarak_minimal);

        this->declare_parameter("jarak_maksimal", 100);
        this->get_parameter("jarak_maksimal", jarak_maksimal);

        this->declare_parameter("pangkal_x", 400);
        this->get_parameter("pangkal_x", pangkal_x);

        this->declare_parameter("pangkal_y", 600);
        this->get_parameter("pangkal_y", pangkal_y);

        this->declare_parameter("batas_x_kiri_scan", 200);
        this->get_parameter("batas_x_kiri_scan", batas_x_kiri_scan);

        this->declare_parameter("batas_x_kanan_scan", 600);
        this->get_parameter("batas_x_kanan_scan", batas_x_kanan_scan);

        this->declare_parameter("minimum_contour_area", 100);
        this->get_parameter("minimum_contour_area", minimum_contour_area);

        this->declare_parameter("deadzone_kanan_kiri_thr", 0.8);
        this->get_parameter("deadzone_kanan_kiri_thr", deadzone_kanan_kiri_thr);

        node_namespace = this->get_namespace();
        node_namespace = node_namespace.substr(1, node_namespace.size() - 1); // /cam_kanan jadi cam_kanan

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (use_dynamic_config)
        {
            load_config();
        }

        if (absolute_image_topic != "")
        {
            logger.info("Subscribing to absolute image topic: %s", absolute_image_topic.c_str());
            sub_image_bgr = this->create_subscription<sensor_msgs::msg::Image>(
                absolute_image_topic, 1, std::bind(&LaneDetection::callback_sub_image_bgr, this, std::placeholders::_1));
        }
        else
        {
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
        }

        pub_point_kiri = this->create_publisher<ros2_interface::msg::PointArray>("/lane_detection/point_kiri", 1);
        pub_point_kanan = this->create_publisher<ros2_interface::msg::PointArray>("/lane_detection/point_kanan", 1);
        pub_point_tengah = this->create_publisher<ros2_interface::msg::PointArray>("/lane_detection/point_tengah", 1);

        pub_frame_display = this->create_publisher<sensor_msgs::msg::Image>("/lane_detection/frame_display", 1);
        pub_frame_binary = this->create_publisher<sensor_msgs::msg::Image>("/lane_detection/frame_binary", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("/lane_detection/error_code", 1);

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

        try
        {
            low_h = config["Detection"][node_namespace.c_str()]["low_h"].as<int>();
            low_l = config["Detection"][node_namespace.c_str()]["low_l"].as<int>();
            low_s = config["Detection"][node_namespace.c_str()]["low_s"].as<int>();
            high_h = config["Detection"][node_namespace.c_str()]["high_h"].as<int>();
            high_l = config["Detection"][node_namespace.c_str()]["high_l"].as<int>();
            high_s = config["Detection"][node_namespace.c_str()]["high_s"].as<int>();
        }
        catch (const std::exception &e)
        {
            logger.warn("Failed to load config file, Recreate the config file");
            save_config();
        }
    }

    void save_config()
    {
        YAML::Node config;
        config = YAML::LoadFile(config_path);

        config["Detection"][node_namespace.c_str()]["low_h"] = low_h;
        config["Detection"][node_namespace.c_str()]["low_l"] = low_l;
        config["Detection"][node_namespace.c_str()]["low_s"] = low_s;
        config["Detection"][node_namespace.c_str()]["high_h"] = high_h;
        config["Detection"][node_namespace.c_str()]["high_l"] = high_l;
        config["Detection"][node_namespace.c_str()]["high_s"] = high_s;

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
            return;
        }

        static uint64_t counter_video = 0;

        if (video_path != "")
        {
            if (counter_video % 10 == 0)
            {
                cv::imwrite(video_path + "/frame_" + std::to_string(counter_video) + ".jpg", frame_bgr_copy);
            }
            counter_video++;
        }

        error_code = 0;

        cv::Mat frame_hls;
        cv::cvtColor(frame_bgr_copy, frame_hls, cv::COLOR_BGR2HSV);
        cv::Mat image_hls_threshold;
        cv::inRange(frame_hls, cv::Scalar(low_h, low_l, low_s), cv::Scalar(high_h, high_l, high_s), image_hls_threshold);

        if (erode_size > 0 && dilate_size > 0)
        {
            cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_size, erode_size));
            cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));
            cv::erode(image_hls_threshold, image_hls_threshold, element_erode);
            cv::dilate(image_hls_threshold, image_hls_threshold, element_dilate);
        }

        if (morph_close_size > 0)
        {
            cv::Mat element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_close_size, morph_close_size));
            cv::morphologyEx(image_hls_threshold, image_hls_threshold, cv::MORPH_CLOSE, element_close);
        }

        // Create mask based on batas_x_kiri_scan and batas_x_kanan_scan and lookahead_distance_y and image_hls_threshold.rows
        cv::Mat mask = cv::Mat::zeros(image_hls_threshold.size(), CV_8UC1);
        cv::rectangle(mask, cv::Point(batas_x_kiri_scan, lookahead_distance_y), cv::Point(batas_x_kanan_scan, image_hls_threshold.rows), cv::Scalar(255), -1);
        cv::bitwise_and(image_hls_threshold, mask, image_hls_threshold);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(image_hls_threshold, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Filter out small contours
        cv::Mat mask_by_contour_area = cv::Mat::zeros(image_hls_threshold.size(), CV_8UC1);

        for (size_t i = 0; i < contours.size(); i++)
        {
            if (cv::contourArea(contours[i]) > minimum_contour_area)
            {
                cv::drawContours(mask_by_contour_area, contours, (int)i, cv::Scalar(255), cv::FILLED);
            }
        }
        cv::bitwise_and(image_hls_threshold, mask_by_contour_area, image_hls_threshold);

        cv::line(frame_bgr_copy, cv::Point(batas_x_kiri_scan, lookahead_distance_y), cv::Point(batas_x_kiri_scan, image_hls_threshold.rows), cv::Scalar(255, 255, 0), 2);
        cv::line(frame_bgr_copy, cv::Point(batas_x_kanan_scan, lookahead_distance_y), cv::Point(batas_x_kanan_scan, image_hls_threshold.rows), cv::Scalar(255, 255, 0), 2);
        cv::line(frame_bgr_copy, cv::Point(batas_x_kiri_scan, lookahead_distance_y), cv::Point(batas_x_kanan_scan, lookahead_distance_y), cv::Scalar(255, 255, 0), 2);
        cv::circle(frame_bgr_copy, cv::Point(pangkal_x, pangkal_y), 15, cv::Scalar(255, 0, 255), -1);

        uint8_t curr_val = 0;
        uint8_t prev_val = 0;
        uint16_t counter_ketemu_satu_garis = 0;
        uint16_t counter_ketemu_dua_garis = 0;

        std::vector<cv::Point> point_kiri;
        std::vector<cv::Point> point_kanan;
        std::vector<cv::Point> point_tengah;
        float sudut_jumlah = 0;
        uint32_t sudut_n = 0;
        for (int i = lookahead_distance_y; i < image_hls_threshold.rows; i++)
        {
            bool point_kiri_ditemukan = false;
            bool point_kanan_ditemukan = false;
            bool point_kanan_satu_ditemukan = false;
            cv::Point point_kanan_satu;
            int point_kiri_x = 0;
            for (int j = batas_x_kiri_scan; j < batas_x_kanan_scan; j++)
            {
                curr_val = image_hls_threshold.at<uchar>(i, j);

                if (jarak_minimal > 0 && jarak_maksimal > 0)
                {
                    if (((prev_val == 0 && curr_val == 255) || (j == batas_x_kiri_scan && curr_val == 255)) && !point_kiri_ditemukan)
                    {
                        point_kiri.push_back(cv::Point(j, i));
                        point_kiri_x = j;
                        point_kiri_ditemukan = true;
                    }
                    else if (((prev_val == 255 && curr_val == 0) || (j == batas_x_kanan_scan - 1 && curr_val == 255)) && point_kiri_ditemukan && j - point_kiri_x > jarak_maksimal)
                    {
                        point_kiri[point_kiri.size() - 1] = cv::Point(j, i);
                    }
                    else if (((prev_val == 255 && curr_val == 0) || (j == batas_x_kanan_scan - 1 && curr_val == 255)) && point_kiri_ditemukan && j - point_kiri_x > jarak_minimal)
                    {
                        point_kanan.push_back(cv::Point(j, i));
                        int point_tengah_x = (point_kiri_x + j) * 0.5;
                        point_tengah.push_back(cv::Point(point_tengah_x, i));
                        point_kanan_ditemukan = true;
                        counter_ketemu_dua_garis++;

                        float dx = point_tengah_x - pangkal_x;
                        float dy = i - pangkal_y;
                        float sudut = atan2(dy, dx);
                        sudut_jumlah += sudut;
                        sudut_n++;

                        break;
                    }
                    else if ((prev_val == 255 && curr_val == 0))
                    {
                        point_kanan_satu = cv::Point(j, i);
                        point_kanan_satu_ditemukan = true;
                    }
                    // Ketika hanya nemu satu garis
                    else if (j == batas_x_kanan_scan - 1 && !point_kanan_ditemukan)
                    {
                        if (point_kiri_ditemukan && point_kanan_satu_ditemukan && point_kanan_satu.x > point_kiri_x)
                        {
                            point_kanan.push_back(point_kanan_satu);
                            int point_tengah_x = (point_kiri_x + point_kanan_satu.x) * 0.5;
                            point_tengah.push_back(cv::Point(point_tengah_x, i));
                            point_kanan_ditemukan = true;
                            counter_ketemu_satu_garis++;

                            float dx = point_tengah_x - pangkal_x;
                            float dy = i - pangkal_y;
                            float sudut = atan2(dy, dx);
                            sudut_jumlah += sudut;
                            sudut_n++;
                        }
                    }
                }
                prev_val = curr_val;
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

        float sudut_mean = 0;
        if (sudut_n > 0)
        {
            sudut_mean = sudut_jumlah / sudut_n + 1.57;
        }
        for (size_t i = 0; i < point_tengah.size(); i++)
        {
            cv::circle(frame_bgr_copy, point_tengah[i], 1, cv::Scalar(255, 0, 0), 3);
            geometry_msgs::msg::Point p;

            if (counter_ketemu_satu_garis - counter_ketemu_dua_garis > 5)
            {

                if (sudut_mean > deadzone_kanan_kiri_thr)
                {
                    p.x = point_kanan[i].x;
                    p.y = point_kanan[i].y;
                }
                else if (sudut_mean < -deadzone_kanan_kiri_thr)
                {
                    p.x = point_kiri[i].x;
                    p.y = point_kiri[i].y;
                }
                else
                {
                    p.x = point_tengah[i].x;
                    p.y = point_tengah[i].y;
                }
            }
            else
            {
                p.x = point_tengah[i].x;
                p.y = point_tengah[i].y;
            }

            msg_point_tengah.points.push_back(p);
        }
        geometry_msgs::msg::Point p;
        p.x = pangkal_x;
        p.y = pangkal_y;
        msg_point_tengah.points.push_back(p);

        pub_point_kiri->publish(msg_point_kiri);
        pub_point_kanan->publish(msg_point_kanan);
        pub_point_tengah->publish(msg_point_tengah);

        auto msg_frame_display = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr_copy).toImageMsg();
        pub_frame_display->publish(*msg_frame_display);

        auto msg_frame_binary = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_hls_threshold).toImageMsg();
        pub_frame_binary->publish(*msg_frame_binary);

        // cv::imshow("Lane DoubleDetection", frame_bgr_copy);
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

        std_msgs::msg::Int16 msg_error_code;
        msg_error_code.data = error_code;
        pub_error_code->publish(msg_error_code);
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