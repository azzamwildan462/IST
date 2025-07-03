#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_interface/msg/point_array.hpp"
#include "ros2_interface/srv/params.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/pid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include <fstream>
#include <mutex>
#include <opencv2/aruco.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

class SingleDetection : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_gray;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_bgr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_depth;
    rclcpp::Publisher<ros2_interface::msg::PointArray>::SharedPtr pub_point_garis;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_frame_display;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_frame_binary;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_hasil_kalkulasi;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_aruco_detected;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_aruco_nearest_marker_id;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_forklift_detected;

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
    int setpoint_x;
    int setpoint_y;
    int pangkal_x;
    int pangkal_y;

    // Configs (static)
    // =======================================================
    std::string config_path;
    bool use_dynamic_config = false;
    bool use_frame_bgr = true;
    bool right_to_left_scan = false;
    int metode_perhitungan = 2;
    int erode_size = 3;
    int dilate_size = 3;
    std::string node_namespace = "";
    float point_to_velocity_ratio = 0.01;
    float point_to_velocity_angle_threshold = 0.02;
    bool detect_aruco = false;
    std::string aruco_dictionary_type = "DICT_4x4_50";
    float min_aruco_range = 100; // Jarak ke setpoint
    int aruco_in_counter_threshold = 50;
    int aruco_out_counter_threshold = 50;
    std::string camera_namespace = "cam_kanan";
    std::string absolute_image_topic = "";
    std::string absolute_depth_image_topic = "";
    bool is_detect_forklift = false;
    int threshold_forklift_px_size = 300; // px
    std::vector<double> roi_detect_forklift;

    bool debug_mode = false;
    float rotation_angle = 0; // rad

    float maximum_error_jarak_setpoint = 70;

    std::vector<double> pid_terms;

    // Vars
    // =======================================================
    int error_code = 0;
    HelpLogger logger;
    PID pid_point_emergency; // Output dari PID digunakan ketika node master nge-hang

    std::mutex mutex_frame_gray;
    std::mutex mutex_frame_bgr;
    cv::Mat frame_gray;
    cv::Mat frame_bgr;
    bool is_frame_gray_available = false;
    bool is_frame_bgr_available = false;
    cv::Ptr<cv::aruco::Dictionary> aruco_dictionary;

    int center_frame_x = 0;
    int center_frame_y = 0;

    // Tesseract engine
    std::unique_ptr<tesseract::TessBaseAPI> tess_;

    SingleDetection()
        : Node("single_detection")
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

        this->declare_parameter("setpoint_x", 320);
        this->get_parameter("setpoint_x", setpoint_x);

        this->declare_parameter("setpoint_y", 240);
        this->get_parameter("setpoint_y", setpoint_y);

        this->declare_parameter("pangkal_x", 9999);
        this->get_parameter("pangkal_x", pangkal_x);

        this->declare_parameter("pangkal_y", 9999);
        this->get_parameter("pangkal_y", pangkal_y);

        this->declare_parameter("metode_perhitungan", 2);
        this->get_parameter("metode_perhitungan", metode_perhitungan);

        this->declare_parameter("erode_size", 3);
        this->get_parameter("erode_size", erode_size);

        this->declare_parameter("dilate_size", 3);
        this->get_parameter("dilate_size", dilate_size);

        this->declare_parameter("point_to_velocity_ratio", 0.01);
        this->get_parameter("point_to_velocity_ratio", point_to_velocity_ratio);

        this->declare_parameter("point_to_velocity_angle_threshold", 0.02);
        this->get_parameter("point_to_velocity_angle_threshold", point_to_velocity_angle_threshold);

        this->declare_parameter("detect_aruco", false);
        this->get_parameter("detect_aruco", detect_aruco);

        this->declare_parameter("aruco_dictionary_type", "DICT_4x4_50");
        this->get_parameter("aruco_dictionary_type", aruco_dictionary_type);

        this->declare_parameter("min_aruco_range", 100.0);
        this->get_parameter("min_aruco_range", min_aruco_range);

        this->declare_parameter("aruco_in_counter_threshold", 50);
        this->get_parameter("aruco_in_counter_threshold", aruco_in_counter_threshold);

        this->declare_parameter("aruco_out_counter_threshold", 50);
        this->get_parameter("aruco_out_counter_threshold", aruco_out_counter_threshold);

        this->declare_parameter("camera_namespace", "/cam_kanan");
        this->get_parameter("camera_namespace", camera_namespace);

        this->declare_parameter("right_to_left_scan", false);
        this->get_parameter("right_to_left_scan", right_to_left_scan);

        this->declare_parameter("absolute_image_topic", "");
        this->get_parameter("absolute_image_topic", absolute_image_topic);

        this->declare_parameter("debug_mode", false);
        this->get_parameter("debug_mode", debug_mode);

        this->declare_parameter("is_detect_forklift", false);
        this->get_parameter("is_detect_forklift", is_detect_forklift);

        this->declare_parameter("absolute_depth_image_topic", "/camera/rs2_cam_main/aligned_depth_to_color/image_raw");
        this->get_parameter("absolute_depth_image_topic", absolute_depth_image_topic);

        this->declare_parameter("rotation_angle", 0.0);
        this->get_parameter("rotation_angle", rotation_angle);

        this->declare_parameter("maximum_error_jarak_setpoint", 50.0);
        this->get_parameter("maximum_error_jarak_setpoint", maximum_error_jarak_setpoint);

        this->declare_parameter("threshold_forklift_px_size", 300);
        this->get_parameter("threshold_forklift_px_size", threshold_forklift_px_size);

        this->declare_parameter<std::vector<double>>("pid_terms", {0.003, 0.000000, 0, 0.02, -0.1, 0.1, -0.0005, 0.0005});
        this->get_parameter("pid_terms", pid_terms);

        this->declare_parameter<std::vector<double>>("roi_detect_forklift", {100.0, 100.0, 300.0, 300.0});
        this->get_parameter("roi_detect_forklift", roi_detect_forklift);

        node_namespace = this->get_namespace();
        node_namespace = node_namespace.substr(1, node_namespace.size() - 1); // /cam_kanan jadi cam_kanan

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        logger.info("Using namespace: %s", node_namespace.c_str());
        if (use_dynamic_config)
        {
            load_config();
        }

        if (detect_aruco)
        {
            logger.info("Aruco detection on %s", aruco_dictionary_type.c_str());
            if (aruco_dictionary_type == "DICT_4X4_50")
                aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            else if (aruco_dictionary_type == "DICT_4X4_100")
                aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
            else if (aruco_dictionary_type == "DICT_4X4_250")
                aruco_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        }

        if (absolute_image_topic != "")
        {
            logger.info("Subscribing to absolute image topic: %s", absolute_image_topic.c_str());
            sub_image_bgr = this->create_subscription<sensor_msgs::msg::Image>(
                absolute_image_topic, 1, std::bind(&SingleDetection::callback_sub_image_bgr, this, std::placeholders::_1));
        }
        else
        {
            if (use_frame_bgr)
            {
                sub_image_bgr = this->create_subscription<sensor_msgs::msg::Image>(
                    "/" + camera_namespace + "/image_bgr", 1, std::bind(&SingleDetection::callback_sub_image_bgr, this, std::placeholders::_1));
            }
            else
            {
                sub_image_gray = this->create_subscription<sensor_msgs::msg::Image>(
                    "/" + camera_namespace + "/image_gray", 1, std::bind(&SingleDetection::callback_sub_image_gray, this, std::placeholders::_1));
            }
        }

        pub_point_garis = this->create_publisher<ros2_interface::msg::PointArray>("point_garis", 1);
        pub_hasil_kalkulasi = this->create_publisher<std_msgs::msg::Float32MultiArray>("hasil_kalkulasi", 1);
        pub_aruco_detected = this->create_publisher<std_msgs::msg::Bool>("aruco_detected", 1);
        pub_aruco_nearest_marker_id = this->create_publisher<std_msgs::msg::Int16>("aruco_nearest_marker_id", 1);
        pub_forklift_detected = this->create_publisher<std_msgs::msg::Float32>("forklift_detected", 1);

        pub_frame_display = this->create_publisher<sensor_msgs::msg::Image>("frame_display", 1);
        pub_frame_binary = this->create_publisher<sensor_msgs::msg::Image>("frame_binary", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("error_code", 1);

        srv_params = this->create_service<ros2_interface::srv::Params>(
            "params", std::bind(&SingleDetection::callback_srv_params, this, std::placeholders::_1, std::placeholders::_2));

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(60), std::bind(&SingleDetection::callback_tim_50hz, this));

        pid_point_emergency.init(pid_terms[0], pid_terms[1], pid_terms[2], pid_terms[3], pid_terms[4], pid_terms[5], pid_terms[6], pid_terms[7]);

        // 1) Prepare Tesseract once
        tess_ = std::make_unique<tesseract::TessBaseAPI>();
        if (tess_->Init(nullptr, "eng", tesseract::OEM_LSTM_ONLY) != 0)
        {
            RCLCPP_FATAL(get_logger(), "Could not initialize tesseract.");
            rclcpp::shutdown();
            return;
        }
        tess_->SetSourceResolution(300); // or whatever DPI makes sense for your camera
        tess_->SetPageSegMode(tesseract::PSM_SINGLE_BLOCK);
        tess_->SetVariable("tessedit_char_whitelist", "E0123456789BYD");

        logger.info("SingleDetection %s init success %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", node_namespace.c_str(), pid_terms[0], pid_terms[1], pid_terms[2], pid_terms[3], pid_terms[4], pid_terms[5], pid_terms[6], pid_terms[7]);
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
        else if (request->req_type == 2)
        {
            response->data.push_back(setpoint_x);
            response->data.push_back(setpoint_y);
        }
        else if (request->req_type == 3)
        {
            setpoint_x = request->req_data[0];
            setpoint_y = request->req_data[1];
            if (use_dynamic_config)
            {
                save_config();
            }
            response->data.push_back(setpoint_x);
            response->data.push_back(setpoint_y);
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
        mutex_frame_bgr.lock();
        try
        {
            frame_bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            is_frame_bgr_available = true;
            mutex_frame_bgr.unlock();
        }
        catch (const cv_bridge::Exception &e)
        {
            error_code = 3;
            logger.error("Failed to convert image: %s", e.what());
            mutex_frame_bgr.unlock();
        }

        // frame_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;

        // process_frame_bgr();
    }

    void forklift_detection_bgr()
    {
        if (!is_frame_bgr_available)
        {
            // error_code = 5;
            return;
        }

        cv::Mat frame_bgr_copy;
        mutex_frame_bgr.lock();
        if (!frame_bgr.empty())
        {
            try
            {
                frame_bgr_copy = frame_bgr.clone();
                mutex_frame_bgr.unlock();
            }
            catch (const cv::Exception &e)
            {
                error_code = 4;
                logger.error("Failed to clone image: %s", e.what());
                mutex_frame_bgr.unlock();
            }
        }

        if (frame_bgr_copy.empty())
        {
            // error_code = 5;
            return;
        }

        // logger.info("DETECT FORKLIFT");

        error_code = 0;

        cv::Mat frame_hsv;
        cv::cvtColor(frame_bgr_copy, frame_hsv, cv::COLOR_BGR2HSV);

        cv::Mat image_threshold;
        cv::inRange(frame_hsv, cv::Scalar(low_h, low_l, low_s), cv::Scalar(high_h, high_l, high_s), image_threshold);

        // Mask dengan roi
        cv::Mat mask_roi = cv::Mat::zeros(image_threshold.size(), CV_8UC1);
        cv::rectangle(mask_roi, cv::Point(roi_detect_forklift[0], roi_detect_forklift[1]), cv::Point(roi_detect_forklift[2], roi_detect_forklift[3]), 255, -1);

        cv::bitwise_and(image_threshold, mask_roi, image_threshold);

        if (erode_size > 0 && dilate_size > 0)
        {
            cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_size, erode_size));
            cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));
            cv::erode(image_threshold, image_threshold, element_erode);
            cv::dilate(image_threshold, image_threshold, element_dilate);
        }

        // Morph closing
        cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
        cv::morphologyEx(image_threshold, image_threshold, cv::MORPH_CLOSE, close_kernel);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(image_threshold, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Rect> forklift_rects;
        float max_size = 0;
        for (const auto &contour : contours)
        {
            if (cv::contourArea(contour) > threshold_forklift_px_size)
            {
                cv::Rect rect = cv::boundingRect(contour);
                forklift_rects.push_back(rect);
                cv::rectangle(frame_bgr_copy, rect, cv::Scalar(0, 0, 255), 2);
            }
            if (cv::contourArea(contour) > max_size)
            {
                max_size = cv::contourArea(contour);
            }
        }

        cv::rectangle(frame_bgr_copy, cv::Point(roi_detect_forklift[0], roi_detect_forklift[1]), cv::Point(roi_detect_forklift[2], roi_detect_forklift[3]), cv::Scalar(0, 255, 0), 2);

        // // OCR with persistent tess_
        // tess_->SetSourceResolution(300); // or whatever DPI makes sense for your camera
        // tess_->SetImage(image_threshold.data, image_threshold.cols, image_threshold.rows, 1, image_threshold.step);
        // std::unique_ptr<char[]> raw(tess_->GetUTF8Text());
        // std::string text = raw ? std::string(raw.get()) : "";

        // // Draw OCR text
        // cv::putText(frame_bgr_copy, text, cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        std_msgs::msg::Float32 msg_forklift_detected;
        msg_forklift_detected.data = max_size;
        pub_forklift_detected->publish(msg_forklift_detected);

        auto msg_frame_display = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr_copy).toImageMsg();
        pub_frame_display->publish(*msg_frame_display);

        auto msg_frame_binary = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_threshold).toImageMsg();
        pub_frame_binary->publish(*msg_frame_binary);
    }

    // ========================================================================

    void aruco_detection_bgr()
    {
        if (!is_frame_bgr_available)
        {
            // error_code = 5;
            return;
        }

        cv::Mat frame_bgr_copy;
        mutex_frame_bgr.lock();
        if (!frame_bgr.empty())
        {
            try
            {
                frame_bgr_copy = frame_bgr.clone();
                mutex_frame_bgr.unlock();
            }
            catch (const cv::Exception &e)
            {
                error_code = 4;
                logger.error("Failed to clone image: %s", e.what());
                mutex_frame_bgr.unlock();
            }
        }

        if (frame_bgr_copy.empty())
        {
            // error_code = 5;
            return;
        }

        error_code = 0;

        static int aruco_detection_in = 0;
        static int aruco_detection_out = 0;

        // Variables to hold detection results
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        // Detect the ArUco markers
        cv::aruco::detectMarkers(frame_bgr_copy, aruco_dictionary, markerCorners, markerIds);

        //==================================================================================================

        // Find the nearest aruco from setpoint
        int nearest_marker_id = -1;
        if (markerIds.size() > 0)
        {
            float nearest_marker_distance = FLT_MAX;
            for (size_t i = 0; i < markerIds.size(); i++)
            {
                cv::Point2f center_marker = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) * 0.25;
                float distance = sqrt(pow(setpoint_x - center_marker.x, 2) + pow(setpoint_y - center_marker.y, 2));
                if (distance < nearest_marker_distance)
                {
                    nearest_marker_distance = distance;
                    nearest_marker_id = markerIds[i];
                }
            }

            if (nearest_marker_distance < min_aruco_range)
            {
                aruco_detection_in += 2;
                aruco_detection_out = 0;

                // Anti winding system
                if (aruco_detection_in > 100)
                    aruco_detection_in = 100;
            }

            cv::aruco::drawDetectedMarkers(frame_bgr_copy, markerCorners, markerIds);

            if (nearest_marker_id > -1)
            {
                cv::putText(frame_bgr_copy, std::to_string(nearest_marker_distance), cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            }
        }
        else
        {
            aruco_detection_in--;
            aruco_detection_out++;

            // Anti winding system
            if (aruco_detection_in < 0)
                aruco_detection_in = 0;

            if (aruco_detection_out > 100)
                aruco_detection_out = 100;
        }

        //================================================================================================

        if (aruco_detection_in > aruco_in_counter_threshold)
            cv::circle(frame_bgr_copy, cv::Point(setpoint_x, setpoint_y), 25, cv::Scalar(0, 255, 255), -1);
        else if (aruco_detection_out > aruco_out_counter_threshold)
            cv::circle(frame_bgr_copy, cv::Point(setpoint_x, setpoint_y), 25, cv::Scalar(255, 255, 0), -1);

        //================================================================================================

        std_msgs::msg::Bool msg_aruco_detected;
        if (aruco_detection_in > aruco_in_counter_threshold)
            msg_aruco_detected.data = true;
        else if (aruco_detection_out > aruco_out_counter_threshold)
            msg_aruco_detected.data = false;
        pub_aruco_detected->publish(msg_aruco_detected);

        auto msg_frame_display = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr_copy).toImageMsg();
        pub_frame_display->publish(*msg_frame_display);

        std_msgs::msg::Int16 msg_aruco_nearest_marker_id;
        msg_aruco_nearest_marker_id.data = (int16_t)nearest_marker_id;
        pub_aruco_nearest_marker_id->publish(msg_aruco_nearest_marker_id);
    }

    void aruco_detection_gray()
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

        static int aruco_detection_in = 0;
        static int aruco_detection_out = 0;

        // Variables to hold detection results
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        // Detect the ArUco markers
        cv::aruco::detectMarkers(frame_gray_copy, aruco_dictionary, markerCorners, markerIds);

        //================================================================================================

        // Find the nearest aruco from setpoint
        if (markerIds.size() > 0)
        {
            int nearest_marker_id = -1;
            float nearest_marker_distance = FLT_MAX;
            for (size_t i = 0; i < markerIds.size(); i++)
            {
                cv::Point2f center_marker = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3]) * 0.25;
                float distance = sqrt(pow(setpoint_x - center_marker.x, 2) + pow(setpoint_y - center_marker.y, 2));
                if (distance < nearest_marker_distance)
                {
                    nearest_marker_distance = distance;
                    nearest_marker_id = markerIds[i];
                }
            }

            if (nearest_marker_distance < min_aruco_range)
            {
                aruco_detection_in += 2;
                aruco_detection_out = 0;

                // Anti winding system
                if (aruco_detection_in > 100)
                    aruco_detection_in = 100;
            }

            cv::aruco::drawDetectedMarkers(frame_gray_copy, markerCorners, markerIds);

            if (nearest_marker_id > -1)
            {
                cv::putText(frame_gray_copy, std::to_string(nearest_marker_distance), cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
            }
        }
        else
        {
            aruco_detection_in--;
            aruco_detection_out++;

            // Anti winding system
            if (aruco_detection_in < 0)
                aruco_detection_in = 0;

            if (aruco_detection_out > 100)
                aruco_detection_out = 100;
        }

        //================================================================================================

        if (aruco_detection_in > aruco_in_counter_threshold)
            cv::circle(frame_gray_copy, cv::Point(setpoint_x, setpoint_y), 25, cv::Scalar(255), -1);
        else if (aruco_detection_out > aruco_out_counter_threshold)
            cv::circle(frame_gray_copy, cv::Point(setpoint_x, setpoint_y), 25, cv::Scalar(0), -1);

        //================================================================================================

        std_msgs::msg::Bool msg_aruco_detected;
        if (aruco_detection_in > aruco_in_counter_threshold)
            msg_aruco_detected.data = true;
        else if (aruco_detection_out > aruco_out_counter_threshold)
            msg_aruco_detected.data = false;
        pub_aruco_detected->publish(msg_aruco_detected);

        auto msg_frame_display = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame_gray_copy).toImageMsg();
        pub_frame_display->publish(*msg_frame_display);
    }

    void process_frame_bgr()
    {
        if (!is_frame_bgr_available)
        {
            error_code = 5;
            return;
        }

        cv::Mat frame_bgr_copy;
        mutex_frame_bgr.lock();
        if (!frame_bgr.empty())
        {
            try
            {
                frame_bgr_copy = frame_bgr.clone();
                is_frame_bgr_available = false;
                mutex_frame_bgr.unlock();
            }
            catch (const cv::Exception &e)
            {
                error_code = 4;
                logger.error("Failed to clone image: %s", e.what());
                mutex_frame_bgr.unlock();
            }
        }

        if (frame_bgr_copy.empty())
        {
            error_code = 5;
            return;
        }

        static rclcpp::Time last_time = this->now();
        rclcpp::Duration elapsed_time = this->now() - last_time;
        last_time = this->now();
        pid_point_emergency.set_dt(elapsed_time.seconds());

        //================================================================================================

        static float output_pid_point_emergency = 0;
        static float sudut_setpoint_digunakan = 0;
        static float sudut_point_terdekat_digunakan = 0;
        static float target_velocity_gain = 0.0;
        uint8_t status_valid = 0;

        //================================================================================================

        cv::Mat lab;
        cv::cvtColor(frame_bgr_copy, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> lab_channels;
        cv::split(lab, lab_channels);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(4, 4));
        clahe->apply(lab_channels[0], lab_channels[0]);
        cv::merge(lab_channels, lab);
        cv::cvtColor(lab, frame_bgr_copy, cv::COLOR_Lab2BGR);

        // cv::Mat frame_hls;
        cv::Mat frame_hsv;
        // cv::cvtColor(frame_bgr_copy, frame_hls, cv::COLOR_BGR2HLS);
        cv::cvtColor(frame_bgr_copy, frame_hsv, cv::COLOR_BGR2HSV);
        cv::Mat image_threshold;
        cv::inRange(frame_hsv, cv::Scalar(low_h, low_l, low_s), cv::Scalar(high_h, high_l, high_s), image_threshold);

        if (erode_size > 0 && dilate_size)
        {
            cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_size, erode_size));
            cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));
            cv::erode(image_threshold, image_threshold, element_erode);
            cv::dilate(image_threshold, image_threshold, element_dilate);
        }

        cv::circle(frame_bgr_copy, cv::Point(setpoint_x, setpoint_y), 25, cv::Scalar(255, 255, 0), -1);

        if (pangkal_x != 9999 && pangkal_y != 9999)
            cv::circle(frame_bgr_copy, cv::Point(pangkal_x, pangkal_y), 15, cv::Scalar(200, 200, 0), -1);

        //================================================================================================

        float error_jarak_terdekat = FLT_MAX;
        int point_terdekat_setpoint_x = 0;
        int point_terdekat_setpoint_y = 0;
        // float sudut_garis_inisialisasi = 0;
        float target_velocity_gain_buffer = 0;
        float jarak_point0_ke_setpoint = 0;

        float mean_buffer_sudut_point0_ke_point_i = 0;
        int mean_buffer_sudut_point0_ke_point_i_count = 0;

        // Filter point, index 0  berarti terdekat dengan mobil
        std::vector<cv::Point> point_garis;

        // Scan from max height to 0
        if (right_to_left_scan == true)
        {
            for (int j = image_threshold.cols - 1; j >= 0; j -= 10)
            {
                for (int i = image_threshold.rows - 1; i >= 0; i -= 10)
                {
                    if (image_threshold.at<uchar>(i, j) == 255)
                    {
                        // Mencatat point garis
                        point_garis.push_back(cv::Point(j, i));

                        // Ketika sudah menemukan 1 point, maka diangap sebagai point initial (point0)
                        if (point_garis.size() == 1)
                        {
                            jarak_point0_ke_setpoint = sqrt(pow(setpoint_x - point_garis[0].x, 2) + pow(setpoint_y - point_garis[0].y, 2));
                        }

                        // Mencari point terdekat dengan setpoint (based on angular scan) (Jarak point mendekati sama dengan jarak ke setpoint)
                        float jarak_point0_ke_point_i = sqrt(pow(point_garis[0].x - j, 2) + pow(point_garis[0].y - i, 2));
                        float error_jarak = fabs(jarak_point0_ke_point_i - jarak_point0_ke_setpoint);

                        if (error_jarak < error_jarak_terdekat)
                        {
                            error_jarak_terdekat = error_jarak;
                            point_terdekat_setpoint_x = j;
                            point_terdekat_setpoint_y = i;

                            float jarak_point_i_ke_setpoint = sqrt(pow(setpoint_x - j, 2) + pow(setpoint_y - i, 2));
                            if (jarak_point_i_ke_setpoint < maximum_error_jarak_setpoint)
                                status_valid = 1;
                        }

                        // Menghitung target velocity
                        if (point_garis.size() > 1)
                        {
                            float sudut_point0_ke_point_i = atan2(j - point_garis[point_garis.size() - 1].y, i - point_garis[point_garis.size() - 1].x);
                            mean_buffer_sudut_point0_ke_point_i += sudut_point0_ke_point_i;
                            mean_buffer_sudut_point0_ke_point_i_count++;
                            float mean_sudut_point0_ke_point_i = mean_buffer_sudut_point0_ke_point_i / mean_buffer_sudut_point0_ke_point_i_count;
                            float sudut_error = mean_sudut_point0_ke_point_i - sudut_point0_ke_point_i;
                            while (sudut_error > M_PI)
                                sudut_error -= 2 * M_PI;
                            while (sudut_error < -M_PI)
                                sudut_error += 2 * M_PI;

                            if (fabs(sudut_error) < point_to_velocity_angle_threshold)
                            {
                                target_velocity_gain_buffer += point_to_velocity_ratio;
                                cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 255, 255), 3);
                            }
                            else
                            {
                                cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
                            }
                        }
                        else
                        {
                            cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
                        }

                        break;
                    }
                }
            }
        }
        else
        {
            for (int j = 0; j < image_threshold.cols - 1; j += 10)
            {
                for (int i = image_threshold.rows - 1; i >= 0; i -= 10)
                {
                    if (image_threshold.at<uchar>(i, j) == 255)
                    {
                        // Mencatat point garis
                        point_garis.push_back(cv::Point(j, i));

                        // Ketika sudah menemukan 1 point, maka diangap sebagai point initial (point0)
                        if (point_garis.size() == 1)
                        {
                            jarak_point0_ke_setpoint = sqrt(pow(setpoint_x - point_garis[0].x, 2) + pow(setpoint_y - point_garis[0].y, 2));
                        }

                        // Mencari point terdekat dengan setpoint (based on angular scan) (Jarak point mendekati sama dengan jarak ke setpoint)
                        float jarak_point0_ke_point_i = sqrt(pow(point_garis[0].x - j, 2) + pow(point_garis[0].y - i, 2));
                        float error_jarak = fabs(jarak_point0_ke_point_i - jarak_point0_ke_setpoint);

                        if (error_jarak < error_jarak_terdekat)
                        {
                            error_jarak_terdekat = error_jarak;
                            point_terdekat_setpoint_x = j;
                            point_terdekat_setpoint_y = i;

                            float jarak_point_i_ke_setpoint = sqrt(pow(setpoint_x - j, 2) + pow(setpoint_y - i, 2));
                            if (jarak_point_i_ke_setpoint < maximum_error_jarak_setpoint)
                                status_valid = 1;
                        }

                        // Menghitung target velocity
                        if (point_garis.size() > 1)
                        {
                            float sudut_point0_ke_point_i = atan2(j - point_garis[point_garis.size() - 1].y, i - point_garis[point_garis.size() - 1].x);
                            mean_buffer_sudut_point0_ke_point_i += sudut_point0_ke_point_i;
                            mean_buffer_sudut_point0_ke_point_i_count++;
                            float mean_sudut_point0_ke_point_i = mean_buffer_sudut_point0_ke_point_i / mean_buffer_sudut_point0_ke_point_i_count;
                            float sudut_error = mean_sudut_point0_ke_point_i - sudut_point0_ke_point_i;
                            while (sudut_error > M_PI)
                                sudut_error -= 2 * M_PI;
                            while (sudut_error < -M_PI)
                                sudut_error += 2 * M_PI;

                            if (fabs(sudut_error) < point_to_velocity_angle_threshold)
                            {
                                target_velocity_gain_buffer += point_to_velocity_ratio;
                                cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 255, 255), 3);
                            }
                            else
                            {
                                cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
                            }
                        }
                        else
                        {
                            cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
                        }

                        break;
                    }
                }
            }
        }

        // for (int i = image_threshold.rows - 1; i >= 0; i--)
        // {
        //     if (right_to_left_scan == false)
        //     {
        //         for (int j = 0; j < image_threshold.cols; j++)
        //         {
        //             if (image_threshold.at<uchar>(i, j) == 255)
        //             {
        //                 // Mencatat point garis
        //                 point_garis.push_back(cv::Point(j, i));

        //                 // Ketika sudah menemukan 1 point, maka diangap sebagai point initial (point0)
        //                 if (point_garis.size() == 1)
        //                 {
        //                     jarak_point0_ke_setpoint = sqrt(pow(setpoint_x - point_garis[0].x, 2) + pow(setpoint_y - point_garis[0].y, 2));
        //                 }

        //                 // Mencari point terdekat dengan setpoint (based on angular scan) (Jarak point mendekati sama dengan jarak ke setpoint)
        //                 float jarak_point0_ke_point_i = sqrt(pow(point_garis[0].x - j, 2) + pow(point_garis[0].y - i, 2));
        //                 float error_jarak = fabs(jarak_point0_ke_point_i - jarak_point0_ke_setpoint);

        //                 if (error_jarak < error_jarak_terdekat)
        //                 {
        //                     error_jarak_terdekat = error_jarak;
        //                     point_terdekat_setpoint_x = j;
        //                     point_terdekat_setpoint_y = i;

        //                     float jarak_point_i_ke_setpoint = sqrt(pow(setpoint_x - j, 2) + pow(setpoint_y - i, 2));
        //                     if (jarak_point_i_ke_setpoint < maximum_error_jarak_setpoint)
        //                         status_valid = 1;
        //                 }

        //                 // Menghitung target velocity
        //                 if (point_garis.size() > 1)
        //                 {
        //                     float sudut_point0_ke_point_i = atan2(i - point_garis[point_garis.size() - 2].y, j - point_garis[point_garis.size() - 2].x);
        //                     mean_buffer_sudut_point0_ke_point_i += sudut_point0_ke_point_i;
        //                     mean_buffer_sudut_point0_ke_point_i_count++;
        //                     float mean_sudut_point0_ke_point_i = mean_buffer_sudut_point0_ke_point_i / mean_buffer_sudut_point0_ke_point_i_count;
        //                     float sudut_error = mean_sudut_point0_ke_point_i - sudut_point0_ke_point_i;
        //                     while (sudut_error > M_PI)
        //                         sudut_error -= 2 * M_PI;
        //                     while (sudut_error < -M_PI)
        //                         sudut_error += 2 * M_PI;

        //                     if (fabs(sudut_error) < point_to_velocity_angle_threshold)
        //                     {
        //                         target_velocity_gain_buffer += point_to_velocity_ratio;
        //                         cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 255, 255), 3);
        //                     }
        //                     else
        //                     {
        //                         cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
        //                     }
        //                 }
        //                 else
        //                 {
        //                     cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
        //                 }

        //                 break;
        //             }
        //         }
        //     }
        //     else
        //     {
        //         for (int j = image_threshold.cols - 1; j >= 0; j--)
        //         {
        //             if (image_threshold.at<uchar>(i, j) == 255)
        //             {
        //                 // Mencatat point garis
        //                 point_garis.push_back(cv::Point(j, i));

        //                 // Ketika sudah menemukan 1 point, maka diangap sebagai point initial (point0)
        //                 if (point_garis.size() == 1)
        //                 {
        //                     jarak_point0_ke_setpoint = sqrt(pow(setpoint_x - point_garis[0].x, 2) + pow(setpoint_y - point_garis[0].y, 2));
        //                 }

        //                 // Mencari point terdekat dengan setpoint (based on angular scan) (Jarak point mendekati sama dengan jarak ke setpoint)
        //                 float jarak_point0_ke_point_i = sqrt(pow(point_garis[0].x - j, 2) + pow(point_garis[0].y - i, 2));
        //                 float error_jarak = fabs(jarak_point0_ke_point_i - jarak_point0_ke_setpoint);

        //                 if (error_jarak < error_jarak_terdekat)
        //                 {
        //                     error_jarak_terdekat = error_jarak;
        //                     point_terdekat_setpoint_x = j;
        //                     point_terdekat_setpoint_y = i;

        //                     float jarak_point_i_ke_setpoint = sqrt(pow(setpoint_x - j, 2) + pow(setpoint_y - i, 2));
        //                     if (jarak_point_i_ke_setpoint < maximum_error_jarak_setpoint)
        //                         status_valid = 1;
        //                 }

        //                 // Menghitung target velocity
        //                 if (point_garis.size() > 1)
        //                 {
        //                     float sudut_point0_ke_point_i = atan2(i - point_garis[point_garis.size() - 2].y, j - point_garis[point_garis.size() - 2].x);
        //                     mean_buffer_sudut_point0_ke_point_i += sudut_point0_ke_point_i;
        //                     mean_buffer_sudut_point0_ke_point_i_count++;
        //                     float mean_sudut_point0_ke_point_i = mean_buffer_sudut_point0_ke_point_i / mean_buffer_sudut_point0_ke_point_i_count;
        //                     float sudut_error = mean_sudut_point0_ke_point_i - sudut_point0_ke_point_i;
        //                     while (sudut_error > M_PI)
        //                         sudut_error -= 2 * M_PI;
        //                     while (sudut_error < -M_PI)
        //                         sudut_error += 2 * M_PI;

        //                     if (fabs(sudut_error) < point_to_velocity_angle_threshold)
        //                     {
        //                         target_velocity_gain_buffer += point_to_velocity_ratio;
        //                         cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 255, 255), 3);
        //                     }
        //                     else
        //                     {
        //                         cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
        //                     }
        //                 }
        //                 else
        //                 {
        //                     cv::circle(frame_bgr_copy, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 3);
        //                 }

        //                 break;
        //             }
        //         }
        //     }
        // }

        if (status_valid == 1)
            cv::circle(frame_bgr_copy, cv::Point(point_terdekat_setpoint_x, point_terdekat_setpoint_y), 8, cv::Scalar(255, 0, 255), -1);

        //================================================================================================

        /**
         * Mengihutng target kecepatan mobil,
         * Kecepatan mobil berdasarkan lane nya, jika lane lurus maka kecepatan maksimum
         * Jika lane belok maka kecepatan berkurang
         */
        if (target_velocity_gain_buffer > 0)
        {
            target_velocity_gain = fminf(target_velocity_gain_buffer, 1);
        }
        else
        {
            target_velocity_gain = target_velocity_gain * 0.9 + 0.01 * 0.1;
        }

        if (point_garis.size() > 1 && status_valid == 1)
        {
            /**
             * Ada 2 metode,
             * Yang pertama dengan metode angular scan seperti diatas
             * Yang kedua adalah metode perbandingan sudut yaitu sudut point0 ke setpoint dengan sudut point0 ke point_max
             */

            // Metode pertama
            if (metode_perhitungan == 1)
            {
                sudut_setpoint_digunakan = atan2(setpoint_y - point_garis[0].y, setpoint_x - point_garis[0].x);
                sudut_point_terdekat_digunakan = atan2(point_terdekat_setpoint_y - point_garis[0].y, point_terdekat_setpoint_x - point_garis[0].x);
                float sudut_error = sudut_setpoint_digunakan - sudut_point_terdekat_digunakan;

                while (sudut_error > M_PI)
                    sudut_error -= 2 * M_PI;
                while (sudut_error < -M_PI)
                    sudut_error += 2 * M_PI;

                if (right_to_left_scan == false)
                {
                    output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0) * -1;
                }
                else
                {
                    output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0);
                }

                // output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0);
            }

            // Metode kedua
            else if (metode_perhitungan == 2)
            {
                sudut_setpoint_digunakan = atan2(setpoint_y - point_garis[0].y, setpoint_x - point_garis[0].x);
                sudut_point_terdekat_digunakan = atan2(point_garis[point_garis.size() - 1].y - point_garis[0].y, point_garis[point_garis.size() - 1].x - point_garis[0].x);
                float sudut_error = sudut_setpoint_digunakan - sudut_point_terdekat_digunakan;

                while (sudut_error > M_PI)
                    sudut_error -= 2 * M_PI;
                while (sudut_error < -M_PI)
                    sudut_error += 2 * M_PI;

                if (right_to_left_scan == false)
                {
                    output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0) * -1;
                }
                else
                {
                    output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0);
                }

                // output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0);
            }
            else if (metode_perhitungan == 3)
            {
                sudut_setpoint_digunakan = atan2(setpoint_y - pangkal_y, setpoint_x - pangkal_x);
                sudut_point_terdekat_digunakan = atan2(point_terdekat_setpoint_y - pangkal_y, point_terdekat_setpoint_x - pangkal_x);
                float sudut_error = sudut_setpoint_digunakan - sudut_point_terdekat_digunakan;

                while (sudut_error > M_PI)
                    sudut_error -= 2 * M_PI;
                while (sudut_error < -M_PI)
                    sudut_error += 2 * M_PI;

                // if (right_to_left_scan == false)
                // {
                //     output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0) * -1;
                // }
                // else
                // {
                // }
                output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -1.0, 1.0);
            }
        }
        else
        {
            output_pid_point_emergency = pid_point_emergency.calculate(0, -1.0, 1.0);
        }

        //================================================================================================

        if (debug_mode)
        {
            image_threshold = cv::Mat::zeros(image_threshold.size(), CV_8UC1);
        }

        ros2_interface::msg::PointArray msg_point_garis;
        for (size_t i = 0; i < point_garis.size(); i++)
        {
            geometry_msgs::msg::Point p;

            if (!debug_mode)
            {
                p.x = point_garis[i].x - center_frame_x;
                p.y = center_frame_y - point_garis[i].y;
            }
            else
            {
                static const float arah_kamera_cos = cosf(rotation_angle);
                static const float arah_kamera_sin = sinf(rotation_angle);

                p.x = setpoint_x + (point_garis[i].x - setpoint_x) * arah_kamera_cos - (point_garis[i].y - setpoint_y) * arah_kamera_sin;
                p.y = setpoint_y + (point_garis[i].x - setpoint_x) * arah_kamera_sin + (point_garis[i].y - setpoint_y) * arah_kamera_cos;

                if (p.x > 0 && p.x < image_threshold.cols && p.y > 0 && p.y < image_threshold.rows)
                {
                    image_threshold.at<uchar>((int)p.y, (int)p.x) = 255;
                }

                p.x = p.x - center_frame_x;
                p.y = center_frame_y - p.y;
            }

            msg_point_garis.points.push_back(p);
        }
        pub_point_garis->publish(msg_point_garis);

        std_msgs::msg::Float32MultiArray msg_hasil_kalkulasi;
        msg_hasil_kalkulasi.data.push_back(output_pid_point_emergency);
        msg_hasil_kalkulasi.data.push_back(sudut_setpoint_digunakan);
        msg_hasil_kalkulasi.data.push_back(sudut_point_terdekat_digunakan);
        msg_hasil_kalkulasi.data.push_back(target_velocity_gain);
        pub_hasil_kalkulasi->publish(msg_hasil_kalkulasi);

        // cv::circle(frame_bgr_copy, cv::Point(setpoint_x, setpoint_y), 25, cv::Scalar(255, 255, 0), -1);

        //================================================================================================

        auto msg_frame_display = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr_copy).toImageMsg();
        pub_frame_display->publish(*msg_frame_display);

        auto msg_frame_binary = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_threshold).toImageMsg();
        pub_frame_binary->publish(*msg_frame_binary);
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
        if (center_frame_x == 0 && center_frame_y == 0)
        {
            center_frame_x = frame_bgr.cols / 2;
            center_frame_y = frame_bgr.rows / 2;
        }

        // logger.info("%d %d %d", is_detect_forklift, use_frame_bgr, detect_aruco);
        if (is_detect_forklift)
        {
            forklift_detection_bgr();
        }
        else
        {
            if (use_frame_bgr && !detect_aruco)
            {
                process_frame_bgr();
            }
            else if (use_frame_bgr && detect_aruco)
            {
                aruco_detection_bgr();
            }
            else if (!use_frame_bgr && detect_aruco)
            {
                aruco_detection_gray();
            }
            else
            {
                process_frame_gray();
            }
        }

        std_msgs::msg::Int16 msg_error_code;
        msg_error_code.data = error_code;
        pub_error_code->publish(msg_error_code);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_single_detection = std::make_shared<SingleDetection>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_single_detection);
    executor.spin();

    return 0;
}