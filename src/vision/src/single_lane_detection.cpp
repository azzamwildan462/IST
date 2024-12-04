#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/pid.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <thread>
#include <mutex>
#include "ros2_interface/msg/point_array.hpp"
#include "ros2_interface/srv/params.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class SingleLaneDetection : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_gray;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_bgr;
    rclcpp::Publisher<ros2_interface::msg::PointArray>::SharedPtr pub_point_garis;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_frame_display;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_frame_binary;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_hasil_kalkulasi;

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

    // Configs (static)
    // =======================================================
    std::string config_path;
    bool use_dynamic_config = false;
    bool use_frame_bgr = true;
    int metode_perhitungan = 2;
    int erode_size = 3;
    int dilate_size = 3;
    std::string node_namespace = "";
    float point_to_velocity_ratio = 0.05;
    float point_to_velocity_angle_threshold = 0.02;

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

    SingleLaneDetection() : Node("single_lane_detection")
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

        this->declare_parameter("metode_perhitungan", 2);
        this->get_parameter("metode_perhitungan", metode_perhitungan);

        this->declare_parameter("erode_size", 3);
        this->get_parameter("erode_size", erode_size);

        this->declare_parameter("dilate_size", 3);
        this->get_parameter("dilate_size", dilate_size);

        this->declare_parameter("point_to_velocity_ratio", 0.05);
        this->get_parameter("point_to_velocity_ratio", point_to_velocity_ratio);

        this->declare_parameter("point_to_velocity_angle_threshold", 0.02);
        this->get_parameter("point_to_velocity_angle_threshold", point_to_velocity_angle_threshold);

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

        if (use_frame_bgr)
        {
            sub_image_bgr = this->create_subscription<sensor_msgs::msg::Image>(
                "image_bgr", 1, std::bind(&SingleLaneDetection::callback_sub_image_bgr, this, std::placeholders::_1));
        }
        else
        {
            sub_image_gray = this->create_subscription<sensor_msgs::msg::Image>(
                "image_gray", 1, std::bind(&SingleLaneDetection::callback_sub_image_gray, this, std::placeholders::_1));
        }

        pub_point_garis = this->create_publisher<ros2_interface::msg::PointArray>("point_garis", 1);
        pub_hasil_kalkulasi = this->create_publisher<std_msgs::msg::Float32MultiArray>("hasil_kalkulasi", 1);

        pub_frame_display = this->create_publisher<sensor_msgs::msg::Image>("frame_display", 1);
        pub_frame_binary = this->create_publisher<sensor_msgs::msg::Image>("frame_binary", 1);

        srv_params = this->create_service<ros2_interface::srv::Params>(
            "params", std::bind(&SingleLaneDetection::callback_srv_params, this, std::placeholders::_1, std::placeholders::_2));

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SingleLaneDetection::callback_tim_50hz, this));

        pid_point_emergency.init(0.5, 0.0001, 0.01, 0.02, 0.00, 10, 0.00, 0.001);

        logger.info("SingleLaneDetection init success");
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

        low_h = config["Detection"][node_namespace.c_str()]["low_h"].as<int>();
        low_l = config["Detection"][node_namespace.c_str()]["low_l"].as<int>();
        low_s = config["Detection"][node_namespace.c_str()]["low_s"].as<int>();
        high_h = config["Detection"][node_namespace.c_str()]["high_h"].as<int>();
        high_l = config["Detection"][node_namespace.c_str()]["high_l"].as<int>();
        high_s = config["Detection"][node_namespace.c_str()]["high_s"].as<int>();
        // setpoint_x = config["Detection"][node_namespace.c_str()]["setpoint_x"].as<int>();
        // setpoint_y = config["Detection"][node_namespace.c_str()]["setpoint_y"].as<int>();
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
        // config["Detection"][node_namespace.c_str()]["setpoint_x"] = setpoint_x;
        // config["Detection"][node_namespace.c_str()]["setpoint_y"] = setpoint_y;

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

        static rclcpp::Time last_time = this->now();
        rclcpp::Duration elapsed_time = this->now() - last_time;
        last_time = this->now();
        pid_point_emergency.set_dt(elapsed_time.seconds());

        //================================================================================================

        static float output_pid_point_emergency = 0;
        static float sudut_setpoint_digunakan = 0;
        static float sudut_point_terdekat_digunakan = 0;
        static float target_velocity_gain = 0.0;

        //================================================================================================

        cv::Mat frame_hls;
        cv::cvtColor(frame_bgr_copy, frame_hls, cv::COLOR_BGR2HLS);
        cv::Mat image_hls_threshold;
        cv::inRange(frame_hls, cv::Scalar(low_h, low_l, low_s), cv::Scalar(high_h, high_l, high_s), image_hls_threshold);

        if (erode_size > 0 && dilate_size)
        {
            cv::Mat element_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_size, erode_size));
            cv::Mat element_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_size, dilate_size));
            cv::erode(image_hls_threshold, image_hls_threshold, element_erode);
            cv::dilate(image_hls_threshold, image_hls_threshold, element_dilate);
        }

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
        for (int i = image_hls_threshold.rows - 1; i >= 0; i--)
        {
            for (int j = 0; j < image_hls_threshold.cols; j++)
            {
                if (image_hls_threshold.at<uchar>(i, j) == 255)
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
                    }

                    // Menghitung target velocity
                    if (point_garis.size() > 1)
                    {
                        float sudut_point0_ke_point_i = atan2(i - point_garis[point_garis.size() - 2].y, j - point_garis[point_garis.size() - 2].x);
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

        if (point_garis.size() > 1)
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

                output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -0.5, 0.5);
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

                output_pid_point_emergency = pid_point_emergency.calculate(sudut_error, -0.5, 0.5);
            }
        }
        else
        {
            output_pid_point_emergency = pid_point_emergency.calculate(0, -0.5, 0.5);
        }

        //================================================================================================

        ros2_interface::msg::PointArray msg_point_garis;
        for (size_t i = 0; i < point_garis.size(); i++)
        {
            geometry_msgs::msg::Point p;
            p.x = point_garis[i].x;
            p.y = point_garis[i].y;
            msg_point_garis.points.push_back(p);
        }
        pub_point_garis->publish(msg_point_garis);

        std_msgs::msg::Float32MultiArray msg_hasil_kalkulasi;
        msg_hasil_kalkulasi.data.push_back(output_pid_point_emergency);
        msg_hasil_kalkulasi.data.push_back(sudut_setpoint_digunakan);
        msg_hasil_kalkulasi.data.push_back(sudut_point_terdekat_digunakan);
        msg_hasil_kalkulasi.data.push_back(target_velocity_gain);
        pub_hasil_kalkulasi->publish(msg_hasil_kalkulasi);

        cv::circle(frame_bgr_copy, cv::Point(setpoint_x, setpoint_y), 25, cv::Scalar(255, 255, 0), -1);

        //================================================================================================

        auto msg_frame_display = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr_copy).toImageMsg();
        pub_frame_display->publish(*msg_frame_display);

        auto msg_frame_binary = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_hls_threshold).toImageMsg();
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

    auto node_single_lane_detection = std::make_shared<SingleLaneDetection>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_single_lane_detection);
    executor.spin();

    return 0;
}