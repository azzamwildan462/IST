#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/int16.hpp"

class VisionCapture : public rclcpp::Node {
public:
    //----Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_bgr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_gray;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;

    // Configs
    // =======================================================
    std::string camera_path;
    int camera_fps;
    int camera_width;
    int camera_height;
    int output_width;
    int output_height;
    bool use_default_params;
    std::string node_namespace = "";

    HelpLogger logger;
    cv::VideoCapture cap;
    int error_code = 0;

    VisionCapture()
        : Node("vision_capture")
    {
        this->declare_parameter("camera_path", "/dev/video0");
        this->get_parameter("camera_path", camera_path);

        this->declare_parameter("camera_fps", 30);
        this->get_parameter("camera_fps", camera_fps);

        this->declare_parameter("camera_width", 640);
        this->get_parameter("camera_width", camera_width);

        this->declare_parameter("camera_height", 480);
        this->get_parameter("camera_height", camera_height);

        this->declare_parameter("output_width", 640);
        this->get_parameter("output_width", output_width);

        this->declare_parameter("output_height", 480);
        this->get_parameter("output_height", output_height);

        this->declare_parameter("use_default_params", true);
        this->get_parameter("use_default_params", use_default_params);

        node_namespace = this->get_namespace();
        node_namespace = node_namespace.substr(1, node_namespace.size() - 1); // /cam_kanan jadi cam_kanan

        if (!logger.init()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        logger.info("Init camera on: %s", camera_path.c_str());

        if (!cap.open(camera_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera, Retry in 3 seconds");
            rclcpp::sleep_for(std::chrono::seconds(3));
            rclcpp::shutdown();
        }

        //----Publisher
        pub_image_bgr = this->create_publisher<sensor_msgs::msg::Image>("image_bgr", 1);
        pub_image_gray = this->create_publisher<sensor_msgs::msg::Image>("image_gray", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("error_code", 1);

        logger.info("VisionCapture init success");
    }

    void init_camera_params()
    {
        if (use_default_params)
            return;
        int camera_fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

        cap.set(cv::CAP_PROP_FOURCC, camera_fourcc);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);
        cap.set(cv::CAP_PROP_FPS, camera_fps);

        int _fourcc = cap.get(cv::CAP_PROP_FOURCC);
        int _width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int _height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        int _fps = cap.get(cv::CAP_PROP_FPS);

        if (camera_fourcc != _fourcc) {
            logger.warn("Failed to set camera fourcc");
            error_code = 1;
        }
        if (camera_width != _width) {
            logger.warn("Failed to set camera width");
            error_code = 2;
        }
        if (camera_height != _height) {
            logger.warn("Failed to set camera height");
            error_code = 3;
        }
        if (camera_fps != _fps) {
            logger.warn("Failed to set camera fps");
            error_code = 4;
        }
    }

    void callback_routine()
    {
        while (rclcpp::ok()) {
            cv::Mat frame;
            cap >> frame;

            if (frame.empty()) {
                logger.error("Failed to capture frame");
                error_code = 11;
            }

            // Hardcode sementara, image dari file
            // cv::Mat frame2 = cv::imread("/home/wildan/proyek/robotika/IST/src/vision/assets/test_lane.webp");
            // cv::Mat frame2 = cv::imread("/home/wildan/proyek/robotika/IST/src/vision/assets/" + node_namespace + ".jpeg");
            // cv::Mat frame2 = cv::imread("/home/wildan/proyek/robotika/IST/src/vision/assets/aruco_kanan.jpeg");

            // cv::Mat flippedImg;
            // cv::flip(frame2, flippedImg, 1); // 1 specifies flipping around the Y-axis

            cv::Mat frame_bgr;
            cv::resize(frame, frame_bgr, cv::Size(output_width, output_height));
            auto msg_frame_bgr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr).toImageMsg();
            pub_image_bgr->publish(*msg_frame_bgr);

            cv::Mat frame_gray;
            cv::cvtColor(frame_bgr, frame_gray, cv::COLOR_BGR2GRAY);
            auto msg_frame_gray = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame_gray).toImageMsg();
            pub_image_gray->publish(*msg_frame_gray);

            std_msgs::msg::Int16 msg_error_code;
            msg_error_code.data = error_code;
            pub_error_code->publish(msg_error_code);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node_vision_capture = std::make_shared<VisionCapture>();
    node_vision_capture->init_camera_params();
    node_vision_capture->callback_routine();
    rclcpp::shutdown();

    return 0;
}