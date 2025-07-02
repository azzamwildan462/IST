// src/forklift_detector.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <regex>

using std::placeholders::_1;

class ForkliftDetector : public rclcpp::Node
{
public:
    ForkliftDetector()
        : Node("forklift_detector"),
          hsv_lower_(100, 80, 50),
          hsv_upper_(140, 255, 255),
          morph_kernel_(cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7))),
          code_regex_("E(\\d+)")
    {
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
        tess_->SetVariable("tessedit_char_whitelist", "E0123456789");

        // 2) ROS2 interfaces
        pub_ = create_publisher<std_msgs::msg::Int8>("forklift_number", 1);
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/rs2_cam_main/color/image_raw",
            rclcpp::QoS(5).best_effort(),
            std::bind(&ForkliftDetector::imageCallback, this, _1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert to OpenCV BGR
        cv::Mat src;
        try
        {
            src = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        // Detect number
        int8_t number = static_cast<int8_t>(detectForkliftNumber(src));

        // Publish
        auto out = std_msgs::msg::Int8();
        out.data = number;
        pub_->publish(out);
    }

    int detectForkliftNumber(const cv::Mat &src)
    {
        // a) isolate blue regions
        cv::Mat hsv, mask;
        cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, hsv_lower_, hsv_upper_, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_);

        // b) find largest contour
        std::vector<std::vector<cv::Point>> ctrs;
        cv::findContours(mask, ctrs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (ctrs.empty())
            return -1;
        auto best = std::max_element(ctrs.begin(), ctrs.end(),
                                     [](auto &a, auto &b)
                                     { return cv::contourArea(a) < cv::contourArea(b); });
        cv::Rect roi = cv::boundingRect(*best);
        if (roi.area() < 1000)
            return -1;

        // c) preprocess for OCR
        cv::Mat gray, bin;
        cv::cvtColor(src(roi), gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        // d) OCR with persistent tess_
        tess_->SetImage(bin.data, bin.cols, bin.rows, 1, bin.step);
        std::unique_ptr<char[]> raw(tess_->GetUTF8Text());
        std::string text = raw ? std::string(raw.get()) : "";

        // e) regex “E” + digits
        std::smatch m;
        if (std::regex_search(text, m, code_regex_) && m.size() >= 2)
        {
            try
            {
                return std::stoi(m.str(1));
            }
            catch (...)
            { /* fall through */
            }
        }
        return -1;
    }

    // ROS interfaces
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    // Tesseract engine
    std::unique_ptr<tesseract::TessBaseAPI> tess_;

    // HSV mask parameters
    const cv::Scalar hsv_lower_, hsv_upper_;
    const cv::Mat morph_kernel_;

    // Regex for matching E<number>
    const std::regex code_regex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForkliftDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
