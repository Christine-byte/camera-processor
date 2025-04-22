#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>  // JSON 库

using json = nlohmann::json;
using namespace std::chrono;

class CameraProcessor : public rclcpp::Node
{
public:
    CameraProcessor() : Node("camera_processor")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/color/image", 10,
            std::bind(&CameraProcessor::image_callback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/stereo/depth", 10,
            std::bind(&CameraProcessor::depth_callback, this, std::placeholders::_1));

        trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/trigger_photo", 10,
            std::bind(&CameraProcessor::trigger_callback, this, std::placeholders::_1));

        std::filesystem::create_directories("captured");

        RCLCPP_INFO(this->get_logger(), "📸 Camera Processor Node started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;

    cv::Mat last_rgb_;
    cv::Mat last_depth_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            last_rgb_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "❌ RGB conversion error: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            last_depth_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Depth conversion error: %s", e.what());
        }
    }

    void trigger_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        if (last_rgb_.empty() || last_depth_.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ Image or depth not ready.");
            return;
        }

        auto now = system_clock::now();
        auto timestamp = duration_cast<seconds>(now.time_since_epoch()).count();
        std::string time_str = get_time_string();

        std::string image_path = "captured/image_" + time_str + ".jpg";
        std::string json_path  = "captured/image_" + time_str + ".json";

        // Step 1: HSV 橙色检测
        cv::Mat hsv, mask;
        cv::cvtColor(last_rgb_, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(5, 100, 100), cv::Scalar(15, 255, 255), mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool object_found = !contours.empty();
        int cx = -1, cy = -1;
        float depth_m = -1;

        if (object_found) {
            auto largest = std::max_element(contours.begin(), contours.end(),
                [](const auto &a, const auto &b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            cv::Moments M = cv::moments(*largest);
            cx = static_cast<int>(M.m10 / M.m00);
            cy = static_cast<int>(M.m01 / M.m00);

            // Step 2: 获取该像素深度
            uint16_t depth_raw = last_depth_.at<uint16_t>(cy, cx); // mm
            depth_m = depth_raw / 1000.0f;

            cv::circle(last_rgb_, cv::Point(cx, cy), 6, cv::Scalar(0, 255, 0), -1);
            cv::putText(last_rgb_, std::to_string(depth_m) + "m", cv::Point(cx + 10, cy),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
        }

        // Step 3: 保存图像
        cv::imwrite(image_path, last_rgb_);
        RCLCPP_INFO(this->get_logger(), "✅ Image saved: %s", image_path.c_str());

        // Step 4: 保存 JSON
        json j;
        j["timestamp"] = time_str;
        j["image_file"] = image_path;
        j["object_detected"] = object_found;
        j["object_center"] = {cx, cy};
        j["depth_m"] = depth_m;

        std::ofstream out(json_path);
        out << std::setw(4) << j << std::endl;
        RCLCPP_INFO(this->get_logger(), "📝 Data saved to: %s", json_path.c_str());
    }

    std::string get_time_string()
    {
        auto t = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        return std::string(buf);
    }
};
