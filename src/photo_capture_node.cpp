#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <filesystem>

using std::placeholders::_1;
namespace fs = std::filesystem;

class PhotoCaptureNode : public rclcpp::Node
{
public:
  PhotoCaptureNode() : Node("photo_capture_node")
  {
    RCLCPP_INFO(this->get_logger(), "Photo Capture Node started.");

    // 订阅彩色图像
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/color/image", 10, std::bind(&PhotoCaptureNode::image_callback, this, _1));

    // 订阅触发拍照
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/trigger_photo", 10, std::bind(&PhotoCaptureNode::trigger_callback, this, _1));

    // 创建保存目录
    save_dir_ = "/workspace/images/";
    fs::create_directories(save_dir_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_image_ = msg;
  }

  void trigger_callback(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!last_image_)
    {
      RCLCPP_WARN(this->get_logger(), "No image received yet.");
      return;
    }

    // 转换为 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(last_image_, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch())
                       .count();

    std::string filename = save_dir_ + "image_" + std::to_string(timestamp) + ".jpg";
    cv::imwrite(filename, cv_ptr->image);
    RCLCPP_INFO(this->get_logger(), "Saved image to %s", filename.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  sensor_msgs::msg::Image::SharedPtr last_image_;
  std::string save_dir_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PhotoCaptureNode>());
  rclcpp::shutdown();
  return 0;
}
