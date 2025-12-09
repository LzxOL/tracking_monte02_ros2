/*
 * @Description: ROS2 node that subscribes to a color video stream and reports
 *               mouse click positions on the displayed image.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

class VideoClickNode : public rclcpp::Node {
public:
  VideoClickNode() : Node("video_click_node") {
    image_topic_ = this->declare_parameter<std::string>(
        "image_topic", "/camera_head_front/color/video");

    auto qos = rclcpp::SensorDataQoS();
    window_name_ = "video_click: " + image_topic_;

    cv::namedWindow(window_name_);
    cv::setMouseCallback(window_name_, &VideoClickNode::mouseCallback, this);

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos,
        std::bind(&VideoClickNode::onImage, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to %s. Click on the window to print XY coordinates.",
                image_topic_.c_str());
  }

  ~VideoClickNode() override { cv::destroyWindow(window_name_); }

private:
  static void mouseCallback(int event, int x, int y, int /*flags*/, void *ctx) {
    if (event != cv::EVENT_LBUTTONDOWN || ctx == nullptr) {
      return;
    }

    auto *node = static_cast<VideoClickNode *>(ctx);
    node->reportClick(x, y);
  }

  void reportClick(int x, int y) {
    RCLCPP_INFO(this->get_logger(), "Clicked pixel: x=%d, y=%d", x, y);
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(
          msg, sensor_msgs::image_encodings::BGR8);
      cv::imshow(window_name_, cv_ptr->image);
      cv::waitKey(1);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  std::string image_topic_;
  std::string window_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VideoClickNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


