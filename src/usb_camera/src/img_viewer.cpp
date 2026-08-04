#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  try {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("img_viewer"), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("img_viewer");

  cv::namedWindow("view");
  cv::startWindowThread();

  auto it = std::make_shared<image_transport::ImageTransport>(node);
  image_transport::Subscriber sub = it->subscribe("/camera/color/image_raw", 1, imageCallback);

  rclcpp::spin(node);
  cv::destroyWindow("view");
  rclcpp::shutdown();
  return 0;
}
