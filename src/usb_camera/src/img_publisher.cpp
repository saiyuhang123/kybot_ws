#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("img_publisher");
  auto it = std::make_shared<image_transport::ImageTransport>(node);
  image_transport::Publisher pub = it->advertise("camera/image", 1);

  cv::VideoCapture cap;
  cv::Mat frame;

  int deviceID = 0;
  if (argc > 1)
    deviceID = argv[1][0] - '0';

  int apiID = cv::CAP_ANY;
  cap.open(deviceID + apiID);

  if (!cap.isOpened()) {
    std::cerr << "ERROR! Unable to open camera" << std::endl;
    return -1;
  }

  rclcpp::Rate loop_rate(30);
  while (rclcpp::ok()) {
    cap.read(frame);
    if (!frame.empty()) {
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
