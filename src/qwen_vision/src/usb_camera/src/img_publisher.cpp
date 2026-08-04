#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh); //专门用于处理图像发布和订阅的库 
  image_transport::Publisher pub = it.advertise("camera/image", 1);  //发布图像消息，话题名称为"camera/image"，队列长度为1
 
  // 实例化了一个视频捕获对象。在 OpenCV 中，VideoCapture 类专门用于处理视频输入流，
  ///主要用来从连接的硬件设备（如 USB 摄像头）或本地磁盘上的视频文件中读取连续的视频画面。
  cv::VideoCapture cap;

  // 创建一个 OpenCV 的 Mat 对象，用于存储从摄像头捕获的每一帧图像数据。
  ///Mat 是 OpenCV 中的基本图像数据结构，类似于一个多维数组，可以存储各种类型的图像数据。
  cv::Mat frame;
    
  int deviceID=0;
  
  //这段代码用于处理命令行参数，允许用户在启动该 ROS 节点时动态指定要打开的摄像头设备号
  if(argc>1)
	deviceID=argv[1][0]-'0';

  // OpenCV 初始化并打开指定的摄像头设备，同时配置与其通信的底层视频捕获框架
  int apiID=cv::CAP_ANY; //不要硬性绑定某一种特定的底层 API，而是根据当前的系统环境自动检测并使用默认或最合适的捕获后端
  cap.open(deviceID+apiID); //（deviceID + apiID）是 OpenCV 提供的一种标准语法，用于指定要打开的摄像头设备和底层视频捕获框架。deviceID 是用户指定的摄像头设备号，apiID 是 OpenCV 定义的常量，表示使用默认或自动检测的捕获后端。
  
  if(!cap.isOpened()){
	std::cerr<<"ERROR! Unable to open camera"<<std::endl;
	return -1;
  }
 
  ros::Rate loop_rate(30);
  while (nh.ok()) {
	cap.read(frame); //从底层的摄像头硬件抓取最新的一帧画面
	if(!frame.empty()){
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		pub.publish(msg);
	}
    	ros::spinOnce();
    	loop_rate.sleep();
  }
  return 0;
}

