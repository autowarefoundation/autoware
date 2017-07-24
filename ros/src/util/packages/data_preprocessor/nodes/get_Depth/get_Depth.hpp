#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"

using namespace std;

class SaveDepth
{
public:
  string save_path_;
  string image_topic_name_;
  string pc2_topic_name_;
  string calib_path_;
  bool is_image_;
  bool is_pc2_;
  sensor_msgs::PointCloud2 points_;
  // pcl::PointCloud<pcl::PointXYZ> points_;
  cv::Mat image_;
  string time_stamp_;
  cv::Mat CameraExtrinsicMat_;
  cv::Mat CameraMat_;
  cv::Mat DistCoeff_;
  cv::Size ImageSize_;
  void get_pcd(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void get_image(const sensor_msgs::Image& msg);
  void depthConverter(int argc, char* argv[]);
  void create_depth();
  void read_CalibFile();
  void reset_flags();
};


static void check_arguments(int argc, char* argv[]);
