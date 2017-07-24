#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ros/ros.h"

using namespace std;

class SavePCD
{
public:
  string save_path_;
  string topic_name_;
  void save_pcd(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void sub_pcd(int argc, char* argv[]);
};

static void check_arguments(int argc, char* argv[]);
// static void save_image(const sensor_msgs::Image& msg);
