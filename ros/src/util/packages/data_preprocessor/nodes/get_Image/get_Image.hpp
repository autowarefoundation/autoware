#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class SaveImage
{
public:
  string save_path_;
  string topic_name_;
  void save_image(const sensor_msgs::Image& msg);
  void sub_image(int argc, char* argv[]);
};

static void check_arguments(int argc, char* argv[]);
// static void save_image(const sensor_msgs::Image& msg);
