#include "lidar_naive_l_shape_detect.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "lidar_naive_l_shape_detect");

  LShapeFilter app;
  ros::spin();

  return 0;
}
