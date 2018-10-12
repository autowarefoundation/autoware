
#include <ros/ros.h>

#include "naive_motion_predict.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "naive_motion_predict");
  NaiveMotionPredict node;
  ros::spin();
  return 0;
}
