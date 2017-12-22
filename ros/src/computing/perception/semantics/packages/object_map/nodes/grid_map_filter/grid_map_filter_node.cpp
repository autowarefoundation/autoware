// ROS Includes
#include <ros/ros.h>

#include "grid_map_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_map_filter");
  object_map::GridMapFilter grid_map_filter;

  grid_map_filter.run();

  return 0;
}
