#include <ros/ros.h>
#include "wayarea2grid.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wayarea2grid");
  object_map::WayareaToGrid wayarea2grid;

  wayarea2grid.run();

  return 0;
}
