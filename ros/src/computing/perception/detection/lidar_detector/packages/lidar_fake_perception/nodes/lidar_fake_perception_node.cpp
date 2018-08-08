/*
*/

#include "lidar_fake_perception.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_fake_percetion");

  LidarFakePerception node;
  node.run();

  return 0;
}
