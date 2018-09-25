/*
*/

#include "astar_navi.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "astar_navi");
  AstarNavi node;
  node.run();
  return 0;
}
