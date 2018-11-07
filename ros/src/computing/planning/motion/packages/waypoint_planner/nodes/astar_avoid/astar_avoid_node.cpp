/*
*/

#include "astar_avoid.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "astar_avoid");
  AstarAvoid node;
  node.run();
  return 0;
}
