/*
 */

// ROS Includes
#include <ros/ros.h>

#include "obstacle_sim.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "udon_translator");

  astar_planner::ObstacleSim obstacle_sim;
  obstacle_sim.run();

  return 0;
}
