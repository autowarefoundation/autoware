/*
 * grid_map_visualization_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "grid_map_visualization/GridMapVisualization.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grid_map_visualization");

  ros::NodeHandle nodeHandle("~");

  grid_map_visualization::GridMapVisualization gridMapVisualization(nodeHandle, "grid_map_visualizations");

  ros::spin();
  return 0;
}
