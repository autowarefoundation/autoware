/*
 * GridCellsVisualization.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/GridCellsVisualization.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/GridCells.h>

// STD
#include <limits>

namespace grid_map_visualization {

GridCellsVisualization::GridCellsVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
: VisualizationBase(nodeHandle, name),
  lowerThreshold_(-std::numeric_limits<float>::infinity()),
  upperThreshold_(std::numeric_limits<float>::infinity())
{
}

GridCellsVisualization::~GridCellsVisualization()
{
}

bool GridCellsVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);

  if (!getParam("layer", layer_)) {
    ROS_ERROR("GridCellsVisualization with name '%s' did not find a 'layer' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("lower_threshold", lowerThreshold_)) {
    ROS_INFO("GridCellsVisualization with name '%s' did not find a 'lower_threshold' parameter. Using negative infinity.", name_.c_str());
  }

  if (!getParam("upper_threshold", upperThreshold_)) {
    ROS_INFO("GridCellsVisualization with name '%s' did not find a 'upper_threshold' parameter. Using infinity.", name_.c_str());
  }

  return true;
}

bool GridCellsVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<nav_msgs::GridCells>(name_, 1, true);
  return true;
}

bool GridCellsVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;
  if (!map.exists(layer_)) {
    ROS_WARN_STREAM("GridCellsVisualization::visualize: No grid map layer with name '" << layer_ << "' found.");
    return false;
  }
  nav_msgs::GridCells gridCells;
  grid_map::GridMapRosConverter::toGridCells(map, layer_, lowerThreshold_, upperThreshold_, gridCells);
  publisher_.publish(gridCells);
  return true;
}

} /* namespace */
