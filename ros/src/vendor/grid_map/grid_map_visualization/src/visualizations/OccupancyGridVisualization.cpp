/*
 * OccupancyGridVisualization.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/OccupancyGridVisualization.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace grid_map_visualization {

OccupancyGridVisualization::OccupancyGridVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
: VisualizationBase(nodeHandle, name),
  dataMin_(0.0),
  dataMax_(1.0)
{
}

OccupancyGridVisualization::~OccupancyGridVisualization()
{
}

bool OccupancyGridVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);

  if (!getParam("layer", layer_)) {
    ROS_ERROR("OccupancyGridVisualization with name '%s' did not find a 'layer' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("data_min", dataMin_)) {
    ROS_ERROR("OccupancyGridVisualization with name '%s' did not find a 'data_min' parameter.", name_.c_str());
    return false;
  }

  if (!getParam("data_max", dataMax_)) {
    ROS_ERROR("OccupancyGridVisualization with name '%s' did not find a 'data_max' parameter.", name_.c_str());
    return false;
  }

  return true;
}

bool OccupancyGridVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<nav_msgs::OccupancyGrid>(name_, 1, true);
  return true;
}

bool OccupancyGridVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;
  if (!map.exists(layer_)) {
    ROS_WARN_STREAM("OccupancyGridVisualization::visualize: No grid map layer with name '" << layer_ << "' found.");
    return false;
  }
  nav_msgs::OccupancyGrid occupancyGrid;
  grid_map::GridMapRosConverter::toOccupancyGrid(map, layer_, dataMin_, dataMax_, occupancyGrid);
  publisher_.publish(occupancyGrid);
  return true;
}

} /* namespace */
