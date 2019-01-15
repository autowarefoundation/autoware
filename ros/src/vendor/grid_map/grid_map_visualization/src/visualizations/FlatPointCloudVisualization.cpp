/*
 * FlatPointCloudVisualization.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <grid_map_visualization/visualizations/FlatPointCloudVisualization.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <sensor_msgs/PointCloud2.h>

namespace grid_map_visualization {

FlatPointCloudVisualization::FlatPointCloudVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
    : VisualizationBase(nodeHandle, name),
      height_(0.0)
{
}

FlatPointCloudVisualization::~FlatPointCloudVisualization()
{
}

bool FlatPointCloudVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);

  height_ = 0.0;
  if (!getParam("height", height_)) {
    ROS_INFO("FlatPointCloudVisualization with name '%s' did not find a 'height' parameter. Using default.", name_.c_str());
  }

  return true;
}

bool FlatPointCloudVisualization::initialize()
{
  publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(name_, 1, true);
  return true;
}

bool FlatPointCloudVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;
  sensor_msgs::PointCloud2 pointCloud;

  grid_map::GridMap mapCopy(map);
  mapCopy.add("flat", height_);
  grid_map::GridMapRosConverter::toPointCloud(mapCopy, "flat", pointCloud);

  publisher_.publish(pointCloud);
  return true;
}

} /* namespace */
