/*
 * VectorVisualization.cpp
 *
 *  Created on: Sep 16, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_visualization/visualizations/VectorVisualization.hpp"

// Color conversion
#include <grid_map_visualization/GridMapVisualizationHelpers.hpp>

// Iterator
#include <grid_map_core/iterators/GridMapIterator.hpp>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace grid_map_visualization {

VectorVisualization::VectorVisualization(ros::NodeHandle& nodeHandle, const std::string& name)
    : VisualizationBase(nodeHandle, name)
{
}

VectorVisualization::~VectorVisualization()
{
}

bool VectorVisualization::readParameters(XmlRpc::XmlRpcValue& config)
{
  VisualizationBase::readParameters(config);

  std::string typePrefix;
  if (!getParam("layer_prefix", typePrefix)) {
    ROS_ERROR("VectorVisualization with name '%s' did not find a 'layer_prefix' parameter.", name_.c_str());
    return false;
  }
  types_.push_back(typePrefix + "x");
  types_.push_back(typePrefix + "y");
  types_.push_back(typePrefix + "z");

  if (!getParam("position_layer", positionLayer_)) {
    ROS_ERROR("VectorVisualization with name '%s' did not find a 'position_layer' parameter.", name_.c_str());
    return false;
  }

  scale_ = 1.0;
  if (!getParam("scale", scale_)) {
    ROS_INFO("VectorVisualization with name '%s' did not find a 'scale' parameter. Using default.", name_.c_str());
  }

  lineWidth_ = 0.003;
  if (!getParam("line_width", lineWidth_)) {
    ROS_INFO("VectorVisualization with name '%s' did not find a 'line_width' parameter. Using default.", name_.c_str());
  }

  int colorValue = 65280; // green
  if (!getParam("color", colorValue)) {
    ROS_INFO("VectorVisualization with name '%s' did not find a 'color' parameter. Using default.", name_.c_str());
  }
  setColorFromColorValue(color_, colorValue, true);

  return true;
}

bool VectorVisualization::initialize()
{
  marker_.ns = "vector";
  marker_.lifetime = ros::Duration();
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.type = visualization_msgs::Marker::LINE_LIST;
  marker_.scale.x = lineWidth_;
  publisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(name_, 1, true);
  return true;
}

bool VectorVisualization::visualize(const grid_map::GridMap& map)
{
  if (!isActive()) return true;

  for (const auto& type : types_) {
    if (!map.exists(type)) {
      ROS_WARN_STREAM(
          "VectorVisualization::visualize: No grid map layer with name '" << type << "' found.");
      return false;
    }
  }

  // Set marker info.
  marker_.header.frame_id = map.getFrameId();
  marker_.header.stamp.fromNSec(map.getTimestamp());

  // Clear points.
  marker_.points.clear();
  marker_.colors.clear();

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  {
    if (!map.isValid(*iterator, positionLayer_) || !map.isValid(*iterator, types_)) continue;
    geometry_msgs::Vector3 vector;
    vector.x = map.at(types_[0], *iterator);
    vector.y = map.at(types_[1], *iterator);
    vector.z = map.at(types_[2], *iterator);

    Eigen::Vector3d position;
    map.getPosition3(positionLayer_, *iterator, position);
    geometry_msgs::Point startPoint;
    startPoint.x = position.x();
    startPoint.y = position.y();
    startPoint.z = position.z();
    marker_.points.push_back(startPoint);

    geometry_msgs::Point endPoint;
    endPoint.x = startPoint.x + scale_ * vector.x;
    endPoint.y = startPoint.y + scale_ * vector.y;
    endPoint.z = startPoint.z + scale_ * vector.z;
    marker_.points.push_back(endPoint);

    marker_.colors.push_back(color_); // Each vertex needs a color.
    marker_.colors.push_back(color_);
  }

  publisher_.publish(marker_);
  return true;
}

} /* namespace */
