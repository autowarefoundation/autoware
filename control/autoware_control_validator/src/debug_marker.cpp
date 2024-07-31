// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/control_validator/debug_marker.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <string>

using visualization_msgs::msg::Marker;

ControlValidatorDebugMarkerPublisher::ControlValidatorDebugMarkerPublisher(rclcpp::Node * node)
: node_(node)
{
  debug_viz_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);

  virtual_wall_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/virtual_wall", 1);
}

void ControlValidatorDebugMarkerPublisher::clear_markers()
{
  marker_array_.markers.clear();
  marker_array_virtual_wall_.markers.clear();
}

void ControlValidatorDebugMarkerPublisher::push_warning_msg(
  const geometry_msgs::msg::Pose & pose, const std::string & msg)
{
  visualization_msgs::msg::Marker marker = autoware::universe_utils::createDefaultMarker(
    "map", node_->get_clock()->now(), "warning_msg", 0, Marker::TEXT_VIEW_FACING,
    autoware::universe_utils::createMarkerScale(0.0, 0.0, 1.0),
    autoware::universe_utils::createMarkerColor(1.0, 0.1, 0.1, 0.999));
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose = pose;
  marker.text = msg;
  marker_array_virtual_wall_.markers.push_back(marker);
}

void ControlValidatorDebugMarkerPublisher::push_virtual_wall(const geometry_msgs::msg::Pose & pose)
{
  const auto now = node_->get_clock()->now();
  const auto stop_wall_marker =
    autoware::motion_utils::createStopVirtualWallMarker(pose, "control_validator", now, 0);
  autoware::universe_utils::appendMarkerArray(stop_wall_marker, &marker_array_virtual_wall_, now);
}

void ControlValidatorDebugMarkerPublisher::publish()
{
  debug_viz_pub_->publish(marker_array_);
  virtual_wall_pub_->publish(marker_array_virtual_wall_);
}
