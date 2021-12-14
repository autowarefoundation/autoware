// Copyright 2021 Tier IV, Inc.
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

#include "planning_error_monitor/debug_marker.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>
#include <vector>

using visualization_msgs::msg::Marker;

PlanningErrorMonitorDebugNode::PlanningErrorMonitorDebugNode() {}

void PlanningErrorMonitorDebugNode::initialize(rclcpp::Node * node)
{
  node_ = node;
  debug_viz_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);
  initialized = true;
}

void PlanningErrorMonitorDebugNode::pushPoseMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, int id)
{
  if (!initialized) {
    return;
  }
  // append arrow marker
  Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = ns;
  marker.id = getMarkerId(ns);
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.type = Marker::ARROW;
  marker.action = Marker::ADD;
  marker.pose = pose;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.2, 0.1, 0.3);
  if (id == 0)  // Red
  {
    marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999);
  }
  if (id == 1)  // Green
  {
    marker.color = tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
  }
  if (id == 2)  // Blue
  {
    marker.color = tier4_autoware_utils::createMarkerColor(0.0, 0.0, 1.0, 0.999);
  }
  marker_array_.markers.push_back(marker);
}

void PlanningErrorMonitorDebugNode::clearPoseMarker(const std::string & ns)
{
  clearMarkerId(ns);

  if (marker_array_.markers.empty()) {
    return;
  }

  for (int i = static_cast<int>(marker_array_.markers.size()) - 1; i >= 0; i--) {
    if (marker_array_.markers.at(i).ns == ns) {
      // clear markers with the namespace "ns"
      marker_array_.markers.erase(marker_array_.markers.begin() + i);
    }
  }
}

void PlanningErrorMonitorDebugNode::publish()
{
  if (initialized) {
    debug_viz_pub_->publish(marker_array_);
  }
}
