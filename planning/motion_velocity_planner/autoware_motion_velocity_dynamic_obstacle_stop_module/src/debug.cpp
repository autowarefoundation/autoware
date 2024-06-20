// Copyright 2023-2024 TIER IV, Inc.
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

#include "debug.hpp"

#include "types.hpp"

#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop::debug
{

std::vector<visualization_msgs::msg::Marker> make_delete_markers(
  const size_t from, const size_t to, const std::string & ns)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker.ns = ns;
  for (marker.id = static_cast<int>(from); marker.id < static_cast<int>(to); ++marker.id)
    markers.push_back(marker);
  return markers;
}

void add_markers(
  visualization_msgs::msg::MarkerArray & array, size_t & prev_nb,
  const std::vector<visualization_msgs::msg::Marker> & markers, const std::string & ns)
{
  array.markers.insert(array.markers.end(), markers.begin(), markers.end());
  const auto delete_markers = debug::make_delete_markers(markers.size(), prev_nb, ns);
  array.markers.insert(array.markers.end(), delete_markers.begin(), delete_markers.end());
  prev_nb = markers.size();
}

std::vector<visualization_msgs::msg::Marker> make_collision_markers(
  const ObjectStopDecisionMap & object_map, const std::string & ns, const double z,
  const rclcpp::Time & now)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = ns;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware::universe_utils::createMarkerScale(0.2, 0.2, 0.5);
  marker.color = autoware::universe_utils::createMarkerColor(0.6, 0.0, 0.6, 1.0);
  for (const auto & [object_uuid, decision] : object_map) {
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.text = object_uuid.substr(0, 5) + "\n";
    if (decision.should_be_avoided()) {
      marker.text += "avoiding\nlast detection = ";
      marker.text += std::to_string((now - *decision.last_stop_decision_time).seconds());
      marker.text += "s\n";
    } else {
      marker.text += "first detection = ";
      marker.text += std::to_string((now - *decision.start_detection_time).seconds());
      marker.text += "s\n";
    }
    marker.pose.position = decision.collision_point;
    marker.pose.position.z = z;
    markers.push_back(marker);
    ++marker.id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    markers.push_back(marker);
    ++marker.id;
  }
  return markers;
}

std::vector<visualization_msgs::msg::Marker> make_polygon_markers(
  const autoware::universe_utils::MultiPolygon2d & footprints, const std::string & ns,
  const double z)
{
  std::vector<visualization_msgs::msg::Marker> markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time(0);
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware::universe_utils::createMarkerScale(0.1, 0.1, 0.1);
  marker.color = autoware::universe_utils::createMarkerColor(0.1, 1.0, 0.1, 0.8);
  for (const auto & footprint : footprints) {
    marker.points.clear();
    for (const auto & p : footprint.outer()) {
      marker.points.emplace_back();
      marker.points.back().x = p.x();
      marker.points.back().y = p.y();
      marker.points.back().z = z;
    }
    markers.push_back(marker);
    ++marker.id;
  }
  return markers;
}
}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop::debug
