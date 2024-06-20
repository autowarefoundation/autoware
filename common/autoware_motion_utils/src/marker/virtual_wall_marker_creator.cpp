// Copyright 2023 Tier IV, Inc.
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

#include "autoware/motion_utils/marker/virtual_wall_marker_creator.hpp"

#include "autoware/motion_utils/marker/marker_helper.hpp"

namespace autoware::motion_utils
{

void VirtualWallMarkerCreator::cleanup()
{
  for (auto it = marker_count_per_namespace_.begin(); it != marker_count_per_namespace_.end();) {
    const auto & marker_count = it->second;
    const auto is_unused_namespace = marker_count.previous == 0 && marker_count.current == 0;
    if (is_unused_namespace)
      it = marker_count_per_namespace_.erase(it);
    else
      ++it;
  }
  virtual_walls_.clear();
}

void VirtualWallMarkerCreator::add_virtual_wall(const VirtualWall & virtual_wall)
{
  virtual_walls_.push_back(virtual_wall);
}
void VirtualWallMarkerCreator::add_virtual_walls(const VirtualWalls & walls)
{
  virtual_walls_.insert(virtual_walls_.end(), walls.begin(), walls.end());
}

visualization_msgs::msg::MarkerArray VirtualWallMarkerCreator::create_markers(
  const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray marker_array;
  // update marker counts
  for (auto & [ns, count] : marker_count_per_namespace_) {
    count.previous = count.current;
    count.current = 0UL;
  }
  // convert to markers
  create_wall_function create_fn;
  for (const auto & virtual_wall : virtual_walls_) {
    switch (virtual_wall.style) {
      case stop:
        create_fn = autoware::motion_utils::createStopVirtualWallMarker;
        break;
      case slowdown:
        create_fn = autoware::motion_utils::createSlowDownVirtualWallMarker;
        break;
      case deadline:
        create_fn = autoware::motion_utils::createDeadLineVirtualWallMarker;
        break;
    }
    auto markers = create_fn(
      virtual_wall.pose, virtual_wall.text, now, 0, virtual_wall.longitudinal_offset,
      virtual_wall.ns, virtual_wall.is_driving_forward);
    for (auto & marker : markers.markers) {
      marker.id = static_cast<int>(marker_count_per_namespace_[marker.ns].current++);
      marker_array.markers.push_back(marker);
    }
  }
  // create delete markers
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  for (const auto & [ns, count] : marker_count_per_namespace_) {
    for (marker.id = static_cast<int>(count.current); marker.id < static_cast<int>(count.previous);
         ++marker.id) {
      marker.ns = ns;
      marker_array.markers.push_back(marker);
    }
  }
  cleanup();
  return marker_array;
}
}  // namespace autoware::motion_utils
