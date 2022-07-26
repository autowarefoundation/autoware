// Copyright 2020 Tier IV, Inc.
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

#include <motion_utils/motion_utils.hpp>
#include <scene_module/traffic_light/scene.hpp>
#include <utilization/util.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace behavior_velocity_planner
{
using tier4_autoware_utils::appendMarkerArray;

visualization_msgs::msg::MarkerArray TrafficLightModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  return debug_marker_array;
}

visualization_msgs::msg::MarkerArray TrafficLightModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  auto id = module_id_;
  const auto now = this->clock_->now();

  for (const auto & p : debug_data_.dead_line_poses) {
    const auto p_front =
      tier4_autoware_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(
      motion_utils::createDeadLineVirtualWallMarker(p_front, "traffic_light", now, id++),
      &wall_marker, now);
  }

  for (const auto & p : debug_data_.stop_poses) {
    const auto p_front =
      tier4_autoware_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(p_front, "traffic_light", now, id++), &wall_marker,
      now);
  }

  return wall_marker;
}

}  // namespace behavior_velocity_planner
