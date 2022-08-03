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
#include <scene_module/stop_line/scene.hpp>
#include <utilization/util.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace behavior_velocity_planner
{

using motion_utils::createStopVirtualWallMarker;
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

MarkerArray StopLineModule::createDebugMarkerArray()
{
  MarkerArray msg;

  const auto now = this->clock_->now();

  // Search stopline
  {
    auto marker = createDefaultMarker(
      "map", now, "search_stopline", module_id_, Marker::LINE_STRIP,
      createMarkerScale(0.1, 0.1, 0.1), createMarkerColor(0.0, 0.0, 1.0, 0.999));

    const auto line = debug_data_.search_stopline;
    if (!line.empty()) {
      marker.points.push_back(createPoint(line.front().x(), line.front().y(), 0.0));
      marker.points.push_back(createPoint(line.back().x(), line.back().y(), 0.0));
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray StopLineModule::createVirtualWallMarkerArray()
{
  MarkerArray wall_marker;

  if (!debug_data_.stop_pose) {
    return wall_marker;
  }

  const auto now = this->clock_->now();

  const auto p = calcOffsetPose(*debug_data_.stop_pose, debug_data_.base_link2front, 0.0, 0.0);
  appendMarkerArray(createStopVirtualWallMarker(p, "stopline", now, module_id_), &wall_marker);

  return wall_marker;
}
}  // namespace behavior_velocity_planner
