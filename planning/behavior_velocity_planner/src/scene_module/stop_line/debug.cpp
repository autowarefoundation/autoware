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
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

namespace
{
using DebugData = StopLineModule::DebugData;

visualization_msgs::msg::MarkerArray createStopLineCollisionCheck(
  const DebugData & debug_data, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;

  // Search Segments
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "search_segments";
    marker.id = module_id;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    for (const auto & e : debug_data.search_segments) {
      marker.points.push_back(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(e.at(0).x()).y(e.at(0).y()).z(0.0));
      marker.points.push_back(
        geometry_msgs::build<geometry_msgs::msg::Point>().x(e.at(1).x()).y(e.at(1).y()).z(0.0));
    }
    marker.scale = createMarkerScale(0.1, 0.1, 0.1);
    marker.color = createMarkerColor(0.0, 0.0, 1.0, 0.999);
    msg.markers.push_back(marker);
  }

  // Search stopline
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "search_stopline";
    marker.id = module_id;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const auto p0 = debug_data.search_stopline.at(0);
    marker.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(p0.x()).y(p0.y()).z(0.0));
    const auto p1 = debug_data.search_stopline.at(1);
    marker.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point>().x(p1.x()).y(p1.y()).z(0.0));

    marker.scale = createMarkerScale(0.1, 0.1, 0.1);
    marker.color = createMarkerColor(1.0, 0.0, 0.0, 0.999);
    msg.markers.push_back(marker);
  }

  return msg;
}

}  // namespace

visualization_msgs::msg::MarkerArray StopLineModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  if (planner_param_.show_stopline_collision_check) {
    appendMarkerArray(
      createStopLineCollisionCheck(debug_data_, module_id_), &debug_marker_array,
      this->clock_->now());
  }
  return debug_marker_array;
}

visualization_msgs::msg::MarkerArray StopLineModule::createVirtualWallMarkerArray()
{
  const auto now = this->clock_->now();
  visualization_msgs::msg::MarkerArray wall_marker;
  if (!debug_data_.stop_pose) {
    return wall_marker;
  }
  const auto p_front = tier4_autoware_utils::calcOffsetPose(
    *debug_data_.stop_pose, debug_data_.base_link2front, 0.0, 0.0);
  if (state_ == State::APPROACH || state_ == State::STOPPED) {
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(p_front, "stopline", now, module_id_), &wall_marker,
      now);
  }
  return wall_marker;
}
}  // namespace behavior_velocity_planner
