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
#include <scene_module/crosswalk/scene_crosswalk.hpp>
#include <scene_module/crosswalk/scene_walkway.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <utilization/util.hpp>

#include <vector>

namespace behavior_velocity_planner
{

using motion_utils::createSlowDownVirtualWallMarker;
using motion_utils::createStopVirtualWallMarker;
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;

namespace
{
visualization_msgs::msg::MarkerArray createCrosswalkMarkers(
  const DebugData & debug_data, const rclcpp::Time & now, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  int32_t uid = planning_utils::bitShift(module_id);

  {
    size_t i = 0;
    for (const auto & p : debug_data.collision_points) {
      auto marker = createDefaultMarker(
        "map", now, "collision point state", uid + i++, Marker::TEXT_VIEW_FACING,
        createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
      std::ostringstream string_stream;
      string_stream << std::fixed << std::setprecision(2);
      string_stream << "(module, ttc, ttv, state)=(" << module_id << " , " << p.time_to_collision
                    << " , " << p.time_to_vehicle;
      switch (p.state) {
        case CollisionPointState::YIELD:
          string_stream << " , YIELD)";
          break;
        case CollisionPointState::EGO_PASS_FIRST:
          string_stream << " , EGO_PASS_FIRST)";
          break;
        case CollisionPointState::EGO_PASS_LATER:
          string_stream << " , EGO_PASS_LATER)";
          break;
        case CollisionPointState::IGNORE:
          string_stream << " , IGNORE)";
          break;
        default:
          string_stream << " , NONE)";
          break;
      }
      marker.text = string_stream.str();
      marker.pose.position = p.collision_point;
      marker.pose.position.z += 2.0;
      msg.markers.push_back(marker);
    }
  }

  if (debug_data.range_near_point) {
    auto marker = createDefaultMarker(
      "map", now, "attention range near", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(0.0, 0.0, 1.0, 0.999));
    marker.points.push_back(debug_data.range_near_point.get());
    msg.markers.push_back(marker);
  }

  if (debug_data.range_far_point) {
    auto marker = createDefaultMarker(
      "map", now, "attention range far", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.points.push_back(debug_data.range_far_point.get());
    msg.markers.push_back(marker);
  }

  {
    size_t i = 0;
    for (const auto & polygon : debug_data.obj_polygons) {
      auto marker = createDefaultMarker(
        "map", now, "object polygon", uid + i++, Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 0.0, 1.0, 0.999));
      for (const auto & p : polygon.points) {
        marker.points.push_back(createPoint(p.x, p.y, p.z));
      }
      marker.points.push_back(marker.points.front());
      msg.markers.push_back(marker);
    }
  }

  {
    size_t i = 0;
    for (const auto & polygon : debug_data.ego_polygons) {
      auto marker = createDefaultMarker(
        "map", now, "vehicle polygon", uid + i++, Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));
      for (const auto & p : polygon.points) {
        marker.points.push_back(createPoint(p.x, p.y, p.z));
      }
      marker.points.push_back(marker.points.front());
      msg.markers.push_back(marker);
    }
  }

  // Crosswalk polygon
  if (!debug_data.crosswalk_polygon.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "crosswalk polygon", uid, Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & p : debug_data.crosswalk_polygon) {
      marker.points.push_back(createPoint(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    marker.color = debug_data.ignore_crosswalk ? createMarkerColor(1.0, 1.0, 1.0, 0.999)
                                               : createMarkerColor(1.0, 0.0, 0.0, 0.999);
    msg.markers.push_back(marker);
  }

  // Collision point
  if (!debug_data.collision_points.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "collision point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_data.collision_points) {
      marker.points.push_back(p.collision_point);
    }
    msg.markers.push_back(marker);
  }

  // Slow point
  if (!debug_data.slow_poses.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "slow point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 1.0, 0.0, 0.999));
    for (const auto & p : debug_data.slow_poses) {
      marker.points.push_back(createPoint(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  // Stop factor point
  if (!debug_data.stop_factor_points.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "stop factor point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(0.0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_data.stop_factor_points) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  // Stop point
  if (!debug_data.stop_poses.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "stop point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 0.0, 0.0, 0.999));
    for (const auto & p : debug_data.stop_poses) {
      marker.points.push_back(createPoint(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray createWalkwayMarkers(
  const DebugData & debug_data, const rclcpp::Time & now, const int64_t module_id)
{
  int32_t uid = planning_utils::bitShift(module_id);
  visualization_msgs::msg::MarkerArray msg;

  // Stop point
  if (!debug_data.stop_poses.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "stop point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 0.0, 0.0, 0.999));
    for (const auto & p : debug_data.stop_poses) {
      marker.points.push_back(createPoint(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  {
    size_t i = 0;
    for (const auto & p : debug_data.stop_poses) {
      auto marker = createDefaultMarker(
        "map", now, "walkway stop judge range", uid + i++, Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.1, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.5));
      for (size_t j = 0; j < 50; ++j) {
        const auto x = p.position.x + debug_data.stop_judge_range * std::cos(M_PI * 2 / 50 * j);
        const auto y = p.position.y + debug_data.stop_judge_range * std::sin(M_PI * 2 / 50 * j);
        marker.points.push_back(createPoint(x, y, p.position.z));
      }
      marker.points.push_back(marker.points.front());
      msg.markers.push_back(marker);
    }
  }

  return msg;
}
}  // namespace

visualization_msgs::msg::MarkerArray CrosswalkModule::createVirtualWallMarkerArray()
//  const std::string & ns, const size_t id, const std::vector<geometry_msgs::msg::Pose> &
//  stop_poses, const std::vector<geometry_msgs::msg::Pose> & slow_poses)
{
  const auto now = this->clock_->now();
  auto id = module_id_;

  visualization_msgs::msg::MarkerArray wall_marker;
  for (const auto & p : debug_data_.stop_poses) {
    const auto p_front = calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(createStopVirtualWallMarker(p_front, "crosswalk", now, id++), &wall_marker);
  }
  for (const auto & p : debug_data_.slow_poses) {
    const auto p_front = calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(
      createSlowDownVirtualWallMarker(p_front, "crosswalk", now, id++), &wall_marker);
  }

  return wall_marker;
}

visualization_msgs::msg::MarkerArray WalkwayModule::createVirtualWallMarkerArray()
{
  const auto now = this->clock_->now();
  auto id = module_id_;

  visualization_msgs::msg::MarkerArray wall_marker;
  for (const auto & p : debug_data_.stop_poses) {
    const auto p_front = calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(createStopVirtualWallMarker(p_front, "walkway", now, id++), &wall_marker);
  }
  return wall_marker;
}

visualization_msgs::msg::MarkerArray CrosswalkModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createCrosswalkMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}

visualization_msgs::msg::MarkerArray WalkwayModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createWalkwayMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
