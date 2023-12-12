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

#include "behavior_path_goal_planner_module/util.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;

namespace behavior_path_planner
{
namespace goal_planner_utils
{

lanelet::ConstLanelets getPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance)
{
  const Pose goal_pose = route_handler.getOriginalGoalPose();

  // Buffer to get enough lanes in front of the goal, need much longer than the pull over distance.
  // In the case of loop lanes, it may not be possible to extend the lane forward.
  // todo(kosuek55): automatically calculates this distance.
  const double backward_distance_with_buffer = backward_distance + 100;

  lanelet::ConstLanelet target_shoulder_lane{};
  if (route_handler::RouteHandler::getPullOverTarget(
        route_handler.getShoulderLanelets(), goal_pose, &target_shoulder_lane)) {
    // pull over on shoulder lane
    return route_handler.getShoulderLaneletSequence(
      target_shoulder_lane, goal_pose, backward_distance_with_buffer, forward_distance);
  }

  lanelet::ConstLanelet closest_lane{};
  route_handler.getClosestLaneletWithinRoute(goal_pose, &closest_lane);
  lanelet::ConstLanelet outermost_lane{};
  if (left_side) {
    outermost_lane = route_handler.getMostLeftLanelet(closest_lane, false, true);
  } else {
    outermost_lane = route_handler.getMostRightLanelet(closest_lane, false, true);
  }

  constexpr bool only_route_lanes = false;
  return route_handler.getLaneletSequence(
    outermost_lane, backward_distance_with_buffer, forward_distance, only_route_lanes);
}

PredictedObjects filterObjectsByLateralDistance(
  const Pose & ego_pose, const double vehicle_width, const PredictedObjects & objects,
  const double distance_thresh, const bool filter_inside)
{
  PredictedObjects filtered_objects;
  for (const auto & object : objects.objects) {
    const double distance =
      utils::calcLateralDistanceFromEgoToObject(ego_pose, vehicle_width, object);
    if (filter_inside ? distance < distance_thresh : distance > distance_thresh) {
      filtered_objects.objects.push_back(object);
    }
  }

  return filtered_objects;
}

MarkerArray createPullOverAreaMarkerArray(
  const tier4_autoware_utils::MultiPolygon2d area_polygons, const std_msgs::msg::Header & header,
  const std_msgs::msg::ColorRGBA & color, const double z)
{
  MarkerArray marker_array{};
  for (size_t i = 0; i < area_polygons.size(); ++i) {
    Marker marker = createDefaultMarker(
      header.frame_id, header.stamp, "pull_over_area_" + std::to_string(i), i,
      visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0), color);
    const auto & poly = area_polygons.at(i);
    for (const auto & p : poly.outer()) {
      marker.points.push_back(createPoint(p.x(), p.y(), z));
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & pose : poses) {
    Marker marker = tier4_autoware_utils::createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::ARROW,
      createMarkerScale(0.5, 0.25, 0.25), color);
    marker.pose = pose;
    marker.id = i++;
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createGoalPriorityTextsMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & pose : poses) {
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.3, 0.3, 0.3), color);
    marker.pose = calcOffsetPose(pose, 0, 0, 1.0);
    marker.id = i;
    marker.text = std::to_string(i);
    msg.markers.push_back(marker);
    i++;
  }

  return msg;
}

MarkerArray createNumObjectsToAvoidTextsMarkerArray(
  const GoalCandidates & goal_candidates, std::string && ns, const std_msgs::msg::ColorRGBA & color)
{
  MarkerArray msg{};
  int32_t i = 0;
  for (const auto & goal_candidate : goal_candidates) {
    const Pose & pose = goal_candidate.goal_pose;
    Marker marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.3, 0.3, 0.3), color);
    marker.pose = calcOffsetPose(pose, -0.5, 0, 1.0);
    marker.id = i;
    marker.text = std::to_string(goal_candidate.num_objects_to_avoid);
    msg.markers.push_back(marker);
    i++;
  }

  return msg;
}

MarkerArray createGoalCandidatesMarkerArray(
  const GoalCandidates & goal_candidates, const std_msgs::msg::ColorRGBA & color)
{
  GoalCandidates safe_goal_candidates{};
  std::copy_if(
    goal_candidates.begin(), goal_candidates.end(), std::back_inserter(safe_goal_candidates),
    [](const auto & goal_candidate) { return goal_candidate.is_safe; });

  std::vector<Pose> pose_vector{};
  std::transform(
    safe_goal_candidates.begin(), safe_goal_candidates.end(), std::back_inserter(pose_vector),
    [](const auto & goal_candidate) { return goal_candidate.goal_pose; });

  auto marker_array = createPosesMarkerArray(pose_vector, "goal_candidates", color);
  for (const auto & text_marker :
       createGoalPriorityTextsMarkerArray(
         pose_vector, "goal_candidates_priority", createMarkerColor(1.0, 1.0, 1.0, 0.999))
         .markers) {
    marker_array.markers.push_back(text_marker);
  }
  for (const auto & text_marker : createNumObjectsToAvoidTextsMarkerArray(
                                    safe_goal_candidates, "goal_candidates_num_objects_to_avoid",
                                    createMarkerColor(0.5, 0.5, 0.5, 0.999))
                                    .markers) {
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

bool isAllowedGoalModification(const std::shared_ptr<RouteHandler> & route_handler)
{
  return route_handler->isAllowedGoalModification() || checkOriginalGoalIsInShoulder(route_handler);
}

bool checkOriginalGoalIsInShoulder(const std::shared_ptr<RouteHandler> & route_handler)
{
  const Pose & goal_pose = route_handler->getOriginalGoalPose();
  const auto shoulder_lanes = route_handler->getShoulderLanelets();

  lanelet::ConstLanelet closest_shoulder_lane{};
  if (lanelet::utils::query::getClosestLanelet(shoulder_lanes, goal_pose, &closest_shoulder_lane)) {
    return lanelet::utils::isInLanelet(goal_pose, closest_shoulder_lane, 0.1);
  }

  return false;
}

}  // namespace goal_planner_utils
}  // namespace behavior_path_planner
