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

#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::goal_planner_utils
{

using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;

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

lanelet::ConstLanelets generateExpandedPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance, double bound_offset)
{
  const auto pull_over_lanes =
    getPullOverLanes(route_handler, left_side, backward_distance, forward_distance);

  return left_side ? lanelet::utils::getExpandedLanelets(pull_over_lanes, bound_offset, 0.0)
                   : lanelet::utils::getExpandedLanelets(pull_over_lanes, 0.0, bound_offset);
}

PredictedObjects extractObjectsInExpandedPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance, double bound_offset, const PredictedObjects & objects)
{
  const auto lanes = generateExpandedPullOverLanes(
    route_handler, left_side, backward_distance, forward_distance, bound_offset);

  const auto [objects_in_lanes, others] = utils::path_safety_checker::separateObjectsByLanelets(
    objects, lanes, utils::path_safety_checker::isPolygonOverlapLanelet);

  return objects_in_lanes;
}

PredictedObjects extractStaticObjectsInExpandedPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance, double bound_offset, const PredictedObjects & objects,
  const double velocity_thresh)
{
  const auto objects_in_lanes = extractObjectsInExpandedPullOverLanes(
    route_handler, left_side, backward_distance, forward_distance, bound_offset, objects);

  return utils::path_safety_checker::filterObjectsByVelocity(objects_in_lanes, velocity_thresh);
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

double calcLateralDeviationBetweenPaths(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path)
{
  double lateral_deviation = 0.0;
  for (const auto & target_point : target_path.points) {
    const size_t nearest_index =
      motion_utils::findNearestIndex(reference_path.points, target_point.point.pose.position);
    lateral_deviation = std::max(
      lateral_deviation,
      std::abs(tier4_autoware_utils::calcLateralDeviation(
        reference_path.points[nearest_index].point.pose, target_point.point.pose.position)));
  }
  return lateral_deviation;
}

bool isReferencePath(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path,
  const double lateral_deviation_threshold)
{
  return calcLateralDeviationBetweenPaths(reference_path, target_path) <
         lateral_deviation_threshold;
}

std::optional<PathWithLaneId> cropPath(const PathWithLaneId & path, const Pose & end_pose)
{
  const size_t end_idx = motion_utils::findNearestSegmentIndex(path.points, end_pose.position);
  std::vector<PathPointWithLaneId> clipped_points{
    path.points.begin(), path.points.begin() + end_idx};
  if (clipped_points.empty()) {
    return std::nullopt;
  }

  // add projected end pose to clipped points
  PathPointWithLaneId projected_point = clipped_points.back();
  const double offset = motion_utils::calcSignedArcLength(path.points, end_idx, end_pose.position);
  projected_point.point.pose =
    tier4_autoware_utils::calcOffsetPose(clipped_points.back().point.pose, offset, 0, 0);
  clipped_points.push_back(projected_point);
  auto clipped_path = path;
  clipped_path.points = clipped_points;

  return clipped_path;
}

PathWithLaneId cropForwardPoints(
  const PathWithLaneId & path, const size_t target_seg_idx, const double forward_length)
{
  const auto & points = path.points;

  double sum_length = 0;
  for (size_t i = target_seg_idx + 1; i < points.size(); ++i) {
    const double seg_length = tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i - 1));
    if (forward_length < sum_length + seg_length) {
      const auto cropped_points =
        std::vector<PathPointWithLaneId>{points.begin() + target_seg_idx, points.begin() + i};
      PathWithLaneId cropped_path = path;
      cropped_path.points = cropped_points;

      // add precise end pose to cropped points
      const double remaining_length = forward_length - sum_length;
      const Pose precise_end_pose =
        calcOffsetPose(cropped_path.points.back().point.pose, remaining_length, 0, 0);
      if (remaining_length < 0.1) {
        // if precise_end_pose is too close, replace the last point
        cropped_path.points.back().point.pose = precise_end_pose;
      } else {
        auto precise_end_point = cropped_path.points.back();
        precise_end_point.point.pose = precise_end_pose;
        cropped_path.points.push_back(precise_end_point);
      }
      return cropped_path;
    }
    sum_length += seg_length;
  }

  // if forward_length is too long, return points after target_seg_idx
  const auto cropped_points =
    std::vector<PathPointWithLaneId>{points.begin() + target_seg_idx, points.end()};
  PathWithLaneId cropped_path = path;
  cropped_path.points = cropped_points;
  return cropped_path;
}

PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const double extend_length)
{
  const auto & target_terminal_pose = target_path.points.back().point.pose;

  // generate clipped road lane reference path from previous module path terminal pose to shift end
  const size_t target_path_terminal_idx =
    motion_utils::findNearestSegmentIndex(reference_path.points, target_terminal_pose.position);

  PathWithLaneId clipped_path =
    cropForwardPoints(reference_path, target_path_terminal_idx, extend_length);

  // shift clipped path to previous module path terminal pose
  const double lateral_shift_from_reference_path =
    motion_utils::calcLateralOffset(reference_path.points, target_terminal_pose.position);
  for (auto & p : clipped_path.points) {
    p.point.pose =
      tier4_autoware_utils::calcOffsetPose(p.point.pose, 0, lateral_shift_from_reference_path, 0);
  }

  auto extended_path = target_path;
  const auto start_point =
    std::find_if(clipped_path.points.begin(), clipped_path.points.end(), [&](const auto & p) {
      const bool is_forward =
        tier4_autoware_utils::inverseTransformPoint(p.point.pose.position, target_terminal_pose).x >
        0.0;
      const bool is_close = tier4_autoware_utils::calcDistance2d(
                              p.point.pose.position, target_terminal_pose.position) < 0.1;
      return is_forward && !is_close;
    });
  std::copy(start_point, clipped_path.points.end(), std::back_inserter(extended_path.points));

  extended_path.points = motion_utils::removeOverlapPoints(extended_path.points);

  return extended_path;
}

PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const Pose & extend_pose)
{
  const auto & target_terminal_pose = target_path.points.back().point.pose;
  const size_t target_path_terminal_idx =
    motion_utils::findNearestSegmentIndex(reference_path.points, target_terminal_pose.position);
  const double extend_distance = motion_utils::calcSignedArcLength(
    reference_path.points, target_path_terminal_idx, extend_pose.position);

  return extendPath(target_path, reference_path, extend_distance);
}

}  // namespace behavior_path_planner::goal_planner_utils
