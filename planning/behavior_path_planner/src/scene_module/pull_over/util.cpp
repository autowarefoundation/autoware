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

#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::inverseTransformPoint;

namespace behavior_path_planner
{
namespace pull_over_utils
{
PathWithLaneId combineReferencePath(const PathWithLaneId path1, const PathWithLaneId path2)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  return path;
}

std::vector<ShiftParkingPath> generateShiftParkingPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & current_pose, const Pose & goal_pose,
  const BehaviorPathPlannerParameters & common_parameter, const PullOverParameters & parameter)
{
  std::vector<ShiftParkingPath> candidate_paths;

  if (original_lanelets.empty() || target_lanelets.empty()) {
    return candidate_paths;
  }

  // rename parameter
  const double backward_path_length = common_parameter.backward_path_length;
  const double pull_over_velocity = parameter.pull_over_velocity;
  const double after_pull_over_straight_distance = parameter.after_pull_over_straight_distance;
  const double margin = parameter.margin_from_boundary;
  const double minimum_lateral_jerk = parameter.minimum_lateral_jerk;
  const double maximum_lateral_jerk = parameter.maximum_lateral_jerk;
  const double deceleration_interval = parameter.deceleration_interval;
  const int pull_over_sampling_num = parameter.pull_over_sampling_num;
  const double jerk_resolution =
    std::abs(maximum_lateral_jerk - minimum_lateral_jerk) / pull_over_sampling_num;

  const double distance_to_shoulder_lane_boundary =
    util::getDistanceToShoulderBoundary(route_handler.getShoulderLanelets(), current_pose);
  const double offset_from_current_pose =
    distance_to_shoulder_lane_boundary + common_parameter.vehicle_width / 2 + margin;

  // shift end point in shoulder lane
  PathPointWithLaneId shift_end_point;
  {
    const auto arc_position_goal = lanelet::utils::getArcCoordinates(target_lanelets, goal_pose);
    const double s_start = arc_position_goal.length - after_pull_over_straight_distance;
    const double s_end = s_start + std::numeric_limits<double>::epsilon();
    const auto path = route_handler.getCenterLinePath(target_lanelets, s_start, s_end, true);
    shift_end_point = path.points.front();
  }

  for (double lateral_jerk = minimum_lateral_jerk; lateral_jerk <= maximum_lateral_jerk;
       lateral_jerk += jerk_resolution) {
    PathShifter path_shifter;
    ShiftedPath shifted_path;
    ShiftParkingPath candidate_path;

    double pull_over_distance = path_shifter.calcLongitudinalDistFromJerk(
      abs(offset_from_current_pose), lateral_jerk, pull_over_velocity);

    // calculate straight distance before pull over
    double straight_distance;
    {
      const auto arc_position_goal =
        lanelet::utils::getArcCoordinates(original_lanelets, goal_pose);
      const auto arc_position_pose =
        lanelet::utils::getArcCoordinates(original_lanelets, current_pose);
      straight_distance = arc_position_goal.length - after_pull_over_straight_distance -
                          pull_over_distance - arc_position_pose.length;
    }

    PathWithLaneId road_lane_reference_path;
    {
      const auto arc_position = lanelet::utils::getArcCoordinates(original_lanelets, current_pose);
      const auto arc_position_ref2_front =
        lanelet::utils::getArcCoordinates(original_lanelets, shift_end_point.point.pose);
      const double s_start = arc_position.length - backward_path_length;
      const double s_end = arc_position_ref2_front.length - pull_over_distance;
      road_lane_reference_path = route_handler.getCenterLinePath(original_lanelets, s_start, s_end);
      // decelerate velocity linearly to minimum pull over velocity
      // ( or accelerate if original velocity is lower than minimum velocity )
      for (auto & point : road_lane_reference_path.points) {
        const auto arclength =
          lanelet::utils::getArcCoordinates(original_lanelets, point.point.pose).length;
        const double distance_to_pull_over_start = std::max(0.0, s_end - arclength);
        point.point.longitudinal_velocity_mps = std::min(
          point.point.longitudinal_velocity_mps,
          static_cast<float>(
            (distance_to_pull_over_start / deceleration_interval) *
              (point.point.longitudinal_velocity_mps - pull_over_velocity) +
            pull_over_velocity));
      }
    }

    if (road_lane_reference_path.points.empty()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("pull_over").get_child("util"),
        "reference path is empty!! something wrong...");
      continue;
    }

    PathWithLaneId target_lane_reference_path;
    {
      const lanelet::ArcCoordinates pull_over_start_arc_position =
        lanelet::utils::getArcCoordinates(
          target_lanelets, road_lane_reference_path.points.back().point.pose);
      const double s_start = pull_over_start_arc_position.length;
      const auto arc_position_goal = lanelet::utils::getArcCoordinates(target_lanelets, goal_pose);
      const double s_end = arc_position_goal.length;
      target_lane_reference_path = route_handler.getCenterLinePath(target_lanelets, s_start, s_end);
      // distance between shoulder lane's left boundary and shoulder lane center
      double distance_shoulder_to_left_bound = util::getDistanceToShoulderBoundary(
        route_handler.getShoulderLanelets(), shift_end_point.point.pose);

      // distance between shoulder lane center and target line
      double distance_shoulder_to_target =
        distance_shoulder_to_left_bound + common_parameter.vehicle_width / 2 + margin;

      // Apply shifting shoulder lane to adjust to target line
      double offset = -distance_shoulder_to_target;
      for (size_t i = 0; i < target_lane_reference_path.points.size(); ++i) {
        {
          if (fabs(offset) < 1.0e-8) {
            RCLCPP_WARN_STREAM(
              rclcpp::get_logger("behavior_path_planner").get_child("pull_over").get_child("util"),
              "no offset from current lane center.");
          }

          auto & p = target_lane_reference_path.points.at(i).point.pose;
          double yaw = tf2::getYaw(p.orientation);
          p.position.x -= std::sin(yaw) * offset;
          p.position.y += std::cos(yaw) * offset;
        }
        path_shifter.setPath(util::resamplePathWithSpline(target_lane_reference_path, 1.0));
      }
    }
    ShiftPoint shift_point;
    {
      shift_point.start = road_lane_reference_path.points.back().point.pose;
      shift_point.end = shift_end_point.point.pose;

      // distance between shoulder lane's left boundary and current lane center
      double distance_road_to_left_boundary = util::getDistanceToShoulderBoundary(
        route_handler.getShoulderLanelets(), road_lane_reference_path.points.back().point.pose);
      // distance between shoulder lane's left boundary and current lane center
      double distance_road_to_target =
        distance_road_to_left_boundary + common_parameter.vehicle_width / 2 + margin;

      shift_point.length = distance_road_to_target;
      path_shifter.addShiftPoint(shift_point);
    }

    // offset front side from reference path
    bool offset_back = false;
    if (!path_shifter.generate(&shifted_path, offset_back)) {
      continue;
    }

    const auto shift_end_idx =
      motion_utils::findNearestIndex(shifted_path.path.points, shift_end_point.point.pose);
    const auto goal_idx = motion_utils::findNearestIndex(shifted_path.path.points, goal_pose);
    if (shift_end_idx && goal_idx) {
      // get target shoulder lane
      lanelet::ConstLanelet target_shoulder_lanelet;
      lanelet::utils::query::getClosestLanelet(
        target_lanelets, shifted_path.path.points.back().point.pose, &target_shoulder_lanelet);

      for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
        auto & point = shifted_path.path.points.at(i);

        // add road lane_ids if not found
        for (const auto id : road_lane_reference_path.points.back().lane_ids) {
          if (std::find(point.lane_ids.begin(), point.lane_ids.end(), id) == point.lane_ids.end()) {
            point.lane_ids.push_back(id);
          }
        }

        // add shoulder lane_id if not found
        if (
          std::find(point.lane_ids.begin(), point.lane_ids.end(), target_shoulder_lanelet.id()) ==
          point.lane_ids.end()) {
          point.lane_ids.push_back(target_shoulder_lanelet.id());
        }

        // set velocity
        if (i < *shift_end_idx) {
          // set velocity during shift
          point.point.longitudinal_velocity_mps = std::min(
            point.point.longitudinal_velocity_mps,
            road_lane_reference_path.points.back().point.longitudinal_velocity_mps);
          continue;
        } else if (i >= *goal_idx) {
          // set velocity after goal
          point.point.longitudinal_velocity_mps = 0.0;
          continue;
        }
        point.point.longitudinal_velocity_mps = pull_over_velocity;
      }

      candidate_path.straight_path = road_lane_reference_path;
      candidate_path.path = combineReferencePath(road_lane_reference_path, shifted_path.path);
      candidate_path.shifted_path = shifted_path;
      shift_point.start_idx = path_shifter.getShiftPoints().front().start_idx;
      shift_point.end_idx = path_shifter.getShiftPoints().front().end_idx;
      candidate_path.shifted_path.shift_length = shifted_path.shift_length;
      candidate_path.shift_point = shift_point;
      // candidate_path.acceleration = acceleration;
      candidate_path.preparation_length = straight_distance;
      candidate_path.pull_over_length = pull_over_distance;
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("pull_over").get_child("util"),
        "lane change end idx not found on target path.");
      continue;
    }

    candidate_paths.push_back(candidate_path);
  }

  return candidate_paths;
}

std::vector<ShiftParkingPath> selectValidPaths(
  const std::vector<ShiftParkingPath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const bool is_in_goal_route_section, const Pose & goal_pose,
  const lane_departure_checker::LaneDepartureChecker & lane_departure_checker)
{
  // combine road and shoulder lanes
  lanelet::ConstLanelets lanes = current_lanes;
  lanes.insert(lanes.end(), target_lanes.begin(), target_lanes.end());

  std::vector<ShiftParkingPath> available_paths;
  for (const auto & path : paths) {
    if (!hasEnoughDistance(
          path, current_lanes, current_pose, is_in_goal_route_section, goal_pose)) {
      continue;
    }

    if (lane_departure_checker.checkPathWillLeaveLane(lanes, path.shifted_path.path)) {
      continue;
    }

    available_paths.push_back(path);
  }

  return available_paths;
}

bool selectSafePath(
  const std::vector<ShiftParkingPath> & paths,
  const OccupancyGridBasedCollisionDetector & occupancy_grid_map, ShiftParkingPath & selected_path)
{
  for (const auto & path : paths) {
    if (!occupancy_grid_map.hasObstacleOnPath(path.shifted_path.path, false)) {
      selected_path = path;
      return true;
    }
  }

  // set first path for force pull over if no valid path found
  if (!paths.empty()) {
    selected_path = paths.front();
    return false;
  }

  return false;
}

bool hasEnoughDistance(
  const ShiftParkingPath & path, const lanelet::ConstLanelets & current_lanes,
  const Pose & current_pose, const bool is_in_goal_route_section, const Pose & goal_pose)
{
  const double pull_over_prepare_distance = path.preparation_length;
  const double pull_over_distance = path.pull_over_length;
  const double pull_over_total_distance = pull_over_prepare_distance + pull_over_distance;

  if (pull_over_total_distance > util::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  if (
    is_in_goal_route_section &&
    pull_over_total_distance > util::getSignedDistance(current_pose, goal_pose, current_lanes)) {
    return false;
  }

  return true;
}

lanelet::ConstLanelets getPullOverLanes(const RouteHandler & route_handler)
{
  lanelet::ConstLanelets pull_over_lanes;
  lanelet::ConstLanelet target_shoulder_lane;
  const Pose goal_pose = route_handler.getGoalPose();

  if (route_handler.getPullOverTarget(
        route_handler.getShoulderLanelets(), goal_pose, &target_shoulder_lane)) {
    constexpr double pull_over_lane_length = 200;
    pull_over_lanes = route_handler.getShoulderLaneletSequence(
      target_shoulder_lane, goal_pose, pull_over_lane_length, pull_over_lane_length);
  }
  return pull_over_lanes;
}

Marker createPullOverAreaMarker(
  const Pose & start_pose, const Pose & end_pose, const int32_t id,
  const std_msgs::msg::Header & header, const double base_link2front, const double base_link2rear,
  const double vehicle_width, const std_msgs::msg::ColorRGBA & color)
{
  using tier4_autoware_utils::createDefaultMarker;
  using tier4_autoware_utils::createMarkerScale;
  using tier4_autoware_utils::createPoint;

  Marker marker = createDefaultMarker(
    header.frame_id, header.stamp, "pull_over_area", id,
    visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0), color);

  auto p_left_front = calcOffsetPose(end_pose, base_link2front, vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_left_front.x, p_left_front.y, p_left_front.z));

  auto p_right_front = calcOffsetPose(end_pose, base_link2front, -vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_right_front.x, p_right_front.y, p_right_front.z));

  auto p_right_back = calcOffsetPose(start_pose, -base_link2rear, -vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_right_back.x, p_right_back.y, p_right_back.z));

  auto p_left_back = calcOffsetPose(start_pose, -base_link2rear, vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_left_back.x, p_left_back.y, p_left_back.z));

  marker.points.push_back(createPoint(p_left_front.x, p_left_front.y, p_left_front.z));

  return marker;
}

}  // namespace pull_over_utils
}  // namespace behavior_path_planner
