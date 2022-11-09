// Copyright 2022 TIER IV, Inc.
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

#include "behavior_path_planner/scene_module/pull_over/shift_pull_over.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
ShiftPullOver::ShiftPullOver(
  rclcpp::Node & node, const PullOverParameters & parameters,
  const LaneDepartureChecker & lane_departure_checker,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
: PullOverPlannerBase{node, parameters},
  lane_departure_checker_{lane_departure_checker},
  occupancy_grid_map_{occupancy_grid_map}
{
}

boost::optional<PullOverPath> ShiftPullOver::plan(const Pose & goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;
  const double after_pull_over_straight_distance = parameters_.after_pull_over_straight_distance;
  const double min_jerk = parameters_.minimum_lateral_jerk;
  const double max_jerk = parameters_.maximum_lateral_jerk;
  const int pull_over_sampling_num = parameters_.pull_over_sampling_num;
  const double jerk_resolution = std::abs(max_jerk - min_jerk) / pull_over_sampling_num;

  const auto road_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto shoulder_lanes = pull_over_utils::getPullOverLanes(*route_handler);
  if (road_lanes.empty() || shoulder_lanes.empty()) {
    return {};
  }

  const auto goal_shoulder_arc_coords =
    lanelet::utils::getArcCoordinates(shoulder_lanes, goal_pose);

  // calculate the pose whose longitudinal position is shift end
  // and whose lateral position is center line
  const auto shift_end_shoulder_center_pose = std::invoke([&]() {
    const double s_start = std::max(
      goal_shoulder_arc_coords.length - after_pull_over_straight_distance, 0.0);  // shift end
    const double s_end = s_start + std::numeric_limits<double>::epsilon();
    const auto shoulder_lane_path =
      route_handler->getCenterLinePath(shoulder_lanes, s_start, s_end, true);
    return shoulder_lane_path.points.front().point.pose;
  });

  // calculate shift end pose
  const double shoulder_left_bound_to_center_distance =
    util::getSignedDistanceFromShoulderLeftBoundary(shoulder_lanes, shift_end_shoulder_center_pose);
  const double shoulder_left_bound_to_goal_distance =
    util::getSignedDistanceFromShoulderLeftBoundary(shoulder_lanes, goal_pose);
  const double shoulder_center_to_goal_distance =
    -shoulder_left_bound_to_center_distance + shoulder_left_bound_to_goal_distance;
  const auto shift_end_pose = tier4_autoware_utils::calcOffsetPose(
    shift_end_shoulder_center_pose, 0, shoulder_center_to_goal_distance, 0);

  // calculate lateral distances
  lanelet::ConstLanelet goal_closest_road_lane{};
  lanelet::utils::query::getClosestLanelet(road_lanes, goal_pose, &goal_closest_road_lane);
  const auto road_center_pose =
    lanelet::utils::getClosestCenterPose(goal_closest_road_lane, goal_pose.position);
  const double shoulder_left_bound_to_road_center =
    util::getSignedDistanceFromShoulderLeftBoundary(shoulder_lanes, road_center_pose);
  const double road_center_to_goal_distance =
    -shoulder_left_bound_to_road_center + shoulder_left_bound_to_goal_distance;

  for (double lateral_jerk = min_jerk; lateral_jerk <= max_jerk; lateral_jerk += jerk_resolution) {
    const auto pull_over_path = generatePullOverPath(
      road_lanes, shoulder_lanes, shift_end_pose, goal_pose, lateral_jerk,
      road_center_to_goal_distance, shoulder_center_to_goal_distance,
      shoulder_left_bound_to_goal_distance);
    if (!pull_over_path) continue;
    return *pull_over_path;
  }

  return {};
}

PathWithLaneId ShiftPullOver::generateRoadLaneReferencePath(
  const lanelet::ConstLanelets & road_lanes, const Pose & shift_end_pose,
  const double pull_over_distance) const
{
  const auto & route_handler = planner_data_->route_handler;
  const Pose & current_pose = planner_data_->self_pose->pose;
  const double backward_path_length = planner_data_->parameters.backward_path_length;
  const double pull_over_velocity = parameters_.pull_over_velocity;
  const double deceleration_interval = parameters_.deceleration_interval;

  const auto current_road_arc_coords = lanelet::utils::getArcCoordinates(road_lanes, current_pose);
  const double s_start = current_road_arc_coords.length - backward_path_length;
  const auto shift_end_road_arc_coords =
    lanelet::utils::getArcCoordinates(road_lanes, shift_end_pose);
  double s_end = shift_end_road_arc_coords.length - pull_over_distance;
  s_end = std::max(s_end, s_start + std::numeric_limits<double>::epsilon());
  auto road_lane_reference_path = route_handler->getCenterLinePath(road_lanes, s_start, s_end);
  // resample road straight path and shift source path respectively
  road_lane_reference_path =
    util::resamplePathWithSpline(road_lane_reference_path, resample_interval_);

  // decelerate velocity linearly to minimum pull over velocity
  // (or keep original velocity if it is lower than pull over velocity)
  for (auto & point : road_lane_reference_path.points) {
    const auto arclength = lanelet::utils::getArcCoordinates(road_lanes, point.point.pose).length;
    const double distance_to_pull_over_start =
      std::clamp(s_end - arclength, 0.0, deceleration_interval);
    const auto decelerated_velocity = static_cast<float>(
      distance_to_pull_over_start / deceleration_interval *
        (point.point.longitudinal_velocity_mps - pull_over_velocity) +
      pull_over_velocity);
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, decelerated_velocity);
  }
  return road_lane_reference_path;
}

PathWithLaneId ShiftPullOver::generateShoulderLaneReferencePath(
  const lanelet::ConstLanelets & shoulder_lanes, const Pose & shift_start_pose,
  const Pose & goal_pose, const double shoulder_center_to_goal_distance) const
{
  const auto & route_handler = planner_data_->route_handler;

  const auto shift_start_shoulder_arc_coords =
    lanelet::utils::getArcCoordinates(shoulder_lanes, shift_start_pose);
  const double s_start = shift_start_shoulder_arc_coords.length;
  const auto goal_shoulder_arc_coords =
    lanelet::utils::getArcCoordinates(shoulder_lanes, goal_pose);
  const double s_end = goal_shoulder_arc_coords.length;
  auto shoulder_lane_reference_path =
    route_handler->getCenterLinePath(shoulder_lanes, s_start, s_end);
  // offset to goal line
  for (auto & p : shoulder_lane_reference_path.points) {
    p.point.pose =
      tier4_autoware_utils::calcOffsetPose(p.point.pose, 0, shoulder_center_to_goal_distance, 0);
  }
  shoulder_lane_reference_path =
    util::resamplePathWithSpline(shoulder_lane_reference_path, resample_interval_);

  return shoulder_lane_reference_path;
}

boost::optional<PullOverPath> ShiftPullOver::generatePullOverPath(
  const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
  const Pose & shift_end_pose, const Pose & goal_pose, const double lateral_jerk,
  const double road_center_to_goal_distance, const double shoulder_center_to_goal_distance,
  const double shoulder_left_bound_to_goal_distance) const
{
  const double pull_over_velocity = parameters_.pull_over_velocity;

  // generate road lane reference path and get shift start pose
  const double pull_over_distance = PathShifter::calcLongitudinalDistFromJerk(
    road_center_to_goal_distance, lateral_jerk, pull_over_velocity);
  const auto road_lane_reference_path =
    generateRoadLaneReferencePath(road_lanes, shift_end_pose, pull_over_distance);
  if (road_lane_reference_path.points.empty()) return {};
  const auto shift_start_pose = road_lane_reference_path.points.back().point.pose;

  // generate Shoulder lane reference path and set it path_shifter
  const auto shoulder_lane_reference_path = generateShoulderLaneReferencePath(
    shoulder_lanes, shift_start_pose, goal_pose, shoulder_center_to_goal_distance);
  if (shoulder_lane_reference_path.points.empty()) return {};
  PathShifter path_shifter{};
  path_shifter.setPath(shoulder_lane_reference_path);

  // generate shift_line and set it to path_sifter
  ShiftLine shift_line{};
  {
    shift_line.start = shift_start_pose;
    shift_line.end = shift_end_pose;
    const double shoulder_left_bound_to_shift_start_distance =
      util::getSignedDistanceFromShoulderLeftBoundary(shoulder_lanes, shift_start_pose);
    const double goal_to_shift_start_distance =
      shoulder_left_bound_to_shift_start_distance - shoulder_left_bound_to_goal_distance;
    shift_line.end_shift_length = goal_to_shift_start_distance;
    path_shifter.addShiftLine(shift_line);
  }

  // offset front side from reference path
  ShiftedPath shifted_path{};
  const bool offset_back = false;
  if (!path_shifter.generate(&shifted_path, offset_back)) return {};

  // check lane departure with road and shoulder lanes
  lanelet::ConstLanelets lanes = road_lanes;
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());
  if (lane_departure_checker_.checkPathWillLeaveLane(lanes, shifted_path.path)) return {};

  // check collision
  if (!isSafePath(shifted_path.path)) return {};

  // set lane_id and velocity to shifted_path
  const auto goal_idx = motion_utils::findNearestIndex(shifted_path.path.points, goal_pose);
  lanelet::ConstLanelet target_shoulder_lanelet{};
  lanelet::utils::query::getClosestLanelet(
    shoulder_lanes, shifted_path.path.points.back().point.pose, &target_shoulder_lanelet);
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
    if (i >= *goal_idx) {
      // set velocity after goal
      point.point.longitudinal_velocity_mps = 0.0;
    } else {
      point.point.longitudinal_velocity_mps = pull_over_velocity;
    }
  }

  PullOverPath pull_over_path{};
  pull_over_path.path =
    pull_over_utils::combineReferencePath(road_lane_reference_path, shifted_path.path);
  // shift path is connected to one, so partial_paths have only one
  pull_over_path.partial_paths.push_back(pull_over_path.path);
  pull_over_path.start_pose = path_shifter.getShiftLines().front().start;
  pull_over_path.end_pose = path_shifter.getShiftLines().front().end;

  // check enough distance
  if (!hasEnoughDistance(
        pull_over_path.path, road_lanes, pull_over_path.start_pose, goal_pose,
        pull_over_distance)) {
    return {};
  }

  return pull_over_path;
}

bool ShiftPullOver::hasEnoughDistance(
  const PathWithLaneId & path, const lanelet::ConstLanelets & road_lanes, const Pose & start_pose,
  const Pose & goal_pose, const double pull_over_distance) const
{
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & common_params = planner_data_->parameters;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  const size_t ego_segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, current_pose, common_params.ego_nearest_dist_threshold,
    common_params.ego_nearest_yaw_threshold);
  const size_t start_segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, start_pose, common_params.ego_nearest_dist_threshold,
    common_params.ego_nearest_yaw_threshold);
  const double dist_to_start_pose = motion_utils::calcSignedArcLength(
    path.points, current_pose.position, ego_segment_idx, start_pose.position, start_segment_idx);

  // once stopped, it cannot start again if start_pose is close.
  // so need enough distance to restart
  constexpr double eps_vel = 0.01;
  // dist to restart should be less than decide_path_distance.
  // otherwise, the goal would change immediately after departure.
  const double dist_to_restart = parameters_.decide_path_distance / 2;
  if (std::abs(current_vel) < eps_vel && dist_to_start_pose < dist_to_restart) {
    return false;
  }
  const double current_to_stop_distance =
    std::pow(current_vel, 2) / parameters_.maximum_deceleration / 2;
  if (dist_to_start_pose < current_to_stop_distance) {
    return false;
  }

  const double road_lane_dist_to_goal = dist_to_start_pose + pull_over_distance;
  if (road_lane_dist_to_goal > util::getDistanceToEndOfLane(current_pose, road_lanes)) {
    return false;
  }

  const bool is_in_goal_route_section =
    planner_data_->route_handler->isInGoalRouteSection(road_lanes.back());
  if (
    is_in_goal_route_section &&
    road_lane_dist_to_goal > util::getSignedDistance(current_pose, goal_pose, road_lanes)) {
    return false;
  }

  return true;
}

bool ShiftPullOver::isSafePath(const PathWithLaneId & path) const
{
  if (parameters_.use_occupancy_grid || !occupancy_grid_map_) {
    const bool check_out_of_range = false;
    if (occupancy_grid_map_->hasObstacleOnPath(path, check_out_of_range)) {
      return false;
    }
  }

  if (parameters_.use_object_recognition) {
    if (util::checkCollisionBetweenPathFootprintsAndObjects(
          vehicle_footprint_, path, *(planner_data_->dynamic_object),
          parameters_.object_recognition_collision_check_margin)) {
      return false;
    }
  }

  return true;
}

}  // namespace behavior_path_planner
