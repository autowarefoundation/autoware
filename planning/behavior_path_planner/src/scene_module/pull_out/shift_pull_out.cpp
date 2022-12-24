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

#include "behavior_path_planner/scene_module/pull_out/shift_pull_out.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_out/util.hpp"

#include <memory>
#include <vector>

using lanelet::utils::getArcCoordinates;
using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
namespace behavior_path_planner
{
using pull_out_utils::combineReferencePath;
using pull_out_utils::getPullOutLanes;

ShiftPullOut::ShiftPullOut(
  rclcpp::Node & node, const PullOutParameters & parameters,
  std::shared_ptr<LaneDepartureChecker> & lane_departure_checker)
: PullOutPlannerBase{node, parameters}, lane_departure_checker_{lane_departure_checker}
{
}

boost::optional<PullOutPath> ShiftPullOut::plan(Pose start_pose, Pose goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & common_parameters = planner_data_->parameters;
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & road_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto & shoulder_lanes = getPullOutLanes(planner_data_);
  if (shoulder_lanes.empty()) {
    return boost::none;
  }

  // find candidate paths
  auto pull_out_paths = calcPullOutPaths(
    *route_handler, road_lanes, shoulder_lanes, start_pose, goal_pose, common_parameters,
    parameters_);
  if (pull_out_paths.empty()) {
    return boost::none;
  }

  // extract objects in shoulder lane for collision check
  const auto [shoulder_lane_objects, others] =
    util::separateObjectsByLanelets(*dynamic_objects, shoulder_lanes);

  // get safe path
  for (auto & pull_out_path : pull_out_paths) {
    auto & shift_path =
      pull_out_path.partial_paths.front();  // shift path is not separate but only one.

    // check lane_departure and collision with path between pull_out_start to pull_out_end
    PathWithLaneId path_start_to_end{};
    {
      const size_t pull_out_start_idx = findNearestIndex(shift_path.points, start_pose.position);

      // calculate collision check end idx
      const size_t collision_check_end_idx = std::invoke([&]() {
        const auto collision_check_end_pose = motion_utils::calcLongitudinalOffsetPose(
          shift_path.points, pull_out_path.end_pose.position,
          parameters_.collision_check_distance_from_end);

        if (collision_check_end_pose) {
          return findNearestIndex(shift_path.points, collision_check_end_pose->position);
        } else {
          return shift_path.points.size() - 1;
        }
      });
      path_start_to_end.points.insert(
        path_start_to_end.points.begin(), shift_path.points.begin() + pull_out_start_idx,
        shift_path.points.begin() + collision_check_end_idx + 1);
    }

    // check lane departure
    const auto drivable_lanes =
      util::generateDrivableLanesWithShoulderLanes(road_lanes, shoulder_lanes);
    const auto expanded_lanes = util::expandLanelets(
      drivable_lanes, parameters_.drivable_area_left_bound_offset,
      parameters_.drivable_area_right_bound_offset);
    if (lane_departure_checker_->checkPathWillLeaveLane(
          util::transformToLanelets(expanded_lanes), path_start_to_end)) {
      continue;
    }

    // check collision
    if (util::checkCollisionBetweenPathFootprintsAndObjects(
          vehicle_footprint_, path_start_to_end, shoulder_lane_objects,
          parameters_.collision_check_margin)) {
      continue;
    }

    // Generate drivable area
    const auto shorten_lanes = util::cutOverlappedLanes(shift_path, drivable_lanes);
    util::generateDrivableArea(
      shift_path, shorten_lanes, common_parameters.vehicle_length, planner_data_);

    shift_path.header = planner_data_->route_handler->getRouteHeader();

    return pull_out_path;
  }

  return boost::none;
}

std::vector<PullOutPath> ShiftPullOut::calcPullOutPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes, const Pose & start_pose, const Pose & goal_pose,
  const BehaviorPathPlannerParameters & common_parameter, const PullOutParameters & parameter)
{
  std::vector<PullOutPath> candidate_paths{};

  if (road_lanes.empty() || shoulder_lanes.empty()) {
    return candidate_paths;
  }

  // rename parameter
  const double backward_path_length = common_parameter.backward_path_length;
  const double shift_pull_out_velocity = parameter.shift_pull_out_velocity;
  const double minimum_shift_pull_out_distance = parameter.minimum_shift_pull_out_distance;
  const double minimum_lateral_jerk = parameter.minimum_lateral_jerk;
  const double maximum_lateral_jerk = parameter.maximum_lateral_jerk;
  const int pull_out_sampling_num = parameter.pull_out_sampling_num;
  const double jerk_resolution =
    std::abs(maximum_lateral_jerk - minimum_lateral_jerk) / pull_out_sampling_num;

  for (double lateral_jerk = minimum_lateral_jerk; lateral_jerk <= maximum_lateral_jerk;
       lateral_jerk += jerk_resolution) {
    // lateral distance from road center to start pose
    const double shift_length = getArcCoordinates(road_lanes, start_pose).distance;

    PathWithLaneId road_lane_reference_path{};
    {
      const auto arc_position = getArcCoordinates(road_lanes, start_pose);
      const double s_start = std::max(arc_position.length - backward_path_length, 0.0);
      const auto arc_position_goal = getArcCoordinates(road_lanes, goal_pose);
      double s_end = arc_position_goal.length;
      road_lane_reference_path = util::resamplePathWithSpline(
        route_handler.getCenterLinePath(road_lanes, s_start, s_end), 1.0);
    }
    PathShifter path_shifter{};
    path_shifter.setPath(road_lane_reference_path);

    // calculate after/before shifted pull out distance
    const double pull_out_distance = std::max(
      PathShifter::calcLongitudinalDistFromJerk(
        abs(shift_length), lateral_jerk, shift_pull_out_velocity),
      minimum_shift_pull_out_distance);
    const size_t shift_start_idx =
      findNearestIndex(road_lane_reference_path.points, start_pose.position);
    PathWithLaneId road_lane_reference_path_from_ego = road_lane_reference_path;
    road_lane_reference_path_from_ego.points.erase(
      road_lane_reference_path_from_ego.points.begin(),
      road_lane_reference_path_from_ego.points.begin() + shift_start_idx);
    // before means distance on road lane
    const double before_shifted_pull_out_distance = calcBeforeShiftedArcLength(
      road_lane_reference_path_from_ego, pull_out_distance, shift_length);

    // check has enough distance
    const bool is_in_goal_route_section = route_handler.isInGoalRouteSection(road_lanes.back());
    if (!hasEnoughDistance(
          before_shifted_pull_out_distance, road_lanes, start_pose, is_in_goal_route_section,
          goal_pose)) {
      continue;
    }

    // get shift end pose
    const auto shift_end_pose = std::invoke([&]() {
      const auto arc_position_shift_start =
        lanelet::utils::getArcCoordinates(road_lanes, start_pose);
      const double s_start = arc_position_shift_start.length + before_shifted_pull_out_distance;
      const double s_end = s_start + std::numeric_limits<double>::epsilon();
      const auto path = route_handler.getCenterLinePath(road_lanes, s_start, s_end, true);
      return path.points.front().point.pose;
    });

    // create shift line
    ShiftLine shift_line{};
    shift_line.start = start_pose;
    shift_line.end = shift_end_pose;
    shift_line.end_shift_length = shift_length;
    path_shifter.addShiftLine(shift_line);

    // offset front side
    ShiftedPath shifted_path;
    const bool offset_back = false;
    if (!path_shifter.generate(&shifted_path, offset_back)) {
      continue;
    }

    // set velocity
    const size_t pull_out_end_idx =
      findNearestIndex(shifted_path.path.points, shift_end_pose.position);
    const size_t goal_idx = findNearestIndex(shifted_path.path.points, goal_pose.position);
    for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
      auto & point = shifted_path.path.points.at(i);
      if (i < pull_out_end_idx) {
        point.point.longitudinal_velocity_mps = std::min(
          point.point.longitudinal_velocity_mps, static_cast<float>(shift_pull_out_velocity));
        continue;
      } else if (i >= goal_idx) {
        point.point.longitudinal_velocity_mps = 0.0;
        continue;
      }
    }

    // add shifted path to candidates
    PullOutPath candidate_path;
    candidate_path.partial_paths.push_back(shifted_path.path);
    candidate_path.start_pose = shift_line.start;
    candidate_path.end_pose = shift_line.end;
    candidate_paths.push_back(candidate_path);
  }

  return candidate_paths;
}

bool ShiftPullOut::hasEnoughDistance(
  const double pull_out_total_distance, const lanelet::ConstLanelets & road_lanes,
  const Pose & current_pose, const bool is_in_goal_route_section, const Pose & goal_pose)
{
  // the goal is far so current_lanes do not include goal's lane
  if (pull_out_total_distance > util::getDistanceToEndOfLane(current_pose, road_lanes)) {
    return false;
  }

  // current_lanes include goal's lane
  if (
    is_in_goal_route_section &&
    pull_out_total_distance > util::getSignedDistance(current_pose, goal_pose, road_lanes)) {
    return false;
  }

  return true;
}

double ShiftPullOut::calcBeforeShiftedArcLength(
  const PathWithLaneId & path, const double target_after_arc_length, const double dr)
{
  double before_arc_length{0.0};
  double after_arc_length{0.0};

  for (const auto & [k, segment_length] : motion_utils::calcCurvatureAndArcLength(path.points)) {
    // after shifted segment length
    const double after_segment_length =
      k < 0 ? segment_length * (1 - k * dr) : segment_length / (1 + k * dr);
    if (after_arc_length + after_segment_length > target_after_arc_length) {
      const double offset = target_after_arc_length - after_arc_length;
      before_arc_length += k < 0 ? offset / (1 - k * dr) : offset * (1 + k * dr);
      break;
    }
    before_arc_length += segment_length;
    after_arc_length += after_segment_length;
  }

  return before_arc_length;
}

}  // namespace behavior_path_planner
