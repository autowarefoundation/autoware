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

#include "behavior_path_planner/utils/start_planner/shift_pull_out.hpp"

#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/start_planner/util.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

using lanelet::utils::getArcCoordinates;
using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
namespace behavior_path_planner
{
using start_planner_utils::combineReferencePath;
using start_planner_utils::getPullOutLanes;

ShiftPullOut::ShiftPullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  std::shared_ptr<LaneDepartureChecker> & lane_departure_checker)
: PullOutPlannerBase{node, parameters}, lane_departure_checker_{lane_departure_checker}
{
}

boost::optional<PullOutPath> ShiftPullOut::plan(Pose start_pose, Pose goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & common_parameters = planner_data_->parameters;
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto shoulder_lanes = getPullOutLanes(planner_data_);
  if (shoulder_lanes.empty()) {
    return boost::none;
  }

  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_.max_back_distance;
  const auto road_lanes =
    utils::getCurrentLanes(planner_data_, backward_path_length, std::numeric_limits<double>::max());

  // find candidate paths
  auto pull_out_paths = calcPullOutPaths(
    *route_handler, road_lanes, shoulder_lanes, start_pose, goal_pose, common_parameters,
    parameters_);
  if (pull_out_paths.empty()) {
    return boost::none;
  }

  // extract objects in shoulder lane for collision check
  const auto [shoulder_lane_objects, others] =
    utils::separateObjectsByLanelets(*dynamic_objects, shoulder_lanes);

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
      utils::generateDrivableLanesWithShoulderLanes(road_lanes, shoulder_lanes);
    const auto & dp = planner_data_->drivable_area_expansion_parameters;
    const auto expanded_lanes = utils::expandLanelets(
      drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
      dp.drivable_area_types_to_skip);
    if (
      parameters_.check_shift_path_lane_departure &&
      lane_departure_checker_->checkPathWillLeaveLane(
        utils::transformToLanelets(expanded_lanes), path_start_to_end)) {
      continue;
    }

    // check collision
    if (utils::checkCollisionBetweenPathFootprintsAndObjects(
          vehicle_footprint_, path_start_to_end, shoulder_lane_objects,
          parameters_.collision_check_margin)) {
      continue;
    }

    // Generate drivable area
    // for old architecture
    // NOTE: drivable_area_info is assigned outside this function.
    const auto shorten_lanes = utils::cutOverlappedLanes(shift_path, drivable_lanes);
    utils::generateDrivableArea(
      shift_path, shorten_lanes, false, common_parameters.vehicle_length, planner_data_);

    shift_path.header = planner_data_->route_handler->getRouteHeader();

    return pull_out_path;
  }

  return boost::none;
}

std::vector<PullOutPath> ShiftPullOut::calcPullOutPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & road_lanes,
  const lanelet::ConstLanelets & shoulder_lanes, const Pose & start_pose, const Pose & goal_pose,
  const BehaviorPathPlannerParameters & common_parameter, const StartPlannerParameters & parameter)
{
  std::vector<PullOutPath> candidate_paths{};

  if (road_lanes.empty() || shoulder_lanes.empty()) {
    return candidate_paths;
  }

  // rename parameter
  const double forward_path_length = common_parameter.forward_path_length;
  const double backward_path_length = common_parameter.backward_path_length;
  const double minimum_shift_pull_out_distance = parameter.minimum_shift_pull_out_distance;
  const double lateral_jerk = parameter.lateral_jerk;
  const double minimum_lateral_acc = parameter.minimum_lateral_acc;
  const double maximum_lateral_acc = parameter.maximum_lateral_acc;
  const int lateral_acceleration_sampling_num = parameter.lateral_acceleration_sampling_num;
  // set minimum acc for breaking loop when sampling num is 1
  const double acc_resolution = std::max(
    std::abs(maximum_lateral_acc - minimum_lateral_acc) / lateral_acceleration_sampling_num,
    std::numeric_limits<double>::epsilon());

  // generate road lane reference path
  const auto arc_position_start = getArcCoordinates(road_lanes, start_pose);
  const double s_start = std::max(arc_position_start.length - backward_path_length, 0.0);
  const auto arc_position_goal = getArcCoordinates(road_lanes, goal_pose);

  // if goal is behind start pose, use path with forward_path_length
  const bool goal_is_behind = arc_position_goal.length < s_start;
  const double s_forward_length = s_start + forward_path_length;
  const double s_end =
    goal_is_behind ? s_forward_length : std::min(arc_position_goal.length, s_forward_length);

  constexpr double RESAMPLE_INTERVAL = 1.0;
  PathWithLaneId road_lane_reference_path = utils::resamplePathWithSpline(
    route_handler.getCenterLinePath(road_lanes, s_start, s_end), RESAMPLE_INTERVAL);

  // non_shifted_path for when shift length or pull out distance is too short
  const PullOutPath non_shifted_path = std::invoke([&]() {
    PullOutPath non_shifted_path{};
    non_shifted_path.partial_paths.push_back(road_lane_reference_path);
    non_shifted_path.start_pose = start_pose;
    non_shifted_path.end_pose = start_pose;
    return non_shifted_path;
  });

  bool has_non_shifted_path = false;
  for (double lateral_acc = minimum_lateral_acc; lateral_acc <= maximum_lateral_acc;
       lateral_acc += acc_resolution) {
    PathShifter path_shifter{};

    path_shifter.setPath(road_lane_reference_path);

    // if shift length is too short, add non sifted path
    constexpr double MINIMUM_SHIFT_LENGTH = 0.01;
    const double shift_length = getArcCoordinates(road_lanes, start_pose).distance;
    if (std::abs(shift_length) < MINIMUM_SHIFT_LENGTH && !has_non_shifted_path) {
      candidate_paths.push_back(non_shifted_path);
      has_non_shifted_path = true;
      continue;
    }

    // calculate pull out distance, longitudinal acc, terminal velocity
    const size_t shift_start_idx =
      findNearestIndex(road_lane_reference_path.points, start_pose.position);
    const double road_velocity =
      road_lane_reference_path.points.at(shift_start_idx).point.longitudinal_velocity_mps;
    const double shift_time =
      PathShifter::calcShiftTimeFromJerk(shift_length, lateral_jerk, lateral_acc);
    const double longitudinal_acc = std::clamp(road_velocity / shift_time, 0.0, 1.0);
    const double pull_out_distance = (longitudinal_acc * std::pow(shift_time, 2)) / 2.0;
    const double terminal_velocity = longitudinal_acc * shift_time;

    // clip from ego pose
    PathWithLaneId road_lane_reference_path_from_ego = road_lane_reference_path;
    road_lane_reference_path_from_ego.points.erase(
      road_lane_reference_path_from_ego.points.begin(),
      road_lane_reference_path_from_ego.points.begin() + shift_start_idx);
    // before means distance on road lane
    const double before_shifted_pull_out_distance = std::max(
      minimum_shift_pull_out_distance,
      calcBeforeShiftedArcLength(
        road_lane_reference_path_from_ego, pull_out_distance, shift_length));

    // check has enough distance
    const bool is_in_goal_route_section = route_handler.isInGoalRouteSection(road_lanes.back());
    if (!hasEnoughDistance(
          before_shifted_pull_out_distance, road_lanes, start_pose, is_in_goal_route_section,
          goal_pose)) {
      continue;
    }

    // if before_shifted_pull_out_distance is too short, shifting path fails, so add non shifted
    if (before_shifted_pull_out_distance < RESAMPLE_INTERVAL && !has_non_shifted_path) {
      candidate_paths.push_back(non_shifted_path);
      has_non_shifted_path = true;
      continue;
    }

    // get shift end pose
    const auto shift_end_pose_ptr = std::invoke([&]() {
      const auto arc_position_shift_start =
        lanelet::utils::getArcCoordinates(road_lanes, start_pose);
      const double s_start = arc_position_shift_start.length + before_shifted_pull_out_distance;
      const double s_end = s_start + std::numeric_limits<double>::epsilon();
      const auto path = route_handler.getCenterLinePath(road_lanes, s_start, s_end, true);
      return path.points.empty()
               ? nullptr
               : std::make_shared<geometry_msgs::msg::Pose>(path.points.front().point.pose);
    });

    if (!shift_end_pose_ptr) {
      continue;
    }

    // create shift line
    ShiftLine shift_line{};
    shift_line.start = start_pose;
    shift_line.end = *shift_end_pose_ptr;
    shift_line.end_shift_length = shift_length;
    path_shifter.addShiftLine(shift_line);
    path_shifter.setVelocity(0.0);  // initial velocity is 0
    path_shifter.setLongitudinalAcceleration(longitudinal_acc);
    path_shifter.setLateralAccelerationLimit(lateral_acc);

    // offset front side
    ShiftedPath shifted_path;
    const bool offset_back = false;
    if (!path_shifter.generate(&shifted_path, offset_back)) {
      continue;
    }

    // set velocity
    const size_t pull_out_end_idx =
      findNearestIndex(shifted_path.path.points, shift_end_pose_ptr->position);
    for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
      auto & point = shifted_path.path.points.at(i);
      if (i < pull_out_end_idx) {
        point.point.longitudinal_velocity_mps =
          std::min(point.point.longitudinal_velocity_mps, static_cast<float>(terminal_velocity));
      }
    }
    // if the end point is the goal, set the velocity to 0
    if (!goal_is_behind) {
      shifted_path.path.points.back().point.longitudinal_velocity_mps = 0.0;
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
  if (pull_out_total_distance > utils::getDistanceToEndOfLane(current_pose, road_lanes)) {
    return false;
  }

  // current_lanes include goal's lane
  if (
    is_in_goal_route_section &&
    pull_out_total_distance > utils::getSignedDistance(current_pose, goal_pose, road_lanes)) {
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
