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

#include "behavior_path_start_planner_module/shift_pull_out.hpp"

#include "behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "behavior_path_start_planner_module/util.hpp"
#include "motion_utils/trajectory/path_with_lane_id.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

using lanelet::utils::getArcCoordinates;
using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
namespace behavior_path_planner
{
using start_planner_utils::getPullOutLanes;

ShiftPullOut::ShiftPullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  std::shared_ptr<LaneDepartureChecker> & lane_departure_checker)
: PullOutPlannerBase{node, parameters}, lane_departure_checker_{lane_departure_checker}
{
}

std::optional<PullOutPath> ShiftPullOut::plan(const Pose & start_pose, const Pose & goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & common_parameters = planner_data_->parameters;

  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_.max_back_distance;
  const auto pull_out_lanes = getPullOutLanes(planner_data_, backward_path_length);
  if (pull_out_lanes.empty()) {
    return std::nullopt;
  }

  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
  // find candidate paths
  auto pull_out_paths = calcPullOutPaths(*route_handler, road_lanes, start_pose, goal_pose);
  if (pull_out_paths.empty()) {
    return std::nullopt;
  }

  // get safe path
  for (auto & pull_out_path : pull_out_paths) {
    auto & shift_path =
      pull_out_path.partial_paths.front();  // shift path is not separate but only one.

    // check lane_departure with path between pull_out_start to pull_out_end
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

    // extract shoulder lanes from pull out lanes
    lanelet::ConstLanelets shoulder_lanes;
    std::copy_if(
      pull_out_lanes.begin(), pull_out_lanes.end(), std::back_inserter(shoulder_lanes),
      [&route_handler](const auto & pull_out_lane) {
        return route_handler->isShoulderLanelet(pull_out_lane);
      });
    const auto drivable_lanes =
      utils::generateDrivableLanesWithShoulderLanes(road_lanes, shoulder_lanes);
    const auto & dp = planner_data_->drivable_area_expansion_parameters;
    const auto expanded_lanes = utils::transformToLanelets(utils::expandLanelets(
      drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
      dp.drivable_area_types_to_skip));

    // crop backward path
    // removes points which are out of lanes up to the start pose.
    // this ensures that the backward_path stays within the drivable area when starting from a
    // narrow place.
    const size_t start_segment_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
      shift_path.points, start_pose, common_parameters.ego_nearest_dist_threshold,
      common_parameters.ego_nearest_yaw_threshold);
    PathWithLaneId cropped_path{};
    for (size_t i = 0; i < shift_path.points.size(); ++i) {
      const Pose pose = shift_path.points.at(i).point.pose;
      const auto transformed_vehicle_footprint =
        transformVector(vehicle_footprint_, tier4_autoware_utils::pose2transform(pose));
      const bool is_out_of_lane =
        LaneDepartureChecker::isOutOfLane(expanded_lanes, transformed_vehicle_footprint);
      if (i <= start_segment_idx) {
        if (!is_out_of_lane) {
          cropped_path.points.push_back(shift_path.points.at(i));
        }
      } else {
        cropped_path.points.push_back(shift_path.points.at(i));
      }
    }
    shift_path.points = cropped_path.points;

    // check lane departure
    if (
      parameters_.check_shift_path_lane_departure &&
      lane_departure_checker_->checkPathWillLeaveLane(expanded_lanes, path_start_to_end)) {
      continue;
    }

    shift_path.header = planner_data_->route_handler->getRouteHeader();

    return pull_out_path;
  }

  return std::nullopt;
}

bool ShiftPullOut::refineShiftedPathToStartPose(
  ShiftedPath & shifted_path, const Pose & start_pose, const Pose & end_pose,
  const double longitudinal_acc, const double lateral_acc)
{
  constexpr double TOLERANCE = 0.01;
  constexpr size_t MAX_ITERATION = 100;

  // Lambda to check if change is above tolerance
  auto is_within_tolerance =
    [](const auto & prev_val, const auto & current_val, const auto & tolerance) {
      return std::abs(current_val - prev_val) < tolerance;
    };

  size_t iteration = 0;
  while (iteration < MAX_ITERATION) {
    const double lateral_offset =
      motion_utils::calcLateralOffset(shifted_path.path.points, start_pose.position);

    PathShifter path_shifter;
    path_shifter.setPath(shifted_path.path);
    ShiftLine shift_line{};
    shift_line.start = start_pose;
    shift_line.end = end_pose;
    shift_line.end_shift_length = lateral_offset;
    path_shifter.addShiftLine(shift_line);
    path_shifter.setVelocity(0.0);
    path_shifter.setLongitudinalAcceleration(longitudinal_acc);
    path_shifter.setLateralAccelerationLimit(lateral_acc);

    if (!path_shifter.generate(&shifted_path, false)) {
      RCLCPP_WARN(
        rclcpp::get_logger("ShiftPullOut:refineShiftedPathToStartPose()"),
        "Failed to generate shifted path");
      return false;
    }

    if (is_within_tolerance(
          lateral_offset,
          motion_utils::calcLateralOffset(shifted_path.path.points, start_pose.position),
          TOLERANCE)) {
      return true;
    }

    iteration++;
  }

  RCLCPP_WARN(
    rclcpp::get_logger("ShiftPullOut:refineShiftedPathToStartPose()"),
    "Failed to converge lateral offset until max iteration");
  return false;
}

std::vector<PullOutPath> ShiftPullOut::calcPullOutPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & road_lanes,
  const Pose & start_pose, const Pose & goal_pose)
{
  std::vector<PullOutPath> candidate_paths{};

  if (road_lanes.empty()) {
    return candidate_paths;
  }

  // rename parameter
  const auto & common_parameters = planner_data_->parameters;
  const double forward_path_length = common_parameters.forward_path_length;
  const double backward_path_length = common_parameters.backward_path_length;
  const double lateral_jerk = parameters_.lateral_jerk;
  const double minimum_lateral_acc = parameters_.minimum_lateral_acc;
  const double maximum_lateral_acc = parameters_.maximum_lateral_acc;
  const double maximum_curvature = parameters_.maximum_curvature;
  const double minimum_shift_pull_out_distance = parameters_.minimum_shift_pull_out_distance;
  const int lateral_acceleration_sampling_num = parameters_.lateral_acceleration_sampling_num;

  // set minimum acc for breaking loop when sampling num is 1
  const double acc_resolution = std::max(
    std::abs(maximum_lateral_acc - minimum_lateral_acc) / lateral_acceleration_sampling_num,
    std::numeric_limits<double>::epsilon());

  // generate road lane reference path
  const auto arc_position_start = getArcCoordinates(road_lanes, start_pose);
  const double s_start = std::max(arc_position_start.length - backward_path_length, 0.0);
  const auto path_end_info = behavior_path_planner::utils::parking_departure::calcEndArcLength(
    s_start, forward_path_length, road_lanes, goal_pose);
  const double s_end = path_end_info.first;
  const bool path_terminal_is_goal = path_end_info.second;

  PathWithLaneId road_lane_reference_path = utils::resamplePathWithSpline(
    route_handler.getCenterLinePath(road_lanes, s_start, s_end),
    parameters_.center_line_path_interval);

  // non_shifted_path for when shift length or pull out distance is too short
  const PullOutPath non_shifted_path = std::invoke([&]() {
    PullOutPath non_shifted_path{};
    // In non_shifted_path, to minimize safety checks, 0 is assigned to prevent the predicted_path
    // of the ego vehicle from becoming too large.
    non_shifted_path.partial_paths.push_back(road_lane_reference_path);
    non_shifted_path.start_pose = start_pose;
    non_shifted_path.end_pose = start_pose;
    non_shifted_path.pairs_terminal_velocity_and_accel.push_back(std::make_pair(0, 0));
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
    const double longitudinal_acc = std::clamp(road_velocity / shift_time, 0.0, /* max acc */ 1.0);
    const auto pull_out_distance = calcPullOutLongitudinalDistance(
      longitudinal_acc, shift_time, shift_length, maximum_curvature,
      minimum_shift_pull_out_distance);
    const double terminal_velocity = longitudinal_acc * shift_time;

    // clip from ego pose
    PathWithLaneId road_lane_reference_path_from_ego = road_lane_reference_path;
    road_lane_reference_path_from_ego.points.erase(
      road_lane_reference_path_from_ego.points.begin(),
      road_lane_reference_path_from_ego.points.begin() + shift_start_idx);
    // before means distance on road lane
    // Note: the pull_out_distance is the required distance on the shifted path. Now we need to
    // calculate the distance on the center line used for the shift path generation. However, since
    // the calcBeforeShiftedArcLength is an approximate conversion from center line to center line
    // (not shift path to centerline), the conversion result may too long or short. To prevent too
    // short length, take maximum with the original distance.
    // TODO(kosuke55): update the conversion function and get rid of the comparison with original
    // distance.
    const double pull_out_distance_converted = calcBeforeShiftedArcLength(
      road_lane_reference_path_from_ego, pull_out_distance, shift_length);
    const double before_shifted_pull_out_distance =
      std::max(pull_out_distance, pull_out_distance_converted);

    // if before_shifted_pull_out_distance is too short, shifting path fails, so add non shifted
    if (
      before_shifted_pull_out_distance < parameters_.center_line_path_interval &&
      !has_non_shifted_path) {
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

    const auto shift_line_idx = path_shifter.getShiftLines().front();
    if (!has_non_shifted_path && (shift_line_idx.end_idx - shift_line_idx.start_idx <= 1)) {
      candidate_paths.push_back(non_shifted_path);
      has_non_shifted_path = true;
      continue;
    }

    // offset front side
    ShiftedPath shifted_path;
    const bool offset_back = false;
    if (!path_shifter.generate(&shifted_path, offset_back)) {
      continue;
    }
    refineShiftedPathToStartPose(
      shifted_path, start_pose, *shift_end_pose_ptr, longitudinal_acc, lateral_acc);

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
    if (path_terminal_is_goal) {
      shifted_path.path.points.back().point.longitudinal_velocity_mps = 0.0;
    }

    // add shifted path to candidates
    PullOutPath candidate_path;
    candidate_path.partial_paths.push_back(shifted_path.path);
    candidate_path.start_pose = shift_line.start;
    candidate_path.end_pose = shift_line.end;
    candidate_path.pairs_terminal_velocity_and_accel.push_back(
      std::make_pair(terminal_velocity, longitudinal_acc));
    candidate_paths.push_back(candidate_path);
  }

  return candidate_paths;
}

double ShiftPullOut::calcPullOutLongitudinalDistance(
  const double lon_acc, const double shift_time, const double shift_length,
  const double max_curvature, const double min_distance) const
{
  // Required distance for acceleration limit
  const double min_pull_out_distance_by_acc = (lon_acc * std::pow(shift_time, 2)) / 2.0;

  // Required distance for curvature limit
  const auto min_pull_out_distance_by_curvature = [&]() {
    // Simple model for the shifted path by a double circular-arc approximation on a straight road.
    const double distance =
      std::sqrt(std::max(4.0 * shift_length / max_curvature - shift_length * shift_length, 0.0));
    return distance;
  }();

  // Take all requirements
  const auto min_pull_out_distance = std::max(
    std::max(min_pull_out_distance_by_acc, min_pull_out_distance_by_curvature), min_distance);

  return min_pull_out_distance;
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
