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

#include "autoware/behavior_path_start_planner_module/shift_pull_out.hpp"

#include "autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

using autoware::motion_utils::findNearestIndex;
using autoware::universe_utils::calcDistance2d;
using autoware::universe_utils::calcOffsetPose;
using lanelet::utils::getArcCoordinates;
namespace autoware::behavior_path_planner
{
using start_planner_utils::getPullOutLanes;

ShiftPullOut::ShiftPullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  std::shared_ptr<LaneDepartureChecker> & lane_departure_checker,
  std::shared_ptr<universe_utils::TimeKeeper> time_keeper)
: PullOutPlannerBase{node, parameters, time_keeper}, lane_departure_checker_{lane_departure_checker}
{
}

std::optional<PullOutPath> ShiftPullOut::plan(
  const Pose & start_pose, const Pose & goal_pose, PlannerDebugData & planner_debug_data)
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & common_parameters = planner_data_->parameters;

  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_.max_back_distance;
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
  // find candidate paths
  auto pull_out_paths = calcPullOutPaths(*route_handler, road_lanes, start_pose, goal_pose);
  if (pull_out_paths.empty()) {
    planner_debug_data.conditions_evaluation.emplace_back("no path found");
    return std::nullopt;
  }

  // get safe path
  for (auto & pull_out_path : pull_out_paths) {
    universe_utils::ScopedTimeTrack st("get safe path", *time_keeper_);

    // shift path is not separate but only one.
    auto & shift_path = pull_out_path.partial_paths.front();
    // check lane_departure with path between pull_out_start to pull_out_end
    PathWithLaneId path_shift_start_to_end{};
    {
      const size_t pull_out_start_idx = findNearestIndex(shift_path.points, start_pose.position);
      const size_t pull_out_end_idx =
        findNearestIndex(shift_path.points, pull_out_path.end_pose.position);

      path_shift_start_to_end.points.insert(
        path_shift_start_to_end.points.begin(), shift_path.points.begin() + pull_out_start_idx,
        shift_path.points.begin() + pull_out_end_idx + 1);
    }

    const auto lanelet_map_ptr = planner_data_->route_handler->getLaneletMapPtr();

    // if lane departure check override is true, and if the initial pose is not fully within a lane,
    // cancel lane departure check
    const bool is_lane_departure_check_required = std::invoke([&]() -> bool {
      if (!parameters_.allow_check_shift_path_lane_departure_override)
        return parameters_.check_shift_path_lane_departure;

      PathWithLaneId path_with_only_first_pose{};
      path_with_only_first_pose.points.push_back(path_shift_start_to_end.points.front());
      return !lane_departure_checker_->checkPathWillLeaveLane(
        lanelet_map_ptr, path_with_only_first_pose);
    });

    // check lane departure
    // The method for lane departure checking verifies if the footprint of each point on the path
    // is contained within a lanelet using `boost::geometry::within`, which incurs a high
    // computational cost.

    if (
      is_lane_departure_check_required &&
      lane_departure_checker_->checkPathWillLeaveLane(lanelet_map_ptr, path_shift_start_to_end)) {
      planner_debug_data.conditions_evaluation.emplace_back("lane departure");
      continue;
    }

    // crop backward path
    // removes points which are out of lanes up to the start pose.
    // this ensures that the backward_path stays within the drivable area when starting from a
    // narrow place.

    const size_t start_segment_idx =
      autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        shift_path.points, start_pose, common_parameters.ego_nearest_dist_threshold,
        common_parameters.ego_nearest_yaw_threshold);

    const auto cropped_path = lane_departure_checker_->cropPointsOutsideOfLanes(
      lanelet_map_ptr, shift_path, start_segment_idx);
    if (cropped_path.points.empty()) {
      planner_debug_data.conditions_evaluation.emplace_back("cropped path is empty");
      continue;
    }

    // check that the path is not cropped in excess and there is not excessive longitudinal
    // deviation between the first 2 points
    auto validate_cropped_path = [&](const auto & cropped_path) -> bool {
      if (cropped_path.points.size() < 2) return false;
      const double max_long_offset = parameters_.maximum_longitudinal_deviation;
      const size_t start_segment_idx_after_crop =
        autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
          cropped_path.points, start_pose);

      // if the start segment id after crop is not 0, then the cropping is not excessive
      if (start_segment_idx_after_crop != 0) return true;

      const auto long_offset_to_closest_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop, start_pose.position);
      const auto long_offset_to_next_point =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          cropped_path.points, start_segment_idx_after_crop + 1, start_pose.position);
      return std::abs(long_offset_to_closest_point - long_offset_to_next_point) < max_long_offset;
    };

    if (!validate_cropped_path(cropped_path)) {
      planner_debug_data.conditions_evaluation.emplace_back("cropped path is invalid");
      continue;
    }
    shift_path.points = cropped_path.points;
    shift_path.header = planner_data_->route_handler->getRouteHeader();

    if (isPullOutPathCollided(pull_out_path, parameters_.shift_collision_check_distance_from_end)) {
      planner_debug_data.conditions_evaluation.emplace_back("collision");
      continue;
    }

    planner_debug_data.conditions_evaluation.emplace_back("success");
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
      autoware::motion_utils::calcLateralOffset(shifted_path.path.points, start_pose.position);

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
          autoware::motion_utils::calcLateralOffset(shifted_path.path.points, start_pose.position),
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
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

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
  const double end_pose_curvature_threshold = parameters_.end_pose_curvature_threshold;
  const double minimum_shift_pull_out_distance = parameters_.minimum_shift_pull_out_distance;
  const int lateral_acceleration_sampling_num = parameters_.lateral_acceleration_sampling_num;

  // set minimum acc for breaking loop when sampling num is 1
  const double acc_resolution = std::max(
    std::abs(maximum_lateral_acc - minimum_lateral_acc) / lateral_acceleration_sampling_num,
    std::numeric_limits<double>::epsilon());

  // generate road lane reference path
  const auto arc_position_start = getArcCoordinates(road_lanes, start_pose);
  const double s_start = std::max(arc_position_start.length - backward_path_length, 0.0);
  const auto path_end_info =
    autoware::behavior_path_planner::utils::parking_departure::calcEndArcLength(
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
    const double pull_out_distance_converted = std::max(
      pull_out_distance, calcBeforeShiftedArcLength(
                           road_lane_reference_path_from_ego, pull_out_distance, shift_length));

    // Calculate the distance until the curvature at end_pose is below a certain threshold.
    // This is to prevent the path curvature from becoming unnecessarily large when end_pose is on a
    // curve.
    const double before_shifted_pull_out_distance = std::invoke([&]() -> double {
      double arc_length = 0.0;

      // If a curvature below end_pose_curvature_threshold is not found, return the distance to the
      // point with the smallest curvature after pull_out_distance_converted. pull_out_distance is a
      // variable to store that distance.
      double pull_out_distance = pull_out_distance_converted;
      double min_curvature_after_distance_converted = std::numeric_limits<double>::max();

      const auto curvatures_and_segment_lengths =
        autoware::motion_utils::calcCurvatureAndSegmentLength(
          road_lane_reference_path_from_ego.points);

      const auto update_arc_length = [&](size_t i, const auto & segment_length) {
        arc_length += (i == curvatures_and_segment_lengths.size() - 1)
                        ? segment_length.first + segment_length.second
                        : segment_length.first;
      };

      const auto update_min_curvature_and_pull_out_distance = [&](double curvature) {
        min_curvature_after_distance_converted = curvature;
        pull_out_distance = arc_length;
      };

      for (size_t i = 0; i < curvatures_and_segment_lengths.size(); ++i) {
        const auto & [signed_curvature, segment_length] = curvatures_and_segment_lengths[i];
        const double curvature = std::abs(signed_curvature);
        update_arc_length(i, segment_length);
        if (arc_length > pull_out_distance_converted) {
          // update distance with minimum curvature after pull_out_distance_converted
          if (curvature < min_curvature_after_distance_converted) {
            update_min_curvature_and_pull_out_distance(curvature);
          }
          // if curvature is smaller than end_pose_curvature_threshold, return the length
          if (curvature < end_pose_curvature_threshold) {
            return arc_length;
          }
        }
      }

      // if not found point with curvature below end_pose_curvature_threshold
      // pull_out_distance_converted, return the distance to the point with the smallest curvature
      // after pull_out_distance_converted
      return pull_out_distance;
    });

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
    // In the current path generation logic:
    // - Considering the maximum curvature of the path results in a smaller shift distance.
    // - Setting the allowable maximum lateral acceleration to a value smaller than the one
    // calculated by the constant lateral jerk trajectory generation.
    // - Setting the initial velocity to a very small value, such as 0.0.
    // These conditions cause the curvature around the shift start pose to become larger than
    // expected. To address this issue, an initial velocity 1.0 is provided.
    path_shifter.setVelocity(1.0);
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

  const auto curvatures_and_segment_lengths =
    autoware::motion_utils::calcCurvatureAndSegmentLength(path.points);
  for (size_t i = 0; i < curvatures_and_segment_lengths.size(); ++i) {
    const auto & [k, segment_length_pair] = curvatures_and_segment_lengths.at(i);

    // If it is the last point, add the lengths of the previous and next segments.
    // For other points, only add the length of the previous segment.
    const double segment_length = i == curvatures_and_segment_lengths.size() - 1
                                    ? segment_length_pair.first + segment_length_pair.second
                                    : segment_length_pair.first;

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

}  // namespace autoware::behavior_path_planner
