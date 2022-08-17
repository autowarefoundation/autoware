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

#include "behavior_path_planner/path_utilities.hpp"

#include "behavior_path_planner/utilities.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/resample/resample.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace behavior_path_planner::util
{
/**
 * @brief calc path arclength on each points from start point to end point.
 */
std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, size_t start, size_t end, double offset)
{
  std::vector<double> out;

  double sum = offset;
  out.push_back(sum);

  start = std::max(start + 1, size_t{1});
  end = std::min(end, path.points.size());

  for (size_t i = start; i < end; ++i) {
    sum +=
      tier4_autoware_utils::calcDistance2d(path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

/**
 * @brief calc path arclength from start point to end point.
 */
double calcPathArcLength(const PathWithLaneId & path, size_t start, size_t end)
{
  if (path.points.size() < 2) {
    return 0.0;
  }

  // swap
  bool is_negative_direction = false;
  if (start > end) {
    std::swap(start, end);
    is_negative_direction = true;
  }

  start = std::max(start, size_t{1});
  end = std::min(end, path.points.size());

  double sum = 0.0;
  for (size_t i = start; i < end; ++i) {
    sum +=
      tier4_autoware_utils::calcDistance2d(path.points.at(i).point, path.points.at(i - 1).point);
  }

  return is_negative_direction ? -sum : sum;
}

/**
 * @brief resamplePathWithSpline
 */
PathWithLaneId resamplePathWithSpline(const PathWithLaneId & path, double interval)
{
  if (path.points.size() < 2) {
    return path;
  }

  std::vector<geometry_msgs::msg::Pose> transformed_path(path.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    transformed_path.at(i) = path.points.at(i).point.pose;
  }

  constexpr double epsilon = 0.01;
  const auto has_almost_same_value = [&](const auto & vec, const auto x) {
    if (vec.empty()) return false;
    const auto has_close = [&](const auto v) { return std::abs(v - x) < epsilon; };
    return std::find_if(vec.begin(), vec.end(), has_close) != vec.end();
  };

  // Get lane ids that are not duplicated
  std::vector<double> s_in;
  std::vector<int64_t> unique_lane_ids;
  for (size_t i = 0; i < path.points.size(); ++i) {
    const double s = motion_utils::calcSignedArcLength(transformed_path, 0, i);
    for (const auto & lane_id : path.points.at(i).lane_ids) {
      if (
        std::find(unique_lane_ids.begin(), unique_lane_ids.end(), lane_id) !=
        unique_lane_ids.end()) {
        unique_lane_ids.push_back(lane_id);
        if (!has_almost_same_value(s_in, s)) {
          s_in.push_back(s);
        }
      }
    }
  }

  std::vector<double> s_out = s_in;

  const double path_len = motion_utils::calcArcLength(transformed_path);
  for (double s = 0.0; s < path_len; s += interval) {
    if (!has_almost_same_value(s_out, s)) {
      s_out.push_back(s);
    }
  }
  if (!has_almost_same_value(s_out, path_len)) {
    s_out.push_back(path_len);
  }

  std::sort(s_out.begin(), s_out.end());

  if (s_out.empty()) {
    return path;
  }

  return motion_utils::resamplePath(path, s_out);
}

Path toPath(const PathWithLaneId & input)
{
  Path output{};
  output.header = input.header;
  output.drivable_area = input.drivable_area;
  output.points.resize(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    output.points.at(i) = input.points.at(i).point;
  }
  return output;
}

size_t getIdxByArclength(const PathWithLaneId & path, const Pose & origin, const double signed_arc)
{
  if (path.points.empty()) {
    throw std::runtime_error("[getIdxByArclength] path points must be > 0");
  }

  const auto boost_closest_idx = motion_utils::findNearestIndex(
    path.points, origin, std::numeric_limits<double>::max(), M_PI / 4.0);

  // If the nearest index search with angle limit fails, search again without angle limit.
  size_t closest_idx = boost_closest_idx
                         ? *boost_closest_idx
                         : motion_utils::findNearestIndex(path.points, origin.position);

  using tier4_autoware_utils::calcDistance2d;
  double sum_length = 0.0;
  if (signed_arc >= 0.0) {
    for (size_t i = closest_idx; i < path.points.size() - 1; ++i) {
      const auto next_i = i + 1;
      sum_length += calcDistance2d(path.points.at(i), path.points.at(next_i));
      if (sum_length > signed_arc) {
        return next_i;
      }
    }
    return path.points.size() - 1;
  } else {
    for (size_t i = closest_idx; i > 0; --i) {
      const auto next_i = i - 1;
      sum_length -= calcDistance2d(path.points.at(i), path.points.at(next_i));
      if (sum_length < signed_arc) {
        return next_i;
      }
    }
    return 0;
  }
}

void clipPathLength(
  PathWithLaneId & path, const Pose base_pose, const double forward, const double backward)
{
  if (path.points.size() < 3) {
    return;
  }

  const auto start_idx = util::getIdxByArclength(path, base_pose, -backward);
  const auto end_idx = util::getIdxByArclength(path, base_pose, forward);

  const std::vector<PathPointWithLaneId> clipped_points{
    path.points.begin() + start_idx, path.points.begin() + end_idx + 1};

  path.points = clipped_points;
}

std::pair<TurnIndicatorsCommand, double> getPathTurnSignal(
  const lanelet::ConstLanelets & current_lanes, const ShiftedPath & path,
  const ShiftPoint & shift_point, const Pose & pose, const double & velocity,
  const BehaviorPathPlannerParameters & common_parameter)
{
  TurnIndicatorsCommand turn_signal;
  turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
  const double max_time = std::numeric_limits<double>::max();
  const double max_distance = std::numeric_limits<double>::max();
  if (path.shift_length.empty()) {
    return std::make_pair(turn_signal, max_distance);
  }
  const auto base_link2front = common_parameter.base_link2front;
  const auto vehicle_width = common_parameter.vehicle_width;
  const auto shift_to_outside = vehicle_width / 2;
  const auto tl_on_threshold_lat = common_parameter.turn_light_on_threshold_dis_lat;
  const auto tl_on_threshold_long = common_parameter.turn_light_on_threshold_dis_long;
  const auto prev_sec = common_parameter.turn_light_on_threshold_time;
  const double epsilon = 1e-6;
  const auto arc_position_current_pose = lanelet::utils::getArcCoordinates(current_lanes, pose);

  // Start turn signal when 1 or 2 is satisfied
  //  1. time to shift start point is less than prev_sec
  //  2. distance to shift point is shorter than tl_on_threshold_long

  // Turn signal on when conditions below are satisfied
  //  1. lateral offset is larger than tl_on_threshold_lat for left signal
  //                      smaller than tl_on_threshold_lat for right signal
  //  2. side point at shift start/end point cross the line
  double distance_to_shift_start;
  {
    const auto arc_position_shift_start =
      lanelet::utils::getArcCoordinates(current_lanes, shift_point.start);
    distance_to_shift_start = arc_position_shift_start.length - arc_position_current_pose.length;
  }

  const auto time_to_shift_start =
    (std::abs(velocity) < epsilon) ? max_time : distance_to_shift_start / velocity;

  const double diff =
    path.shift_length.at(shift_point.end_idx) - path.shift_length.at(shift_point.start_idx);

  Pose shift_start_point = path.path.points.at(shift_point.start_idx).point.pose;
  Pose shift_end_point = path.path.points.at(shift_point.end_idx).point.pose;
  Pose left_start_point = shift_start_point;
  Pose right_start_point = shift_start_point;
  Pose left_end_point = shift_end_point;
  Pose right_end_point = shift_end_point;
  {
    const double start_yaw = tf2::getYaw(shift_point.start.orientation);
    const double end_yaw = tf2::getYaw(shift_point.end.orientation);
    left_start_point.position.x -= std::sin(start_yaw) * (shift_to_outside);
    left_start_point.position.y += std::cos(start_yaw) * (shift_to_outside);
    right_start_point.position.x -= std::sin(start_yaw) * (-shift_to_outside);
    right_start_point.position.y += std::cos(start_yaw) * (-shift_to_outside);
    left_end_point.position.x -= std::sin(end_yaw) * (shift_to_outside);
    left_end_point.position.y += std::cos(end_yaw) * (shift_to_outside);
    right_end_point.position.x -= std::sin(end_yaw) * (-shift_to_outside);
    right_end_point.position.y += std::cos(end_yaw) * (-shift_to_outside);
  }

  bool left_start_point_is_in_lane = false;
  bool right_start_point_is_in_lane = false;
  bool left_end_point_is_in_lane = false;
  bool right_end_point_is_in_lane = false;
  {
    for (const auto & llt : current_lanes) {
      if (lanelet::utils::isInLanelet(left_start_point, llt, 0.1)) {
        left_start_point_is_in_lane = true;
      }
      if (lanelet::utils::isInLanelet(right_start_point, llt, 0.1)) {
        right_start_point_is_in_lane = true;
      }
      if (lanelet::utils::isInLanelet(left_end_point, llt, 0.1)) {
        left_end_point_is_in_lane = true;
      }
      if (lanelet::utils::isInLanelet(right_end_point, llt, 0.1)) {
        right_end_point_is_in_lane = true;
      }
    }
  }

  bool cross_line = false;
  bool TEMPORARY_SET_CROSS_LINE_TRUE =
    true;  // due to a bug. See link:
           // https://github.com/autowarefoundation/autoware.universe/pull/748
  if (TEMPORARY_SET_CROSS_LINE_TRUE) {
    cross_line = true;
  } else {
    cross_line =
      (left_start_point_is_in_lane != left_end_point_is_in_lane ||
       right_start_point_is_in_lane != right_end_point_is_in_lane);
  }

  if (time_to_shift_start < prev_sec || distance_to_shift_start < tl_on_threshold_long) {
    if (diff > tl_on_threshold_lat && cross_line) {
      turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
    } else if (diff < -tl_on_threshold_lat && cross_line) {
      turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    }
  }

  // calc distance from ego vehicle front to shift end point.
  double distance_from_vehicle_front;
  {
    const auto arc_position_shift_end =
      lanelet::utils::getArcCoordinates(current_lanes, shift_point.end);
    distance_from_vehicle_front =
      arc_position_shift_end.length - arc_position_current_pose.length - base_link2front;
  }

  if (distance_from_vehicle_front >= 0.0) {
    return std::make_pair(turn_signal, distance_from_vehicle_front);
  }

  return std::make_pair(turn_signal, max_distance);
}

}  // namespace behavior_path_planner::util
