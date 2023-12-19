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

#include "behavior_path_lane_change_module/utils/utils.hpp"

#include "behavior_path_lane_change_module/utils/data_structs.hpp"
#include "behavior_path_lane_change_module/utils/path.hpp"
#include "behavior_path_planner_common/marker_utils/utils.hpp"
#include "behavior_path_planner_common/parameters.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/interpolation.hpp>
#include <motion_utils/trajectory/path_with_lane_id.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace behavior_path_planner::utils::lane_change
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using route_handler::RouteHandler;
using tier4_autoware_utils::LineString2d;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using lanelet::ArcCoordinates;

double calcLaneChangeResampleInterval(
  const double lane_changing_length, const double lane_changing_velocity)
{
  constexpr auto min_resampling_points{30.0};
  constexpr auto resampling_dt{0.2};
  return std::max(
    lane_changing_length / min_resampling_points, lane_changing_velocity * resampling_dt);
}

double calcMinimumLaneChangeLength(
  const LaneChangeParameters & lane_change_parameters, const std::vector<double> & shift_intervals,
  const double length_to_intersection)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const double & vel = lane_change_parameters.minimum_lane_changing_velocity;
  const auto lat_acc = lane_change_parameters.lane_change_lat_acc_map.find(vel);
  const double & max_lateral_acc = lat_acc.second;
  const double & lateral_jerk = lane_change_parameters.lane_changing_lateral_jerk;
  const double & finish_judge_buffer = lane_change_parameters.lane_change_finish_judge_buffer;
  const double & backward_buffer = lane_change_parameters.backward_length_buffer_for_end_of_lane;

  double accumulated_length = length_to_intersection;
  for (const auto & shift_interval : shift_intervals) {
    const double t =
      PathShifter::calcShiftTimeFromJerk(shift_interval, lateral_jerk, max_lateral_acc);
    accumulated_length += vel * t + finish_judge_buffer;
  }
  accumulated_length += backward_buffer * (shift_intervals.size() - 1.0);

  return accumulated_length;
}

double calcMaximumLaneChangeLength(
  const double current_velocity, const LaneChangeParameters & lane_change_parameters,
  const std::vector<double> & shift_intervals, const double max_acc)
{
  if (shift_intervals.empty()) {
    return 0.0;
  }

  const double & finish_judge_buffer = lane_change_parameters.lane_change_finish_judge_buffer;
  const double & lateral_jerk = lane_change_parameters.lane_changing_lateral_jerk;
  const double & t_prepare = lane_change_parameters.lane_change_prepare_duration;

  double vel = current_velocity;
  double accumulated_length = 0.0;
  for (const auto & shift_interval : shift_intervals) {
    // prepare section
    const double prepare_length = vel * t_prepare + 0.5 * max_acc * t_prepare * t_prepare;
    vel = vel + max_acc * t_prepare;

    // lane changing section
    const auto lat_acc = lane_change_parameters.lane_change_lat_acc_map.find(vel);
    const double t_lane_changing =
      PathShifter::calcShiftTimeFromJerk(shift_interval, lateral_jerk, lat_acc.second);
    const double lane_changing_length =
      vel * t_lane_changing + 0.5 * max_acc * t_lane_changing * t_lane_changing;

    accumulated_length += prepare_length + lane_changing_length + finish_judge_buffer;
    vel = vel + max_acc * t_lane_changing;
  }
  accumulated_length +=
    lane_change_parameters.backward_length_buffer_for_end_of_lane * (shift_intervals.size() - 1.0);

  return accumulated_length;
}

double calcMinimumAcceleration(
  const double current_velocity, const double min_longitudinal_acc,
  const LaneChangeParameters & lane_change_parameters)
{
  const double min_lane_changing_velocity = lane_change_parameters.minimum_lane_changing_velocity;
  const double prepare_duration = lane_change_parameters.lane_change_prepare_duration;
  const double acc = (min_lane_changing_velocity - current_velocity) / prepare_duration;
  return std::clamp(acc, -std::abs(min_longitudinal_acc), -std::numeric_limits<double>::epsilon());
}

double calcMaximumAcceleration(
  const double current_velocity, const double current_max_velocity,
  const double max_longitudinal_acc, const LaneChangeParameters & lane_change_parameters)
{
  const double prepare_duration = lane_change_parameters.lane_change_prepare_duration;
  const double acc = (current_max_velocity - current_velocity) / prepare_duration;
  return std::clamp(acc, 0.0, max_longitudinal_acc);
}

double calcLaneChangingAcceleration(
  const double initial_lane_changing_velocity, const double max_path_velocity,
  const double lane_changing_time, const double prepare_longitudinal_acc)
{
  if (prepare_longitudinal_acc <= 0.0) {
    return 0.0;
  }

  return std::clamp(
    (max_path_velocity - initial_lane_changing_velocity) / lane_changing_time, 0.0,
    prepare_longitudinal_acc);
}

void setPrepareVelocity(
  PathWithLaneId & prepare_segment, const double current_velocity, const double prepare_velocity)
{
  if (current_velocity < prepare_velocity) {
    // acceleration
    for (size_t i = 0; i < prepare_segment.points.size(); ++i) {
      prepare_segment.points.at(i).point.longitudinal_velocity_mps = std::min(
        prepare_segment.points.at(i).point.longitudinal_velocity_mps,
        static_cast<float>(prepare_velocity));
    }
  } else {
    // deceleration
    prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
      prepare_segment.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(prepare_velocity));
  }
}

std::vector<double> getAccelerationValues(
  const double min_acc, const double max_acc, const size_t sampling_num)
{
  if (min_acc > max_acc) {
    return {};
  }

  if (max_acc - min_acc < std::numeric_limits<double>::epsilon()) {
    return {0.0};
  }

  constexpr double epsilon = 0.001;
  const auto resolution = std::abs(max_acc - min_acc) / sampling_num;

  std::vector<double> sampled_values{min_acc};
  for (double sampled_acc = min_acc + resolution;
       sampled_acc < max_acc + std::numeric_limits<double>::epsilon(); sampled_acc += resolution) {
    // check whether if we need to add 0.0
    if (sampled_values.back() < -epsilon && sampled_acc > epsilon) {
      sampled_values.push_back(0.0);
    }

    sampled_values.push_back(sampled_acc);
  }
  std::reverse(sampled_values.begin(), sampled_values.end());

  return sampled_values;
}

lanelet::ConstLanelets getTargetPreferredLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Direction & direction,
  const LaneChangeModuleType & type)
{
  if (type != LaneChangeModuleType::NORMAL) {
    return target_lanes;
  }

  const auto target_lane =
    utils::lane_change::getLaneChangeTargetLane(route_handler, current_lanes, type, direction);
  if (!target_lane) {
    return target_lanes;
  }

  const auto itr = std::find_if(
    target_lanes.begin(), target_lanes.end(),
    [&](const lanelet::ConstLanelet & lane) { return lane.id() == target_lane->id(); });

  if (itr == target_lanes.end()) {
    return target_lanes;
  }

  const int target_id = std::distance(target_lanes.begin(), itr);
  const lanelet::ConstLanelets target_preferred_lanes(
    target_lanes.begin() + target_id, target_lanes.end());
  return target_preferred_lanes;
}

lanelet::ConstLanelets getTargetNeighborLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType & type)
{
  lanelet::ConstLanelets neighbor_lanes;

  for (const auto & current_lane : current_lanes) {
    if (route_handler.getNumLaneToPreferredLane(current_lane) != 0) {
      if (
        type == LaneChangeModuleType::NORMAL ||
        type == LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE) {
        neighbor_lanes.push_back(current_lane);
      }
    } else {
      if (type != LaneChangeModuleType::NORMAL) {
        neighbor_lanes.push_back(current_lane);
      }
    }
  }

  return neighbor_lanes;
}

lanelet::BasicPolygon2d getTargetNeighborLanesPolygon(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType & type)
{
  const auto target_neighbor_lanelets =
    utils::lane_change::getTargetNeighborLanes(route_handler, current_lanes, type);
  if (target_neighbor_lanelets.empty()) {
    return {};
  }
  const auto target_neighbor_preferred_lane_poly = lanelet::utils::getPolygonFromArcLength(
    target_neighbor_lanelets, 0, std::numeric_limits<double>::max());

  return lanelet::utils::to2D(target_neighbor_preferred_lane_poly).basicPolygon();
}

bool isPathInLanelets(
  const PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes)
{
  const auto current_lane_poly =
    lanelet::utils::getPolygonFromArcLength(current_lanes, 0, std::numeric_limits<double>::max());
  const auto target_lane_poly =
    lanelet::utils::getPolygonFromArcLength(target_lanes, 0, std::numeric_limits<double>::max());
  const auto current_lane_poly_2d = lanelet::utils::to2D(current_lane_poly).basicPolygon();
  const auto target_lane_poly_2d = lanelet::utils::to2D(target_lane_poly).basicPolygon();
  for (const auto & pt : path.points) {
    const lanelet::BasicPoint2d ll_pt(pt.point.pose.position.x, pt.point.pose.position.y);
    const auto is_in_current = boost::geometry::covered_by(ll_pt, current_lane_poly_2d);
    if (is_in_current) {
      continue;
    }
    const auto is_in_target = boost::geometry::covered_by(ll_pt, target_lane_poly_2d);
    if (!is_in_target) {
      return false;
    }
  }
  return true;
}

std::optional<LaneChangePath> constructCandidatePath(
  const LaneChangeInfo & lane_change_info, const PathWithLaneId & prepare_segment,
  const PathWithLaneId & target_segment, const PathWithLaneId & target_lane_reference_path,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids)
{
  const auto & shift_line = lane_change_info.shift_line;
  const auto & original_lanes = lane_change_info.current_lanes;
  const auto & target_lanes = lane_change_info.target_lanes;
  const auto terminal_lane_changing_velocity = lane_change_info.terminal_lane_changing_velocity;
  const auto longitudinal_acceleration = lane_change_info.longitudinal_acceleration;
  const auto lane_change_velocity = lane_change_info.velocity;
  const auto lane_change_length = lane_change_info.length;

  PathShifter path_shifter;
  path_shifter.setPath(target_lane_reference_path);
  path_shifter.addShiftLine(shift_line);
  path_shifter.setLongitudinalAcceleration(longitudinal_acceleration.lane_changing);
  ShiftedPath shifted_path;

  // offset front side
  bool offset_back = false;

  const auto initial_lane_changing_velocity = lane_change_velocity.lane_changing;
  path_shifter.setVelocity(initial_lane_changing_velocity);
  path_shifter.setLateralAccelerationLimit(std::abs(lane_change_info.lateral_acceleration));

  if (!path_shifter.generate(&shifted_path, offset_back)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
      "failed to generate shifted path.");
  }

  // TODO(Zulfaqar Azmi): have to think of a more feasible solution for points being remove by path
  // shifter.
  if (shifted_path.path.points.size() < shift_line.end_idx + 1) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("behavior_path_planner").get_child("utils").get_child(__func__),
      "path points are removed by PathShifter.");
    return std::nullopt;
  }

  const auto & prepare_length = lane_change_length.prepare;
  const auto & lane_changing_length = lane_change_length.lane_changing;

  LaneChangePath candidate_path;
  candidate_path.info = lane_change_info;

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("constructCandidatePath"),
    "prepare_length: %f, lane_change: %f", prepare_length, lane_changing_length);

  const auto lane_change_end_idx =
    motion_utils::findNearestIndex(shifted_path.path.points, candidate_path.info.lane_changing_end);

  if (!lane_change_end_idx) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
      "lane change end idx not found on target path.");
    return std::nullopt;
  }

  for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
    auto & point = shifted_path.path.points.at(i);
    if (i < *lane_change_end_idx) {
      point.lane_ids = replaceWithSortedIds(point.lane_ids, sorted_lane_ids);
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps, static_cast<float>(terminal_lane_changing_velocity));
      continue;
    }
    const auto nearest_idx =
      motion_utils::findNearestIndex(target_segment.points, point.point.pose);
    point.lane_ids = target_segment.points.at(*nearest_idx).lane_ids;
  }

  // TODO(Yutaka Shimizu): remove this flag after make the isPathInLanelets faster
  const bool enable_path_check_in_lanelet = false;

  // check candidate path is in lanelet
  if (
    enable_path_check_in_lanelet &&
    !isPathInLanelets(shifted_path.path, original_lanes, target_lanes)) {
    return std::nullopt;
  }

  candidate_path.path = utils::combinePath(prepare_segment, shifted_path.path);
  candidate_path.shifted_path = shifted_path;

  return std::optional<LaneChangePath>{candidate_path};
}

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & lane_changing_start_pose, const double target_lane_length,
  const double lane_changing_length, const double forward_path_length,
  const double resample_interval, const bool is_goal_in_route, const double next_lane_change_buffer)
{
  const ArcCoordinates lane_change_start_arc_position =
    lanelet::utils::getArcCoordinates(target_lanes, lane_changing_start_pose);

  const double s_start = lane_change_start_arc_position.length;
  const double s_end = std::invoke([&]() {
    const auto dist_from_lc_start = s_start + lane_changing_length + forward_path_length;
    if (is_goal_in_route) {
      const double s_goal =
        lanelet::utils::getArcCoordinates(target_lanes, route_handler.getGoalPose()).length -
        next_lane_change_buffer;
      return std::min(dist_from_lc_start, s_goal);
    }
    return std::min(dist_from_lc_start, target_lane_length - next_lane_change_buffer);
  });

  if (s_end - s_start < lane_changing_length) {
    return PathWithLaneId();
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("getReferencePathFromTargetLane"),
    "start: %f, end: %f", s_start, s_end);

  const auto lane_changing_reference_path =
    route_handler.getCenterLinePath(target_lanes, s_start, s_end);

  return utils::resamplePathWithSpline(
    lane_changing_reference_path, resample_interval, true, {0.0, lane_changing_length});
}

ShiftLine getLaneChangingShiftLine(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_segment,
  const PathWithLaneId & reference_path, const double shift_length)
{
  const Pose & lane_changing_start_pose = prepare_segment.points.back().point.pose;
  const Pose & lane_changing_end_pose = target_segment.points.front().point.pose;

  ShiftLine shift_line;
  shift_line.end_shift_length = shift_length;
  shift_line.start = lane_changing_start_pose;
  shift_line.end = lane_changing_end_pose;
  shift_line.start_idx =
    motion_utils::findNearestIndex(reference_path.points, lane_changing_start_pose.position);
  shift_line.end_idx =
    motion_utils::findNearestIndex(reference_path.points, lane_changing_end_pose.position);

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("getLaneChangingShiftLine"),
    "shift_line distance: %f", shift_length);
  return shift_line;
}

std::vector<DrivableLanes> generateDrivableLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & lane_change_lanes)
{
  size_t current_lc_idx = 0;
  std::vector<DrivableLanes> drivable_lanes(current_lanes.size());
  for (size_t i = 0; i < current_lanes.size(); ++i) {
    const auto & current_lane = current_lanes.at(i);
    drivable_lanes.at(i).left_lane = current_lane;
    drivable_lanes.at(i).right_lane = current_lane;

    const auto left_lane = route_handler.getLeftLanelet(current_lane, false, false);
    const auto right_lane = route_handler.getRightLanelet(current_lane, false, false);
    if (!left_lane && !right_lane) {
      continue;
    }

    for (size_t lc_idx = current_lc_idx; lc_idx < lane_change_lanes.size(); ++lc_idx) {
      const auto & lc_lane = lane_change_lanes.at(lc_idx);
      if (left_lane && lc_lane.id() == left_lane->id()) {
        drivable_lanes.at(i).left_lane = lc_lane;
        current_lc_idx = lc_idx;
        break;
      }

      if (right_lane && lc_lane.id() == right_lane->id()) {
        drivable_lanes.at(i).right_lane = lc_lane;
        current_lc_idx = lc_idx;
        break;
      }
    }
  }

  for (size_t i = current_lc_idx + 1; i < lane_change_lanes.size(); ++i) {
    const auto & lc_lane = lane_change_lanes.at(i);
    DrivableLanes drivable_lane;
    drivable_lane.left_lane = lc_lane;
    drivable_lane.right_lane = lc_lane;
    drivable_lanes.push_back(drivable_lane);
  }

  return drivable_lanes;
}

std::vector<DrivableLanes> generateDrivableLanes(
  const std::vector<DrivableLanes> original_drivable_lanes, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & lane_change_lanes)
{
  const auto has_same_lane =
    [](const lanelet::ConstLanelets lanes, const lanelet::ConstLanelet & lane) {
      if (lanes.empty()) return false;
      const auto has_same = [&](const auto & ll) { return ll.id() == lane.id(); };
      return std::find_if(lanes.begin(), lanes.end(), has_same) != lanes.end();
    };

  const auto check_middle = [&](const auto & lane) -> std::optional<DrivableLanes> {
    for (const auto & drivable_lane : original_drivable_lanes) {
      if (has_same_lane(drivable_lane.middle_lanes, lane)) {
        return drivable_lane;
      }
    }
    return std::nullopt;
  };

  const auto check_left = [&](const auto & lane) -> std::optional<DrivableLanes> {
    for (const auto & drivable_lane : original_drivable_lanes) {
      if (drivable_lane.left_lane.id() == lane.id()) {
        return drivable_lane;
      }
    }
    return std::nullopt;
  };

  const auto check_right = [&](const auto & lane) -> std::optional<DrivableLanes> {
    for (const auto & drivable_lane : original_drivable_lanes) {
      if (drivable_lane.right_lane.id() == lane.id()) {
        return drivable_lane;
      }
    }
    return std::nullopt;
  };

  size_t current_lc_idx = 0;
  std::vector<DrivableLanes> drivable_lanes(current_lanes.size());
  for (size_t i = 0; i < current_lanes.size(); ++i) {
    const auto & current_lane = current_lanes.at(i);

    const auto middle_drivable_lane = check_middle(current_lane);
    if (middle_drivable_lane) {
      drivable_lanes.at(i) = *middle_drivable_lane;
    }

    const auto left_drivable_lane = check_left(current_lane);
    if (left_drivable_lane) {
      drivable_lanes.at(i) = *left_drivable_lane;
    }

    const auto right_drivable_lane = check_right(current_lane);
    if (right_drivable_lane) {
      drivable_lanes.at(i) = *right_drivable_lane;
    }

    if (!middle_drivable_lane && !left_drivable_lane && !right_drivable_lane) {
      drivable_lanes.at(i).left_lane = current_lane;
      drivable_lanes.at(i).right_lane = current_lane;
    }

    const auto left_lane = route_handler.getLeftLanelet(current_lane);
    const auto right_lane = route_handler.getRightLanelet(current_lane);
    if (!left_lane && !right_lane) {
      continue;
    }

    for (size_t lc_idx = current_lc_idx; lc_idx < lane_change_lanes.size(); ++lc_idx) {
      const auto & lc_lane = lane_change_lanes.at(lc_idx);
      if (left_lane && lc_lane.id() == left_lane->id()) {
        if (left_drivable_lane) {
          drivable_lanes.at(i).left_lane = lc_lane;
        }
        current_lc_idx = lc_idx;
        break;
      }

      if (right_lane && lc_lane.id() == right_lane->id()) {
        if (right_drivable_lane) {
          drivable_lanes.at(i).right_lane = lc_lane;
        }
        current_lc_idx = lc_idx;
        break;
      }
    }
  }

  for (size_t i = current_lc_idx + 1; i < lane_change_lanes.size(); ++i) {
    const auto & lc_lane = lane_change_lanes.at(i);
    DrivableLanes drivable_lane;

    const auto middle_drivable_lane = check_middle(lc_lane);
    if (middle_drivable_lane) {
      drivable_lane = *middle_drivable_lane;
    }

    const auto left_drivable_lane = check_left(lc_lane);
    if (left_drivable_lane) {
      drivable_lane = *left_drivable_lane;
    }

    const auto right_drivable_lane = check_right(lc_lane);
    if (right_drivable_lane) {
      drivable_lane = *right_drivable_lane;
    }

    if (!middle_drivable_lane && !left_drivable_lane && !right_drivable_lane) {
      drivable_lane.left_lane = lc_lane;
      drivable_lane.right_lane = lc_lane;
    }

    drivable_lanes.push_back(drivable_lane);
  }

  return drivable_lanes;
}

double getLateralShift(const LaneChangePath & path)
{
  const auto start_idx = path.info.shift_line.start_idx;
  const auto end_idx = path.info.shift_line.end_idx;

  return path.shifted_path.shift_length.at(end_idx) - path.shifted_path.shift_length.at(start_idx);
}

bool hasEnoughLengthToLaneChangeAfterAbort(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const Pose & current_pose, const double abort_return_dist,
  const LaneChangeParameters & lane_change_parameters, const Direction direction)
{
  const auto shift_intervals =
    route_handler.getLateralIntervalsToPreferredLane(current_lanes.back(), direction);
  const double minimum_lane_change_length =
    calcMinimumLaneChangeLength(lane_change_parameters, shift_intervals);
  const auto abort_plus_lane_change_length = abort_return_dist + minimum_lane_change_length;
  if (abort_plus_lane_change_length > utils::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  if (
    route_handler.isInGoalRouteSection(current_lanes.back()) &&
    abort_plus_lane_change_length >
      utils::getSignedDistance(current_pose, route_handler.getGoalPose(), current_lanes)) {
    return false;
  }

  return true;
}

// TODO(Azu): In the future, get back lanelet within `to_back_dist` [m] from queried lane
lanelet::ConstLanelets getBackwardLanelets(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & current_pose, const double backward_length)
{
  if (target_lanes.empty()) {
    return {};
  }

  const auto arc_length = lanelet::utils::getArcCoordinates(target_lanes, current_pose);

  if (arc_length.length >= backward_length) {
    return {};
  }

  const auto & front_lane = target_lanes.front();
  const auto preceding_lanes = route_handler.getPrecedingLaneletSequence(
    front_lane, std::abs(backward_length - arc_length.length), {front_lane});

  lanelet::ConstLanelets backward_lanes{};
  const auto num_of_lanes = std::invoke([&preceding_lanes]() {
    size_t sum{0};
    for (const auto & lanes : preceding_lanes) {
      sum += lanes.size();
    }
    return sum;
  });

  backward_lanes.reserve(num_of_lanes);

  for (const auto & lanes : preceding_lanes) {
    backward_lanes.insert(backward_lanes.end(), lanes.begin(), lanes.end());
  }

  return backward_lanes;
}

double calcLateralBufferForFiltering(const double vehicle_width, const double lateral_buffer)
{
  return lateral_buffer + 0.5 * vehicle_width;
}

std::string getStrDirection(const std::string & name, const Direction direction)
{
  if (direction == Direction::LEFT) {
    return name + "_left";
  }
  if (direction == Direction::RIGHT) {
    return name + "_right";
  }
  return "";
}

std::vector<std::vector<int64_t>> getSortedLaneIds(
  const RouteHandler & route_handler, const Pose & current_pose,
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes)
{
  const auto rough_shift_length =
    lanelet::utils::getArcCoordinates(target_lanes, current_pose).distance;

  std::vector<std::vector<int64_t>> sorted_lane_ids{};
  sorted_lane_ids.reserve(target_lanes.size());
  const auto get_sorted_lane_ids = [&](const lanelet::ConstLanelet & target_lane) {
    const auto routing_graph_ptr = route_handler.getRoutingGraphPtr();
    lanelet::ConstLanelet lane;
    if (rough_shift_length < 0.0) {
      // lane change to the left, so i wan to take the lane right to target
      const auto has_target_right = routing_graph_ptr->right(target_lane);
      if (has_target_right) {
        lane = *has_target_right;
      }
    } else if (rough_shift_length > 0.0) {
      const auto has_target_left = routing_graph_ptr->left(target_lane);
      if (has_target_left) {
        lane = *has_target_left;
      }
    } else {
      lane = target_lane;
    }

    const auto find_same_id = std::find_if(
      current_lanes.cbegin(), current_lanes.cend(),
      [&lane](const lanelet::ConstLanelet & orig) { return orig.id() == lane.id(); });

    if (find_same_id == current_lanes.cend()) {
      return std::vector{target_lane.id()};
    }

    if (target_lane.id() > find_same_id->id()) {
      return std::vector{find_same_id->id(), target_lane.id()};
    }

    return std::vector{target_lane.id(), find_same_id->id()};
  };

  std::transform(
    target_lanes.cbegin(), target_lanes.cend(), std::back_inserter(sorted_lane_ids),
    get_sorted_lane_ids);

  return sorted_lane_ids;
}

std::vector<int64_t> replaceWithSortedIds(
  const std::vector<int64_t> & original_lane_ids,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids)
{
  for (const auto original_id : original_lane_ids) {
    for (const auto & sorted_id : sorted_lane_ids) {
      if (std::find(sorted_id.cbegin(), sorted_id.cend(), original_id) != sorted_id.cend()) {
        return sorted_id;
      }
    }
  }
  return original_lane_ids;
}

CandidateOutput assignToCandidate(
  const LaneChangePath & lane_change_path, const Point & ego_position)
{
  CandidateOutput candidate_output;
  candidate_output.path_candidate = lane_change_path.path;
  candidate_output.lateral_shift = utils::lane_change::getLateralShift(lane_change_path);
  candidate_output.start_distance_to_path_change = motion_utils::calcSignedArcLength(
    lane_change_path.path.points, ego_position, lane_change_path.info.shift_line.start.position);
  candidate_output.finish_distance_to_path_change = motion_utils::calcSignedArcLength(
    lane_change_path.path.points, ego_position, lane_change_path.info.shift_line.end.position);

  return candidate_output;
}

std::optional<lanelet::ConstLanelet> getLaneChangeTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType type, const Direction & direction)
{
  if (
    type == LaneChangeModuleType::NORMAL ||
    type == LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE) {
    return route_handler.getLaneChangeTarget(current_lanes, direction);
  }

  return route_handler.getLaneChangeTargetExceptPreferredLane(current_lanes, direction);
}

std::vector<PoseWithVelocityStamped> convertToPredictedPath(
  const LaneChangePath & lane_change_path, const Twist & vehicle_twist, const Pose & vehicle_pose,
  const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const double resolution)
{
  if (lane_change_path.path.points.empty()) {
    return {};
  }

  const auto & path = lane_change_path.path;
  const auto prepare_acc = lane_change_path.info.longitudinal_acceleration.prepare;
  const auto lane_changing_acc = lane_change_path.info.longitudinal_acceleration.lane_changing;
  const auto duration = lane_change_path.info.duration.sum();
  const auto prepare_time = lane_change_path.info.duration.prepare;
  const auto & minimum_lane_changing_velocity =
    lane_change_parameters.minimum_lane_changing_velocity;

  const auto nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, vehicle_pose, common_parameters.ego_nearest_dist_threshold,
    common_parameters.ego_nearest_yaw_threshold);

  std::vector<PoseWithVelocityStamped> predicted_path;
  const auto vehicle_pose_frenet =
    convertToFrenetPoint(path.points, vehicle_pose.position, nearest_seg_idx);
  const double initial_velocity = std::abs(vehicle_twist.linear.x);

  // prepare segment
  for (double t = 0.0; t < prepare_time; t += resolution) {
    const double velocity =
      std::max(initial_velocity + prepare_acc * t, minimum_lane_changing_velocity);
    const double length = initial_velocity * t + 0.5 * prepare_acc * t * t;
    const auto pose =
      motion_utils::calcInterpolatedPose(path.points, vehicle_pose_frenet.length + length);
    predicted_path.emplace_back(t, pose, velocity);
  }

  // lane changing segment
  const double lane_changing_velocity =
    std::max(initial_velocity + prepare_acc * prepare_time, minimum_lane_changing_velocity);
  const double offset =
    initial_velocity * prepare_time + 0.5 * prepare_acc * prepare_time * prepare_time;
  for (double t = prepare_time; t < duration; t += resolution) {
    const double delta_t = t - prepare_time;
    const double velocity = lane_changing_velocity + lane_changing_acc * delta_t;
    const double length =
      lane_changing_velocity * delta_t + 0.5 * lane_changing_acc * delta_t * delta_t + offset;
    const auto pose =
      motion_utils::calcInterpolatedPose(path.points, vehicle_pose_frenet.length + length);
    predicted_path.emplace_back(t, pose, velocity);
  }

  return predicted_path;
}

bool isParkedObject(
  const PathWithLaneId & path, const RouteHandler & route_handler,
  const ExtendedPredictedObject & object, const double object_check_min_road_shoulder_width,
  const double object_shiftable_ratio_threshold, const double static_object_velocity_threshold)
{
  // ============================================ <- most_left_lanelet.leftBound()
  // y              road shoulder
  // ^ ------------------------------------------
  // |   x                                +
  // +---> --- object closest lanelet --- o ----- <- object_closest_lanelet.centerline()
  //
  // --------------------------------------------
  // +: object position
  // o: nearest point on centerline

  using lanelet::geometry::distance2d;
  using lanelet::geometry::toArcCoordinates;

  const double object_vel_norm =
    std::hypot(object.initial_twist.twist.linear.x, object.initial_twist.twist.linear.y);
  if (object_vel_norm > static_object_velocity_threshold) {
    return false;
  }

  const auto & object_pose = object.initial_pose.pose;
  const auto object_closest_index =
    motion_utils::findNearestIndex(path.points, object_pose.position);
  const auto object_closest_pose = path.points.at(object_closest_index).point.pose;

  lanelet::ConstLanelet closest_lanelet;
  if (!route_handler.getClosestLaneletWithinRoute(object_closest_pose, &closest_lanelet)) {
    return false;
  }

  const double lat_dist = motion_utils::calcLateralOffset(path.points, object_pose.position);
  lanelet::BasicLineString2d bound;
  double center_to_bound_buffer = 0.0;
  if (lat_dist > 0.0) {
    // left side vehicle
    const auto most_left_road_lanelet = route_handler.getMostLeftLanelet(closest_lanelet);
    const auto most_left_lanelet_candidates =
      route_handler.getLaneletMapPtr()->laneletLayer.findUsages(most_left_road_lanelet.leftBound());
    lanelet::ConstLanelet most_left_lanelet = most_left_road_lanelet;
    const lanelet::Attribute sub_type =
      most_left_lanelet.attribute(lanelet::AttributeName::Subtype);

    for (const auto & ll : most_left_lanelet_candidates) {
      const lanelet::Attribute sub_type = ll.attribute(lanelet::AttributeName::Subtype);
      if (sub_type.value() == "road_shoulder") {
        most_left_lanelet = ll;
      }
    }
    bound = most_left_lanelet.leftBound2d().basicLineString();
    if (sub_type.value() != "road_shoulder") {
      center_to_bound_buffer = object_check_min_road_shoulder_width;
    }
  } else {
    // right side vehicle
    const auto most_right_road_lanelet = route_handler.getMostRightLanelet(closest_lanelet);
    const auto most_right_lanelet_candidates =
      route_handler.getLaneletMapPtr()->laneletLayer.findUsages(
        most_right_road_lanelet.rightBound());

    lanelet::ConstLanelet most_right_lanelet = most_right_road_lanelet;
    const lanelet::Attribute sub_type =
      most_right_lanelet.attribute(lanelet::AttributeName::Subtype);

    for (const auto & ll : most_right_lanelet_candidates) {
      const lanelet::Attribute sub_type = ll.attribute(lanelet::AttributeName::Subtype);
      if (sub_type.value() == "road_shoulder") {
        most_right_lanelet = ll;
      }
    }
    bound = most_right_lanelet.rightBound2d().basicLineString();
    if (sub_type.value() != "road_shoulder") {
      center_to_bound_buffer = object_check_min_road_shoulder_width;
    }
  }

  return isParkedObject(
    closest_lanelet, bound, object, center_to_bound_buffer, object_shiftable_ratio_threshold);
}

bool isParkedObject(
  const lanelet::ConstLanelet & closest_lanelet, const lanelet::BasicLineString2d & boundary,
  const ExtendedPredictedObject & object, const double buffer_to_bound,
  const double ratio_threshold)
{
  using lanelet::geometry::distance2d;

  const auto & obj_pose = object.initial_pose.pose;
  const auto & obj_shape = object.shape;
  const auto obj_poly = tier4_autoware_utils::toPolygon2d(obj_pose, obj_shape);
  const auto obj_point = obj_pose.position;

  double max_dist_to_bound = std::numeric_limits<double>::lowest();
  double min_dist_to_bound = std::numeric_limits<double>::max();
  for (const auto & edge : obj_poly.outer()) {
    const auto ll_edge = lanelet::Point2d(lanelet::InvalId, edge.x(), edge.y());
    const auto dist = distance2d(boundary, ll_edge);
    max_dist_to_bound = std::max(dist, max_dist_to_bound);
    min_dist_to_bound = std::min(dist, min_dist_to_bound);
  }
  const double obj_width = std::max(max_dist_to_bound - min_dist_to_bound, 0.0);

  // distance from centerline to the boundary line with object width
  const auto centerline_pose = lanelet::utils::getClosestCenterPose(closest_lanelet, obj_point);
  const lanelet::BasicPoint3d centerline_point(
    centerline_pose.position.x, centerline_pose.position.y, centerline_pose.position.z);
  const double dist_bound_to_centerline =
    std::abs(distance2d(boundary, centerline_point)) - 0.5 * obj_width + buffer_to_bound;

  // distance from object point to centerline
  const auto centerline = closest_lanelet.centerline();
  const auto ll_obj_point = lanelet::Point2d(lanelet::InvalId, obj_point.x, obj_point.y);
  const double dist_obj_to_centerline = std::abs(distance2d(centerline, ll_obj_point));

  const double ratio = dist_obj_to_centerline / std::max(dist_bound_to_centerline, 1e-6);
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  return clamped_ratio > ratio_threshold;
}

bool passParkedObject(
  const RouteHandler & route_handler, const LaneChangePath & lane_change_path,
  const std::vector<ExtendedPredictedObject> & objects, const double minimum_lane_change_length,
  const bool is_goal_in_route, const LaneChangeParameters & lane_change_parameters,
  CollisionCheckDebugMap & object_debug)
{
  const auto & object_check_min_road_shoulder_width =
    lane_change_parameters.object_check_min_road_shoulder_width;
  const auto & object_shiftable_ratio_threshold =
    lane_change_parameters.object_shiftable_ratio_threshold;
  const auto & path = lane_change_path.path;
  const auto & current_lanes = lane_change_path.info.current_lanes;
  const auto current_lane_path =
    route_handler.getCenterLinePath(current_lanes, 0.0, std::numeric_limits<double>::max());

  if (objects.empty() || path.points.empty() || current_lane_path.points.empty()) {
    return false;
  }

  const auto leading_obj_idx = getLeadingStaticObjectIdx(
    route_handler, lane_change_path, objects, object_check_min_road_shoulder_width,
    object_shiftable_ratio_threshold);
  if (!leading_obj_idx) {
    return false;
  }

  const auto & leading_obj = objects.at(*leading_obj_idx);
  auto debug = utils::path_safety_checker::createObjectDebug(leading_obj);
  const auto leading_obj_poly =
    tier4_autoware_utils::toPolygon2d(leading_obj.initial_pose.pose, leading_obj.shape);
  if (leading_obj_poly.outer().empty()) {
    return false;
  }

  const auto & current_path_end = current_lane_path.points.back().point.pose.position;
  double min_dist_to_end_of_current_lane = std::numeric_limits<double>::max();
  for (const auto & point : leading_obj_poly.outer()) {
    const auto obj_p = tier4_autoware_utils::createPoint(point.x(), point.y(), 0.0);
    const double dist =
      motion_utils::calcSignedArcLength(current_lane_path.points, obj_p, current_path_end);
    min_dist_to_end_of_current_lane = std::min(dist, min_dist_to_end_of_current_lane);
    if (is_goal_in_route) {
      const auto goal_pose = route_handler.getGoalPose();
      const double dist_to_goal =
        motion_utils::calcSignedArcLength(current_lane_path.points, obj_p, goal_pose.position);
      min_dist_to_end_of_current_lane = std::min(min_dist_to_end_of_current_lane, dist_to_goal);
    }
  }

  // If there are still enough length after the target object, we delay the lane change
  if (min_dist_to_end_of_current_lane > minimum_lane_change_length) {
    debug.second.unsafe_reason = "delay lane change";
    utils::path_safety_checker::updateCollisionCheckDebugMap(object_debug, debug, false);
    return true;
  }

  return false;
}

std::optional<size_t> getLeadingStaticObjectIdx(
  const RouteHandler & route_handler, const LaneChangePath & lane_change_path,
  const std::vector<ExtendedPredictedObject> & objects,
  const double object_check_min_road_shoulder_width, const double object_shiftable_ratio_threshold)
{
  const auto & path = lane_change_path.path;

  if (path.points.empty() || objects.empty()) {
    return {};
  }

  const auto & lane_change_start = lane_change_path.info.lane_changing_start;
  const auto & path_end = path.points.back();

  double dist_lc_start_to_leading_obj = 0.0;
  std::optional<size_t> leading_obj_idx = std::nullopt;
  for (size_t obj_idx = 0; obj_idx < objects.size(); ++obj_idx) {
    const auto & obj = objects.at(obj_idx);
    const auto & obj_pose = obj.initial_pose.pose;

    // ignore non-static object
    // TODO(shimizu): parametrize threshold
    const double obj_vel_norm =
      std::hypot(obj.initial_twist.twist.linear.x, obj.initial_twist.twist.linear.y);
    if (obj_vel_norm > 1.0) {
      continue;
    }

    // ignore non-parked object
    if (!isParkedObject(
          path, route_handler, obj, object_check_min_road_shoulder_width,
          object_shiftable_ratio_threshold)) {
      continue;
    }

    const double dist_back_to_obj = motion_utils::calcSignedArcLength(
      path.points, path_end.point.pose.position, obj_pose.position);
    if (dist_back_to_obj > 0.0) {
      // object is not on the lane change path
      continue;
    }

    const double dist_lc_start_to_obj =
      motion_utils::calcSignedArcLength(path.points, lane_change_start.position, obj_pose.position);
    if (dist_lc_start_to_obj < 0.0) {
      // object is on the lane changing path or behind it. It will be detected in safety check
      continue;
    }

    if (dist_lc_start_to_obj > dist_lc_start_to_leading_obj) {
      dist_lc_start_to_leading_obj = dist_lc_start_to_obj;
      leading_obj_idx = obj_idx;
    }
  }

  return leading_obj_idx;
}

std::optional<lanelet::BasicPolygon2d> createPolygon(
  const lanelet::ConstLanelets & lanes, const double start_dist, const double end_dist)
{
  if (lanes.empty()) {
    return {};
  }
  const auto polygon_3d = lanelet::utils::getPolygonFromArcLength(lanes, start_dist, end_dist);
  return lanelet::utils::to2D(polygon_3d).basicPolygon();
}

ExtendedPredictedObject transform(
  const PredictedObject & object,
  [[maybe_unused]] const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters)
{
  ExtendedPredictedObject extended_object;
  extended_object.uuid = object.object_id;
  extended_object.initial_pose = object.kinematics.initial_pose_with_covariance;
  extended_object.initial_twist = object.kinematics.initial_twist_with_covariance;
  extended_object.initial_acceleration = object.kinematics.initial_acceleration_with_covariance;
  extended_object.shape = object.shape;

  const auto & time_resolution = lane_change_parameters.prediction_time_resolution;
  const auto & check_at_prepare_phase =
    lane_change_parameters.enable_prepare_segment_collision_check;
  const auto & prepare_duration = lane_change_parameters.lane_change_prepare_duration;
  const auto & velocity_threshold =
    lane_change_parameters.prepare_segment_ignore_object_velocity_thresh;
  const auto start_time = check_at_prepare_phase ? 0.0 : prepare_duration;
  const double obj_vel_norm = std::hypot(
    extended_object.initial_twist.twist.linear.x, extended_object.initial_twist.twist.linear.y);

  extended_object.predicted_paths.resize(object.kinematics.predicted_paths.size());
  for (size_t i = 0; i < object.kinematics.predicted_paths.size(); ++i) {
    const auto & path = object.kinematics.predicted_paths.at(i);
    const double end_time =
      rclcpp::Duration(path.time_step).seconds() * static_cast<double>(path.path.size() - 1);
    extended_object.predicted_paths.at(i).confidence = path.confidence;

    // create path
    for (double t = start_time; t < end_time + std::numeric_limits<double>::epsilon();
         t += time_resolution) {
      if (t < prepare_duration && obj_vel_norm < velocity_threshold) {
        continue;
      }
      const auto obj_pose = object_recognition_utils::calcInterpolatedPose(path, t);
      if (obj_pose) {
        const auto obj_polygon = tier4_autoware_utils::toPolygon2d(*obj_pose, object.shape);
        extended_object.predicted_paths.at(i).path.emplace_back(
          t, *obj_pose, obj_vel_norm, obj_polygon);
      }
    }
  }

  return extended_object;
}

bool isCollidedPolygonsInLanelet(
  const std::vector<Polygon2d> & collided_polygons, const lanelet::ConstLanelets & lanes)
{
  const auto lanes_polygon = createPolygon(lanes, 0.0, std::numeric_limits<double>::max());

  const auto is_in_lanes = [&](const auto & collided_polygon) {
    return lanes_polygon && boost::geometry::intersects(lanes_polygon.value(), collided_polygon);
  };

  return std::any_of(collided_polygons.begin(), collided_polygons.end(), is_in_lanes);
}

lanelet::ConstLanelets generateExpandedLanelets(
  const lanelet::ConstLanelets & lanes, const Direction direction, const double left_offset,
  const double right_offset)
{
  const auto left_extend_offset = (direction == Direction::LEFT) ? left_offset : 0.0;
  const auto right_extend_offset = (direction == Direction::RIGHT) ? -right_offset : 0.0;
  return lanelet::utils::getExpandedLanelets(lanes, left_extend_offset, right_extend_offset);
}

rclcpp::Logger getLogger(const std::string & type)
{
  return rclcpp::get_logger("lane_change").get_child(type);
}
}  // namespace behavior_path_planner::utils::lane_change
