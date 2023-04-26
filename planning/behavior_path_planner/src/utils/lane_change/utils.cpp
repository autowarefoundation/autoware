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

#include "behavior_path_planner/utils/lane_change/utils.hpp"

#include "behavior_path_planner/parameters.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/safety_check.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
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

PathWithLaneId combineReferencePath(const PathWithLaneId & path1, const PathWithLaneId & path2)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  return path;
}

bool isPathInLanelets(
  const PathWithLaneId & path, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets)
{
  const auto current_lane_poly = lanelet::utils::getPolygonFromArcLength(
    original_lanelets, 0, std::numeric_limits<double>::max());
  const auto target_lane_poly =
    lanelet::utils::getPolygonFromArcLength(target_lanelets, 0, std::numeric_limits<double>::max());
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
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_segment,
  const PathWithLaneId & target_lane_reference_path, const ShiftLine & shift_line,
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids, const double acceleration,
  const LaneChangePhaseInfo lane_change_length, const LaneChangePhaseInfo lane_change_velocity,
  const BehaviorPathPlannerParameters & common_parameter,
  const LaneChangeParameters & lane_change_param)
{
  PathShifter path_shifter;
  path_shifter.setPath(target_lane_reference_path);
  path_shifter.addShiftLine(shift_line);
  ShiftedPath shifted_path;

  // offset front side
  bool offset_back = false;

  const auto lane_changing_velocity = lane_change_velocity.lane_changing;
  const auto lateral_acc = lane_changing_velocity < common_parameter.lateral_acc_switching_velocity
                             ? common_parameter.lane_changing_lateral_acc_at_low_velocity
                             : common_parameter.lane_changing_lateral_acc;

  path_shifter.setVelocity(lane_changing_velocity);
  path_shifter.setLateralAccelerationLimit(std::abs(lateral_acc));

  if (!path_shifter.generate(&shifted_path, offset_back)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
      "failed to generate shifted path.");
  }

  const auto & prepare_length = lane_change_length.prepare;
  const auto & lane_changing_length = lane_change_length.lane_changing;

  LaneChangePath candidate_path;
  candidate_path.acceleration = acceleration;
  candidate_path.length.prepare = prepare_length;
  candidate_path.length.lane_changing = lane_changing_length;
  candidate_path.duration.prepare = std::invoke([&]() {
    const auto duration = prepare_length / lane_change_velocity.prepare;
    return std::min(duration, common_parameter.lane_change_prepare_duration);
  });
  candidate_path.duration.lane_changing = std::invoke([&]() {
    const auto rounding_multiplier = 1.0 / lane_change_param.prediction_time_resolution;
    return std::ceil((lane_changing_length / lane_changing_velocity) * rounding_multiplier) /
           rounding_multiplier;
  });
  candidate_path.shift_line = shift_line;
  candidate_path.reference_lanelets = original_lanelets;
  candidate_path.target_lanelets = target_lanelets;

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("constructCandidatePath"),
    "prepare_length: %f, lane_change: %f", prepare_length, lane_changing_length);

  const PathPointWithLaneId & lane_changing_start_point = prepare_segment.points.back();
  const PathPointWithLaneId & lane_changing_end_point = target_segment.points.front();
  const Pose & lane_changing_end_pose = lane_changing_end_point.point.pose;
  const auto lane_change_end_idx =
    motion_utils::findNearestIndex(shifted_path.path.points, lane_changing_end_pose);

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
        point.point.longitudinal_velocity_mps,
        lane_changing_start_point.point.longitudinal_velocity_mps);
      continue;
    }
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(lane_changing_velocity));
    const auto nearest_idx =
      motion_utils::findNearestIndex(target_segment.points, point.point.pose);
    point.lane_ids = target_segment.points.at(*nearest_idx).lane_ids;
  }

  // check candidate path is in lanelet
  if (!isPathInLanelets(shifted_path.path, original_lanelets, target_lanelets)) {
    return std::nullopt;
  }

  candidate_path.path = combineReferencePath(prepare_segment, shifted_path.path);
  candidate_path.shifted_path = shifted_path;

  return std::optional<LaneChangePath>{candidate_path};
}

#ifdef USE_OLD_ARCHITECTURE
bool getLaneChangePaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & pose, const Twist & twist,
  const PredictedObjects::ConstSharedPtr dynamic_objects,
  const BehaviorPathPlannerParameters & common_parameter, const LaneChangeParameters & parameter,
  const double check_length, LaneChangePaths * candidate_paths,
  std::unordered_map<std::string, CollisionCheckDebug> * debug_data)
#else
bool getLaneChangePaths(
  const PathWithLaneId & original_path, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  const Pose & pose, const Twist & twist, const PredictedObjects::ConstSharedPtr dynamic_objects,
  const BehaviorPathPlannerParameters & common_parameter, const LaneChangeParameters & parameter,
  const double check_length, const Direction direction, LaneChangePaths * candidate_paths,
  std::unordered_map<std::string, CollisionCheckDebug> * debug_data)
#endif
{
  debug_data->clear();
  if (original_lanelets.empty() || target_lanelets.empty()) {
    return false;
  }

  Pose ego_pose_before_collision{};

  // rename parameter
  const auto backward_path_length = common_parameter.backward_path_length;
  const auto forward_path_length = common_parameter.forward_path_length;
  const auto prepare_duration = common_parameter.lane_change_prepare_duration;
  const auto minimum_lane_changing_velocity = common_parameter.minimum_lane_changing_velocity;
  const auto lane_change_sampling_num = parameter.lane_change_sampling_num;

  // get velocity
  const auto current_velocity = twist.linear.x;

  // compute maximum_deceleration
  const auto maximum_deceleration = std::invoke(
    [&minimum_lane_changing_velocity, &current_velocity, &common_parameter, &parameter]() {
      const double min_a = (minimum_lane_changing_velocity - current_velocity) /
                           common_parameter.lane_change_prepare_duration;
      return std::clamp(
        min_a, -std::abs(common_parameter.min_acc), -std::numeric_limits<double>::epsilon());
    });

  const auto acceleration_resolution = std::abs(maximum_deceleration) / lane_change_sampling_num;

  const auto target_length =
    utils::getArcLengthToTargetLanelet(original_lanelets, target_lanelets.front(), pose);

#ifdef USE_OLD_ARCHITECTURE
  const auto shift_intervals =
    route_handler.getLateralIntervalsToPreferredLane(target_lanelets.back());
#else
  const auto get_opposite_direction =
    (direction == Direction::RIGHT) ? Direction::LEFT : Direction::RIGHT;
  const auto shift_intervals = route_handler.getLateralIntervalsToPreferredLane(
    target_lanelets.back(), get_opposite_direction);
#endif
  const auto required_total_min_length =
    utils::calcMinimumLaneChangeLength(common_parameter, shift_intervals);

  const auto is_goal_in_route = route_handler.isInGoalRouteSection(target_lanelets.back());

  const auto dist_to_end_of_current_lanes =
    utils::getDistanceToEndOfLane(pose, original_lanelets) - required_total_min_length;

  [[maybe_unused]] const auto arc_position_from_current =
    lanelet::utils::getArcCoordinates(original_lanelets, pose);
  const auto arc_position_from_target = lanelet::utils::getArcCoordinates(target_lanelets, pose);

  const auto target_lane_length = lanelet::utils::getLaneletLength2d(target_lanelets);

  const auto sorted_lane_ids = getSortedLaneIds(
    route_handler, original_lanelets, target_lanelets, arc_position_from_target.distance);
  const auto lateral_buffer = calcLateralBufferForFiltering(common_parameter.vehicle_width, 0.5);

  LaneChangeTargetObjectIndices dynamic_object_indices;

  candidate_paths->reserve(lane_change_sampling_num);
  for (double sampled_acc = 0.0; sampled_acc >= maximum_deceleration;
       sampled_acc -= acceleration_resolution) {
    const auto prepare_velocity =
      std::max(current_velocity + sampled_acc * prepare_duration, minimum_lane_changing_velocity);

    // compute actual acceleration
    const double acceleration = (prepare_velocity - current_velocity) / prepare_duration;

    // get path on original lanes
    const double prepare_length =
      current_velocity * prepare_duration + 0.5 * acceleration * std::pow(prepare_duration, 2);

    if (prepare_length < target_length) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "prepare length is shorter than distance to target lane!!");
      break;
    }

#ifdef USE_OLD_ARCHITECTURE
    const auto prepare_segment = getPrepareSegment(
      route_handler, original_lanelets, arc_position_from_current.length, backward_path_length,
      prepare_length, prepare_velocity);
#else
    const auto prepare_segment = getPrepareSegment(
      original_path, original_lanelets, pose, backward_path_length, prepare_length,
      std::max(prepare_velocity, minimum_lane_changing_velocity));
#endif

    if (prepare_segment.points.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "prepare segment is empty!!");
      continue;
    }

    // lane changing start pose is at the end of prepare segment
    const auto & lane_changing_start_pose = prepare_segment.points.back().point.pose;

    const auto target_length_from_lane_change_start_pose = utils::getArcLengthToTargetLanelet(
      original_lanelets, target_lanelets.front(), lane_changing_start_pose);
    // In new architecture, there is a possibility that the lane change start pose is behind of the
    // target lanelet, even if the condition prepare_length > target_length is satisfied. In
    // that case, the lane change shouldn't be executed.
    if (target_length_from_lane_change_start_pose > 0.0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "[only new arch] lane change start pose is behind target lanelet!!");
      break;
    }

    // calculate shift length
    const auto shift_length =
      lanelet::utils::getLateralDistanceToClosestLanelet(target_lanelets, lane_changing_start_pose);

    // we assume constant velocity during lane change
    const auto lane_changing_velocity = prepare_velocity;
    const auto lane_changing_length =
      calcLaneChangingLength(lane_changing_velocity, shift_length, common_parameter);

    if (lane_changing_length + prepare_length > dist_to_end_of_current_lanes) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "lane changing path too long");
      continue;
    }

    const auto target_segment = getTargetSegment(
      route_handler, target_lanelets, forward_path_length, lane_changing_start_pose,
      target_lane_length, lane_changing_length, lane_changing_velocity, required_total_min_length);

    if (target_segment.points.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "target segment is empty!! something wrong...");
      continue;
    }

    const auto resample_interval =
      calcLaneChangeResampleInterval(lane_changing_length, lane_changing_velocity);

    const auto lc_length = LaneChangePhaseInfo{prepare_length, lane_changing_length};
    const auto target_lane_reference_path = getReferencePathFromTargetLane(
      route_handler, target_lanelets, lane_changing_start_pose, target_lane_length,
      lc_length.lane_changing, forward_path_length, resample_interval, is_goal_in_route);

    if (target_lane_reference_path.points.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "target_lane_reference_path is empty!!");
      continue;
    }

    const auto shift_line = getLaneChangingShiftLine(
      prepare_segment, target_segment, target_lane_reference_path, shift_length);

    const auto lc_velocity = LaneChangePhaseInfo{prepare_velocity, lane_changing_velocity};

    const auto candidate_path = constructCandidatePath(
      prepare_segment, target_segment, target_lane_reference_path, shift_line, original_lanelets,
      target_lanelets, sorted_lane_ids, acceleration, lc_length, lc_velocity, common_parameter,
      parameter);

    if (!candidate_path) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "no candidate path!!");
      continue;
    }

#ifdef USE_OLD_ARCHITECTURE
    const auto is_valid = hasEnoughLength(
      *candidate_path, original_lanelets, target_lanelets, pose, route_handler,
      minimum_lane_changing_velocity, common_parameter);
#else
    const auto is_valid = hasEnoughLength(
      *candidate_path, original_lanelets, target_lanelets, pose, route_handler,
      minimum_lane_changing_velocity, common_parameter, direction);
#endif

    if (!is_valid) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "invalid candidate path!!");
      continue;
    }

    if (candidate_paths->empty()) {
      // only compute dynamic object indices once
      const auto backward_lanes = utils::lane_change::getExtendedTargetLanesForCollisionCheck(
        route_handler, target_lanelets.front(), pose, check_length);
      dynamic_object_indices = filterObjectIndices(
        {*candidate_path}, *dynamic_objects, backward_lanes, pose,
        common_parameter.forward_path_length, parameter, lateral_buffer);
    }
    candidate_paths->push_back(*candidate_path);

    const auto is_safe = isLaneChangePathSafe(
      *candidate_path, dynamic_objects, dynamic_object_indices, pose, twist, common_parameter,
      parameter, common_parameter.expected_front_deceleration,
      common_parameter.expected_rear_deceleration, ego_pose_before_collision, *debug_data,
      acceleration);

    if (is_safe) {
      return true;
    }
  }

  return false;
}

bool hasEnoughLength(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  [[maybe_unused]] const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const RouteHandler & route_handler, const double minimum_lane_changing_velocity,
  const BehaviorPathPlannerParameters & common_parameter, const Direction direction)
{
  const double lane_change_length = path.length.sum();
  const auto shift_intervals =
    route_handler.getLateralIntervalsToPreferredLane(target_lanes.back(), direction);

  double minimum_lane_change_length_to_preferred_lane = 0.0;
  for (const auto & shift_length : shift_intervals) {
    const auto lane_changing_time =
      utils::calcLaneChangingTime(minimum_lane_changing_velocity, shift_length, common_parameter);
    minimum_lane_change_length_to_preferred_lane +=
      minimum_lane_changing_velocity * lane_changing_time + common_parameter.minimum_prepare_length;
  }

  if (lane_change_length > utils::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  const auto goal_pose = route_handler.getGoalPose();
  if (
    route_handler.isInGoalRouteSection(current_lanes.back()) &&
    lane_change_length + minimum_lane_change_length_to_preferred_lane >
      utils::getSignedDistance(current_pose, goal_pose, current_lanes)) {
    return false;
  }

  // return if there are no target lanes
  if (target_lanes.empty()) {
    return true;
  }

  if (
    lane_change_length + minimum_lane_change_length_to_preferred_lane >
    utils::getDistanceToEndOfLane(current_pose, target_lanes)) {
    return false;
  }

  return true;
}

bool isLaneChangePathSafe(
  const LaneChangePath & lane_change_path, const PredictedObjects::ConstSharedPtr dynamic_objects,
  const LaneChangeTargetObjectIndices & dynamic_objects_indices, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameter,
  const LaneChangeParameters & lane_change_parameter, const double front_decel,
  const double rear_decel, Pose & ego_pose_before_collision,
  std::unordered_map<std::string, CollisionCheckDebug> & debug_data, const double acceleration)
{
  if (dynamic_objects == nullptr) {
    return true;
  }

  const auto & path = lane_change_path.path;

  if (path.points.empty()) {
    return false;
  }

  const double time_resolution = lane_change_parameter.prediction_time_resolution;
  const auto check_at_prepare_phase = lane_change_parameter.enable_prepare_segment_collision_check;

  const double check_start_time = check_at_prepare_phase ? 0.0 : lane_change_path.duration.prepare;
  const double check_end_time = lane_change_path.duration.sum();
  const double & prepare_duration = common_parameter.lane_change_prepare_duration;

  const auto current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, current_pose, common_parameter.ego_nearest_dist_threshold,
    common_parameter.ego_nearest_yaw_threshold);

  const auto vehicle_predicted_path = utils::convertToPredictedPath(
    path, current_twist, current_pose, current_seg_idx, check_end_time, time_resolution,
    prepare_duration, acceleration);
  const auto & vehicle_info = common_parameter.vehicle_info;

  auto in_lane_object_indices = dynamic_objects_indices.target_lane;
  in_lane_object_indices.insert(
    in_lane_object_indices.end(), dynamic_objects_indices.current_lane.begin(),
    dynamic_objects_indices.current_lane.end());

  RCLCPP_DEBUG(
    rclcpp::get_logger("lane_change"), "number of object -> total: %lu, in lane: %lu, others: %lu",
    dynamic_objects->objects.size(), in_lane_object_indices.size(),
    dynamic_objects_indices.other_lane.size());

  const auto assignDebugData = [](const PredictedObject & obj) {
    CollisionCheckDebug debug;
    debug.current_pose = obj.kinematics.initial_pose_with_covariance.pose;
    debug.current_twist = obj.kinematics.initial_twist_with_covariance.twist;
    return std::make_pair(tier4_autoware_utils::toHexString(obj.object_id), debug);
  };

  const auto appendDebugInfo =
    [&debug_data](std::pair<std::string, CollisionCheckDebug> & obj, bool && is_allowed) {
      const auto & key = obj.first;
      auto & element = obj.second;
      element.allow_lane_change = is_allowed;
      if (debug_data.find(key) != debug_data.end()) {
        debug_data[key] = element;
      } else {
        debug_data.insert(obj);
      }
    };

  const auto reserve_size =
    static_cast<size_t>((check_end_time - check_start_time) / time_resolution);
  std::vector<double> check_durations{};
  std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> interpolated_ego{};
  check_durations.reserve(reserve_size);
  interpolated_ego.reserve(reserve_size);

  for (double t = check_start_time; t < check_end_time; t += time_resolution) {
    tier4_autoware_utils::Polygon2d ego_polygon;
    const auto result =
      utils::getEgoExpectedPoseAndConvertToPolygon(vehicle_predicted_path, t, vehicle_info);
    if (!result) {
      continue;
    }
    check_durations.push_back(t);
    interpolated_ego.emplace_back(result->first, result->second);
  }

  for (const auto & i : in_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    auto current_debug_data = assignDebugData(obj);
    const auto predicted_paths =
      utils::getPredictedPathFromObj(obj, lane_change_parameter.use_all_predicted_path);
    for (const auto & obj_path : predicted_paths) {
      if (!utils::safety_check::isSafeInLaneletCollisionCheck(
            interpolated_ego, current_twist, check_durations, lane_change_path.duration.prepare,
            obj, obj_path, common_parameter,
            lane_change_parameter.prepare_segment_ignore_object_velocity_thresh, front_decel,
            rear_decel, ego_pose_before_collision, current_debug_data.second)) {
        appendDebugInfo(current_debug_data, false);
        return false;
      }
    }
    appendDebugInfo(current_debug_data, true);
  }

  if (!lane_change_parameter.use_predicted_path_outside_lanelet) {
    return true;
  }

  for (const auto & i : dynamic_objects_indices.other_lane) {
    const auto & obj = dynamic_objects->objects.at(i);
    auto current_debug_data = assignDebugData(obj);
    current_debug_data.second.ego_predicted_path.push_back(vehicle_predicted_path);

    const auto predicted_paths =
      utils::getPredictedPathFromObj(obj, lane_change_parameter.use_all_predicted_path);

    if (!utils::safety_check::isSafeInFreeSpaceCollisionCheck(
          interpolated_ego, current_twist, check_durations, lane_change_path.duration.prepare, obj,
          common_parameter, lane_change_parameter.prepare_segment_ignore_object_velocity_thresh,
          front_decel, rear_decel, current_debug_data.second)) {
      appendDebugInfo(current_debug_data, false);
      return false;
    }
    appendDebugInfo(current_debug_data, true);
  }
  return true;
}

PathWithLaneId getPrepareSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const double arc_length_from_current, const double backward_path_length,
  const double prepare_length, const double prepare_velocity)
{
  if (original_lanelets.empty()) {
    return PathWithLaneId();
  }

  const double s_start = arc_length_from_current - backward_path_length;
  const double s_end = arc_length_from_current + prepare_length;

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("getPrepareSegment"),
    "start: %f, end: %f", s_start, s_end);

  PathWithLaneId prepare_segment =
    route_handler.getCenterLinePath(original_lanelets, s_start, s_end);

  prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
    prepare_segment.points.back().point.longitudinal_velocity_mps,
    static_cast<float>(prepare_velocity));

  return prepare_segment;
}

PathWithLaneId getPrepareSegment(
  const PathWithLaneId & original_path, const lanelet::ConstLanelets & original_lanelets,
  const Pose & current_pose, const double backward_path_length, const double prepare_length,
  const double prepare_velocity)
{
  if (original_lanelets.empty()) {
    return PathWithLaneId();
  }

  auto prepare_segment = original_path;
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    prepare_segment.points, current_pose, 3.0, 1.0);
  utils::clipPathLength(prepare_segment, current_seg_idx, prepare_length, backward_path_length);

  prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
    prepare_segment.points.back().point.longitudinal_velocity_mps,
    static_cast<float>(prepare_velocity));

  return prepare_segment;
}

double calcLaneChangingLength(
  const double lane_changing_velocity, const double shift_length,
  const BehaviorPathPlannerParameters & common_parameter)
{
  const auto required_time =
    utils::calcLaneChangingTime(lane_changing_velocity, shift_length, common_parameter);
  const auto lane_changing_length = lane_changing_velocity * required_time;

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("calcLaneChangingLength"),
    "required_time: %f [s] lane_changing_velocity : %f [m/s], lane_changing_length: %f [m]",
    required_time, lane_changing_velocity, lane_changing_length);

  return lane_changing_length;
}

PathWithLaneId getTargetSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanelets,
  const double forward_path_length, const Pose & lane_changing_start_pose,
  const double target_lane_length, const double lane_changing_length,
  const double lane_changing_velocity, const double total_required_min_dist)
{
  const double s_start =
    std::invoke([&lane_changing_start_pose, &target_lanelets, &lane_changing_length,
                 &target_lane_length, &total_required_min_dist]() {
      const auto arc_to_start_pose =
        lanelet::utils::getArcCoordinates(target_lanelets, lane_changing_start_pose);
      const double dist_from_front_target_lanelet = arc_to_start_pose.length + lane_changing_length;
      const double end_of_lane_dist_without_buffer = target_lane_length - total_required_min_dist;
      return std::min(dist_from_front_target_lanelet, end_of_lane_dist_without_buffer);
    });

  const double s_end =
    std::invoke([&s_start, &forward_path_length, &target_lane_length, &total_required_min_dist]() {
      const double dist_from_start = s_start + forward_path_length;
      const double dist_from_end = target_lane_length - total_required_min_dist;
      return std::max(
        std::min(dist_from_start, dist_from_end), s_start + std::numeric_limits<double>::epsilon());
    });

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("getTargetSegment"),
    "start: %f, end: %f", s_start, s_end);

  PathWithLaneId target_segment = route_handler.getCenterLinePath(target_lanelets, s_start, s_end);
  for (auto & point : target_segment.points) {
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(lane_changing_velocity));
  }

  return target_segment;
}

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & lane_changing_start_pose, const double target_lane_length,
  const double lane_changing_length, const double forward_path_length,
  const double resample_interval, const bool is_goal_in_route)
{
  const ArcCoordinates lane_change_start_arc_position =
    lanelet::utils::getArcCoordinates(target_lanes, lane_changing_start_pose);

  const double s_start = lane_change_start_arc_position.length;
  const double s_end = std::invoke([&]() {
    const auto dist_from_lc_start = s_start + lane_changing_length + forward_path_length;
    if (is_goal_in_route) {
      const double s_goal =
        lanelet::utils::getArcCoordinates(target_lanes, route_handler.getGoalPose()).length;
      return std::min(dist_from_lc_start, s_goal);
    }
    return std::min(dist_from_lc_start, target_lane_length);
  });

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

bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param)
{
  const auto lane_length = lanelet::utils::getLaneletLength2d(current_lanes);
  const auto lane_poly = lanelet::utils::getPolygonFromArcLength(current_lanes, 0, lane_length);
  const auto base_link2front = common_param.base_link2front;
  const auto base_link2rear = common_param.base_link2rear;
  const auto vehicle_width = common_param.vehicle_width;
  const auto vehicle_poly =
    tier4_autoware_utils::toFootprint(current_pose, base_link2front, base_link2rear, vehicle_width);
  return boost::geometry::within(vehicle_poly, lanelet::utils::to2D(lane_poly).basicPolygon());
}

void get_turn_signal_info(
  const LaneChangePath & lane_change_path, TurnSignalInfo * turn_signal_info)
{
  turn_signal_info->desired_start_point = lane_change_path.turn_signal_info.desired_start_point;
  turn_signal_info->required_start_point = lane_change_path.turn_signal_info.required_start_point;
  turn_signal_info->required_end_point = lane_change_path.turn_signal_info.required_end_point;
  turn_signal_info->desired_end_point = lane_change_path.turn_signal_info.desired_end_point;
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

    const auto left_lane = route_handler.getLeftLanelet(current_lane);
    const auto right_lane = route_handler.getRightLanelet(current_lane);
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

std::optional<LaneChangePath> getAbortPaths(
  const std::shared_ptr<const PlannerData> & planner_data, const LaneChangePath & selected_path,
  [[maybe_unused]] const Pose & ego_pose_before_collision,
  const BehaviorPathPlannerParameters & common_param,
  [[maybe_unused]] const LaneChangeParameters & lane_change_param)
{
  const auto & route_handler = planner_data->route_handler;
  const auto current_velocity = planner_data->self_odometry->twist.twist.linear.x;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto reference_lanelets = selected_path.reference_lanelets;

  const auto ego_nearest_dist_threshold = planner_data->parameters.ego_nearest_dist_threshold;
  const auto ego_nearest_yaw_threshold = planner_data->parameters.ego_nearest_yaw_threshold;
  const double minimum_lane_change_length = 30.0;  // temporary

  const auto & lane_changing_path = selected_path.path;
  const auto lane_changing_end_pose_idx = std::invoke([&]() {
    constexpr double s_start = 0.0;
    const double s_end =
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length;

    const auto ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      lane_changing_path.points, ref.points.back().point.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  });

  const auto ego_pose_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    lane_changing_path.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

  const auto get_abort_idx_and_distance = [&](const double param_time) {
    double turning_point_dist{0.0};
    if (ego_pose_idx > lane_changing_end_pose_idx) {
      return std::make_pair(ego_pose_idx, turning_point_dist);
    }

    constexpr auto min_velocity{2.77};
    const auto desired_distance = std::max(min_velocity, current_velocity) * param_time;
    const auto & points = lane_changing_path.points;
    size_t idx{0};
    for (idx = ego_pose_idx; idx < lane_changing_end_pose_idx; ++idx) {
      const auto dist_to_ego =
        utils::getSignedDistance(current_pose, points.at(idx).point.pose, reference_lanelets);
      turning_point_dist = dist_to_ego;
      if (dist_to_ego > desired_distance) {
        break;
      }
    }
    return std::make_pair(idx, turning_point_dist);
  };

  const auto abort_delta_time = lane_change_param.abort_delta_time;
  const auto [abort_start_idx, abort_start_dist] = get_abort_idx_and_distance(abort_delta_time);
  const auto [abort_return_idx, abort_return_dist] =
    get_abort_idx_and_distance(abort_delta_time * 2);

  if (abort_start_idx >= abort_return_idx) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
      "abort start idx and return idx is equal. can't compute abort path.");
    return std::nullopt;
  }

  if (!hasEnoughLengthToLaneChangeAfterAbort(
        *route_handler, reference_lanelets, current_pose, abort_return_dist, common_param)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
      "insufficient distance to abort.");
    return std::nullopt;
  }

  const auto abort_start_pose = lane_changing_path.points.at(abort_start_idx).point.pose;
  const auto abort_return_pose = lane_changing_path.points.at(abort_return_idx).point.pose;
  const auto arc_position =
    lanelet::utils::getArcCoordinates(reference_lanelets, abort_return_pose);
  const PathWithLaneId reference_lane_segment = std::invoke([&]() {
    const double minimum_lane_changing_length =
      common_param.backward_length_buffer_for_end_of_lane + 16.5;

    const double s_start = arc_position.length;
    double s_end =
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_changing_length;

    if (route_handler->isInGoalRouteSection(selected_path.target_lanelets.back())) {
      const auto goal_arc_coordinates = lanelet::utils::getArcCoordinates(
        selected_path.target_lanelets, route_handler->getGoalPose());
      s_end = std::min(s_end, goal_arc_coordinates.length) - minimum_lane_changing_length;
    }
    PathWithLaneId ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    ref.points.back().point.longitudinal_velocity_mps = std::min(
      ref.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(common_param.minimum_lane_changing_velocity));
    return ref;
  });

  ShiftLine shift_line;
  shift_line.start = abort_start_pose;
  shift_line.end = abort_return_pose;
  shift_line.end_shift_length = -arc_position.distance;
  shift_line.start_idx = abort_start_idx;
  shift_line.end_idx = abort_return_idx;

  PathShifter path_shifter;
  path_shifter.setPath(lane_changing_path);
  path_shifter.addShiftLine(shift_line);
  const auto lateral_jerk = behavior_path_planner::PathShifter::calcJerkFromLatLonDistance(
    shift_line.end_shift_length, abort_start_dist, current_velocity);

  if (lateral_jerk > lane_change_param.abort_max_lateral_jerk) {
    return std::nullopt;
  }

  ShiftedPath shifted_path;
  // offset front side
  // bool offset_back = false;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
      "failed to generate abort shifted path.");
  }

  PathWithLaneId start_to_abort_return_pose;
  start_to_abort_return_pose.points.insert(
    start_to_abort_return_pose.points.end(), shifted_path.path.points.begin(),
    shifted_path.path.points.begin() + abort_return_idx);
  start_to_abort_return_pose.points.insert(
    start_to_abort_return_pose.points.end(), reference_lane_segment.points.begin(),
    reference_lane_segment.points.end());

  auto abort_path = selected_path;
  abort_path.shifted_path = shifted_path;
  abort_path.path = start_to_abort_return_pose;
  abort_path.shift_line = shift_line;
  return std::optional<LaneChangePath>{abort_path};
}

double getLateralShift(const LaneChangePath & path)
{
  const auto start_idx = path.shift_line.start_idx;
  const auto end_idx = path.shift_line.end_idx;

  return path.shifted_path.shift_length.at(end_idx) - path.shifted_path.shift_length.at(start_idx);
}

bool hasEnoughLengthToLaneChangeAfterAbort(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const Pose & current_pose, const double abort_return_dist,
  const BehaviorPathPlannerParameters & common_param)
{
  const double minimum_lane_changing_length = 16.5;
  const auto minimum_lane_change_length = common_param.minimum_prepare_length +
                                          minimum_lane_changing_length +
                                          common_param.backward_length_buffer_for_end_of_lane;
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
lanelet::ConstLanelets getExtendedTargetLanesForCollisionCheck(
  const RouteHandler & route_handler, const lanelet::ConstLanelet & target_lane,
  const Pose & current_pose, const double backward_length)
{
  const auto arc_length = lanelet::utils::getArcCoordinates({target_lane}, current_pose);

  if (arc_length.length >= backward_length) {
    return {};
  }

  const auto preceding_lanes = route_handler.getPrecedingLaneletSequence(
    target_lane, std::abs(backward_length - arc_length.length), {target_lane});

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

LaneChangeTargetObjectIndices filterObjectIndices(
  const LaneChangePaths & lane_change_paths, const PredictedObjects & objects,
  const lanelet::ConstLanelets & target_backward_lanes, const Pose & current_pose,
  const double forward_path_length, const LaneChangeParameters & lane_change_parameter,
  const double filter_width)
{
  // Reserve maximum amount possible

  std::vector<size_t> current_lane_obj_indices{};
  std::vector<size_t> target_lane_obj_indices{};
  std::vector<size_t> others_obj_indices{};
  current_lane_obj_indices.reserve(objects.objects.size());
  target_lane_obj_indices.reserve(objects.objects.size());
  others_obj_indices.reserve(objects.objects.size());

  const auto & longest_path = lane_change_paths.front();
  const auto & current_lanes = longest_path.reference_lanelets;
  const auto & target_lanes = longest_path.target_lanelets;
  const auto & ego_path = longest_path.path;

  const auto get_basic_polygon =
    [](const lanelet::ConstLanelets & lanes, const double start_dist, const double end_dist) {
      const auto polygon_3d = lanelet::utils::getPolygonFromArcLength(lanes, start_dist, end_dist);
      return lanelet::utils::to2D(polygon_3d).basicPolygon();
    };
  const auto arc = lanelet::utils::getArcCoordinates(current_lanes, current_pose);
  const auto current_polygon =
    get_basic_polygon(current_lanes, arc.length, arc.length + forward_path_length);
  const auto target_polygon =
    get_basic_polygon(target_lanes, 0.0, std::numeric_limits<double>::max());
  LineString2d ego_path_linestring;
  ego_path_linestring.reserve(ego_path.points.size());
  for (const auto & pt : ego_path.points) {
    const auto & position = pt.point.pose.position;
    boost::geometry::append(ego_path_linestring, Point2d(position.x, position.y));
  }

  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & obj = objects.objects.at(i);

    if (!isTargetObjectType(obj, lane_change_parameter)) {
      continue;
    }

    const auto obj_polygon = tier4_autoware_utils::toPolygon2d(obj);
    if (boost::geometry::intersects(current_polygon, obj_polygon)) {
      const double distance = boost::geometry::distance(obj_polygon, ego_path_linestring);

      if (distance < filter_width) {
        current_lane_obj_indices.push_back(i);
        continue;
      }
    }

    const bool is_intersect_with_target = boost::geometry::intersects(target_polygon, obj_polygon);
    if (is_intersect_with_target) {
      target_lane_obj_indices.push_back(i);
      continue;
    }

    const bool is_intersect_with_backward = std::invoke([&]() {
      for (const auto & ll : target_backward_lanes) {
        const bool is_intersect_with_backward =
          boost::geometry::intersects(ll.polygon2d().basicPolygon(), obj_polygon);
        if (is_intersect_with_backward) {
          target_lane_obj_indices.push_back(i);
          return true;
        }
      }
      return false;
    });

    if (!is_intersect_with_backward) {
      others_obj_indices.push_back(i);
    }
  }

  return {current_lane_obj_indices, target_lane_obj_indices, others_obj_indices};
}

bool isTargetObjectType(const PredictedObject & object, const LaneChangeParameters & parameter)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  const auto t = utils::getHighestProbLabel(object.classification);
  const auto is_object_type =
    ((t == ObjectClassification::CAR && parameter.check_car) ||
     (t == ObjectClassification::TRUCK && parameter.check_truck) ||
     (t == ObjectClassification::BUS && parameter.check_bus) ||
     (t == ObjectClassification::TRAILER && parameter.check_trailer) ||
     (t == ObjectClassification::UNKNOWN && parameter.check_unknown) ||
     (t == ObjectClassification::BICYCLE && parameter.check_bicycle) ||
     (t == ObjectClassification::MOTORCYCLE && parameter.check_motorcycle) ||
     (t == ObjectClassification::PEDESTRIAN && parameter.check_pedestrian));
  return is_object_type;
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
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const double rough_shift_length)
{
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
    lane_change_path.path.points, ego_position, lane_change_path.shift_line.start.position);
  candidate_output.finish_distance_to_path_change = motion_utils::calcSignedArcLength(
    lane_change_path.path.points, ego_position, lane_change_path.shift_line.end.position);

  return candidate_output;
}

boost::optional<lanelet::ConstLanelet> getLaneChangeTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType type, const Direction & direction)
{
  if (type == LaneChangeModuleType::NORMAL) {
    return route_handler.getLaneChangeTarget(current_lanes, direction);
  }

  return route_handler.getLaneChangeTargetExceptPreferredLane(current_lanes, direction);
}
}  // namespace behavior_path_planner::utils::lane_change
