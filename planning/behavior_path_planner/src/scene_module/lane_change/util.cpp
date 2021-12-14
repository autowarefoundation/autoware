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

#include "behavior_path_planner/scene_module/lane_change/util.hpp"

#include "behavior_path_planner/path_shifter/path_shifter.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
namespace lane_change_utils
{
PathWithLaneId combineReferencePath(const PathWithLaneId path1, const PathWithLaneId path2)
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
  for (const auto & pt : path.points) {
    bool is_in_lanelet = false;
    for (const auto & llt : original_lanelets) {
      if (lanelet::utils::isInLanelet(pt.point.pose, llt, 0.1)) {
        is_in_lanelet = true;
      }
    }
    for (const auto & llt : target_lanelets) {
      if (lanelet::utils::isInLanelet(pt.point.pose, llt, 0.1)) {
        is_in_lanelet = true;
      }
    }
    if (!is_in_lanelet) {
      return false;
    }
  }
  return true;
}

std::vector<LaneChangePath> getLaneChangePaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & pose, const Twist & twist,
  const BehaviorPathPlannerParameters & common_parameter, const LaneChangeParameters & parameter)
{
  std::vector<LaneChangePath> candidate_paths;

  if (original_lanelets.empty() || target_lanelets.empty()) {
    return candidate_paths;
  }

  // rename parameter
  const double backward_path_length = common_parameter.backward_path_length;
  const double forward_path_length = common_parameter.forward_path_length;
  const double lane_change_prepare_duration = parameter.lane_change_prepare_duration;
  const double lane_changing_duration = parameter.lane_changing_duration;
  const double minimum_lane_change_length = common_parameter.minimum_lane_change_length;
  const double minimum_lane_change_velocity = parameter.minimum_lane_change_velocity;
  const double maximum_deceleration = parameter.maximum_deceleration;
  const int lane_change_sampling_num = parameter.lane_change_sampling_num;

  // get velocity
  const double v0 = util::l2Norm(twist.linear);

  constexpr double buffer = 1.0;  // buffer for min_lane_change_length
  const double acceleration_resolution = std::abs(maximum_deceleration) / lane_change_sampling_num;

  const double target_distance =
    util::getArcLengthToTargetLanelet(original_lanelets, target_lanelets.front(), pose);

  for (double acceleration = 0.0; acceleration >= -maximum_deceleration;
       acceleration -= acceleration_resolution) {
    PathWithLaneId reference_path{};
    const double v1 = v0 + acceleration * lane_change_prepare_duration;
    const double v2 = v1 + acceleration * lane_changing_duration;

    // skip if velocity becomes less than zero before finishing lane change
    if (v2 < 0.0) {
      continue;
    }

    // get path on original lanes
    const double straight_distance = v0 * lane_change_prepare_duration +
                                     0.5 * acceleration * std::pow(lane_change_prepare_duration, 2);
    double lane_change_distance =
      v1 * lane_changing_duration + 0.5 * acceleration * std::pow(lane_changing_duration, 2);
    lane_change_distance = std::max(lane_change_distance, minimum_lane_change_length);

    if (straight_distance < target_distance) {
      continue;
    }

    PathWithLaneId reference_path1;
    {
      const auto arc_position = lanelet::utils::getArcCoordinates(original_lanelets, pose);
      const double s_start = arc_position.length - backward_path_length;
      const double s_end = arc_position.length + straight_distance;
      reference_path1 = route_handler.getCenterLinePath(original_lanelets, s_start, s_end);
    }

    reference_path1.points.back().point.longitudinal_velocity_mps = std::min(
      reference_path1.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(
        std::max(straight_distance / lane_change_prepare_duration, minimum_lane_change_velocity)));

    PathWithLaneId reference_path2{};
    {
      const double lane_length = lanelet::utils::getLaneletLength2d(target_lanelets);
      const auto arc_position = lanelet::utils::getArcCoordinates(target_lanelets, pose);
      const int num = std::abs(route_handler.getNumLaneToPreferredLane(target_lanelets.back()));
      double s_start = arc_position.length + straight_distance + lane_change_distance;
      s_start = std::min(s_start, lane_length - num * minimum_lane_change_length);
      double s_end = s_start + forward_path_length;
      s_end = std::min(s_end, lane_length - num * (minimum_lane_change_length + buffer));
      s_end = std::max(s_end, s_start + std::numeric_limits<double>::epsilon());
      reference_path2 = route_handler.getCenterLinePath(target_lanelets, s_start, s_end);
    }

    for (auto & point : reference_path2.points) {
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps,
        static_cast<float>(
          std::max(lane_change_distance / lane_changing_duration, minimum_lane_change_velocity)));
    }

    if (reference_path1.points.empty() || reference_path2.points.empty()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
        "reference path is empty!! something wrong...");
      continue;
    }

    LaneChangePath candidate_path;
    candidate_path.acceleration = acceleration;
    candidate_path.preparation_length = straight_distance;
    candidate_path.lane_change_length = lane_change_distance;

    PathWithLaneId target_lane_reference_path;
    {
      const lanelet::ArcCoordinates lane_change_start_arc_position =
        lanelet::utils::getArcCoordinates(
          target_lanelets, reference_path1.points.back().point.pose);
      double s_start = lane_change_start_arc_position.length;
      double s_end = s_start + straight_distance + lane_change_distance + forward_path_length;
      target_lane_reference_path = route_handler.getCenterLinePath(target_lanelets, s_start, s_end);
    }

    ShiftPoint shift_point;
    {
      const Pose lane_change_start_on_self_lane = reference_path1.points.back().point.pose;
      const Pose lane_change_end_on_target_lane = reference_path2.points.front().point.pose;
      const lanelet::ArcCoordinates lane_change_start_on_self_lane_arc =
        lanelet::utils::getArcCoordinates(target_lanelets, lane_change_start_on_self_lane);
      shift_point.length = lane_change_start_on_self_lane_arc.distance;
      shift_point.start = lane_change_start_on_self_lane;
      shift_point.end = lane_change_end_on_target_lane;
      shift_point.start_idx = tier4_autoware_utils::findNearestIndex(
        target_lane_reference_path.points, lane_change_start_on_self_lane.position);
      shift_point.end_idx = tier4_autoware_utils::findNearestIndex(
        target_lane_reference_path.points, lane_change_end_on_target_lane.position);
    }

    PathShifter path_shifter;
    path_shifter.setPath(target_lane_reference_path);
    path_shifter.addShiftPoint(shift_point);
    ShiftedPath shifted_path;

    // offset front side
    bool offset_back = false;
    if (!path_shifter.generate(&shifted_path, offset_back)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
        "failed to generate shifted path.");
    }
    const auto lanechange_end_idx = tier4_autoware_utils::findNearestIndex(
      shifted_path.path.points, reference_path2.points.front().point.pose);
    if (lanechange_end_idx) {
      for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
        auto & point = shifted_path.path.points.at(i);
        if (i < *lanechange_end_idx) {
          point.lane_ids.insert(
            point.lane_ids.end(), reference_path1.points.back().lane_ids.begin(),
            reference_path1.points.back().lane_ids.end());
          point.lane_ids.insert(
            point.lane_ids.end(), reference_path2.points.front().lane_ids.begin(),
            reference_path2.points.front().lane_ids.end());
          point.point.longitudinal_velocity_mps = std::min(
            point.point.longitudinal_velocity_mps,
            reference_path1.points.back().point.longitudinal_velocity_mps);
          continue;
        }
        point.point.longitudinal_velocity_mps = std::min(
          point.point.longitudinal_velocity_mps,
          static_cast<float>(
            std::max(lane_change_distance / lane_changing_duration, minimum_lane_change_velocity)));
        const auto nearest_idx =
          tier4_autoware_utils::findNearestIndex(reference_path2.points, point.point.pose);
        point.lane_ids = reference_path2.points.at(*nearest_idx).lane_ids;
      }

      candidate_path.path = combineReferencePath(reference_path1, shifted_path.path);
      candidate_path.shifted_path = shifted_path;
      candidate_path.shift_point = shift_point;
    } else {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
        "lane change end idx not found on target path.");
      continue;
    }

    // check candidate path is in lanelet
    if (!isPathInLanelets(candidate_path.path, original_lanelets, target_lanelets)) {
      continue;
    }

    candidate_paths.push_back(candidate_path);
  }

  return candidate_paths;
}

std::vector<LaneChangePath> selectValidPaths(
  const std::vector<LaneChangePath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const lanelet::routing::RoutingGraphContainer & overall_graphs, const Pose & current_pose,
  const bool isInGoalRouteSection, const Pose & goal_pose)
{
  std::vector<LaneChangePath> available_paths;

  for (const auto & path : paths) {
    if (hasEnoughDistance(
          path, current_lanes, target_lanes, current_pose, isInGoalRouteSection, goal_pose,
          overall_graphs)) {
      available_paths.push_back(path);
    }
  }

  return available_paths;
}

bool selectSafePath(
  const std::vector<LaneChangePath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr & dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const double vehicle_width,
  const LaneChangeParameters & ros_parameters, LaneChangePath * selected_path)
{
  for (const auto & path : paths) {
    if (isLaneChangePathSafe(
          path.path, current_lanes, target_lanes, dynamic_objects, current_pose, current_twist,
          vehicle_width, ros_parameters, true, path.acceleration)) {
      *selected_path = path;
      return true;
    }
  }

  // set first path for force lane change if no valid path found
  if (!paths.empty()) {
    *selected_path = paths.front();
    return false;
  }

  return false;
}

bool hasEnoughDistance(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  [[maybe_unused]] const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const bool isInGoalRouteSection, const Pose & goal_pose,
  const lanelet::routing::RoutingGraphContainer & overall_graphs)
{
  const double lane_change_prepare_distance = path.preparation_length;
  const double lane_changing_distance = path.lane_change_length;
  const double lane_change_total_distance = lane_change_prepare_distance + lane_changing_distance;

  if (lane_change_total_distance > util::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  if (
    lane_change_total_distance > util::getDistanceToNextIntersection(current_pose, current_lanes)) {
    return false;
  }

  if (
    isInGoalRouteSection &&
    lane_change_total_distance > util::getSignedDistance(current_pose, goal_pose, current_lanes)) {
    return false;
  }

  if (
    lane_change_total_distance >
    util::getDistanceToCrosswalk(current_pose, current_lanes, overall_graphs)) {
    return false;
  }
  return true;
}

bool isLaneChangePathSafe(
  const PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr & dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const double vehicle_width,
  const LaneChangeParameters & ros_parameters, const bool use_buffer, const double acceleration)
{
  if (path.points.empty()) {
    return false;
  }
  if (target_lanes.empty() || current_lanes.empty()) {
    return false;
  }
  if (dynamic_objects == nullptr) {
    return true;
  }
  const auto arc = lanelet::utils::getArcCoordinates(current_lanes, current_pose);
  constexpr double check_distance = 100.0;

  // parameters
  const double time_resolution = ros_parameters.prediction_time_resolution;
  const double min_thresh = ros_parameters.min_stop_distance;
  const double stop_time = ros_parameters.stop_time;

  double buffer;
  double lateral_buffer;
  if (use_buffer) {
    buffer = ros_parameters.hysteresis_buffer_distance;
    lateral_buffer = 0.5;
  } else {
    buffer = 0.0;
    lateral_buffer = 0.0;
  }
  double current_lane_check_start_time = 0.0;
  const double current_lane_check_end_time =
    ros_parameters.lane_change_prepare_duration + ros_parameters.lane_changing_duration;
  double target_lane_check_start_time = 0.0;
  const double target_lane_check_end_time =
    ros_parameters.lane_change_prepare_duration + ros_parameters.lane_changing_duration;
  if (!ros_parameters.enable_collision_check_at_prepare_phase) {
    current_lane_check_start_time = ros_parameters.lane_change_prepare_duration;
    target_lane_check_start_time = ros_parameters.lane_change_prepare_duration;
  }

  // find obstacle in lane change target lanes
  // retrieve lanes that are merging target lanes as well
  const auto target_lane_object_indices =
    util::filterObjectsByLanelets(*dynamic_objects, target_lanes);

  // find objects in current lane
  const auto current_lane_object_indices_lanelet = util::filterObjectsByLanelets(
    *dynamic_objects, current_lanes, arc.length, arc.length + check_distance);
  const auto current_lane_object_indices = util::filterObjectsByPath(
    *dynamic_objects, current_lane_object_indices_lanelet, path,
    vehicle_width / 2 + lateral_buffer);

  const auto & vehicle_predicted_path = util::convertToPredictedPath(
    path, current_twist, current_pose, target_lane_check_end_time, time_resolution, acceleration);

  // Collision check for objects in current lane
  for (const auto & i : current_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    std::vector<PredictedPath> predicted_paths;
    if (ros_parameters.use_all_predicted_path) {
      std::copy(
        obj.kinematics.predicted_paths.begin(), obj.kinematics.predicted_paths.end(),
        predicted_paths.begin());
    } else {
      auto & max_confidence_path = *(std::max_element(
        obj.kinematics.predicted_paths.begin(), obj.kinematics.predicted_paths.end(),
        [](const auto & path1, const auto & path2) {
          return path1.confidence > path2.confidence;
        }));
      predicted_paths.push_back(max_confidence_path);
    }
    for (const auto & obj_path : predicted_paths) {
      double distance = util::getDistanceBetweenPredictedPaths(
        obj_path, vehicle_predicted_path, current_lane_check_start_time,
        current_lane_check_end_time, time_resolution);
      double thresh;
      if (isObjectFront(current_pose, obj.kinematics.initial_pose_with_covariance.pose)) {
        thresh = util::l2Norm(current_twist.linear) * stop_time;
      } else {
        thresh =
          util::l2Norm(obj.kinematics.initial_twist_with_covariance.twist.linear) * stop_time;
      }
      thresh = std::max(thresh, min_thresh);
      thresh += buffer;
      if (distance < thresh) {
        return false;
      }
    }
  }

  // Collision check for objects in lane change target lane
  for (const auto & i : target_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    std::vector<PredictedPath> predicted_paths;
    if (ros_parameters.use_all_predicted_path) {
      std::copy(
        obj.kinematics.predicted_paths.begin(), obj.kinematics.predicted_paths.end(),
        predicted_paths.begin());
    } else {
      auto & max_confidence_path = *(std::max_element(
        obj.kinematics.predicted_paths.begin(), obj.kinematics.predicted_paths.end(),
        [](const auto & path1, const auto & path2) {
          return path1.confidence > path2.confidence;
        }));
      predicted_paths.push_back(max_confidence_path);
    }

    bool is_object_in_target = false;
    if (ros_parameters.use_predicted_path_outside_lanelet) {
      is_object_in_target = true;
    } else {
      for (const auto & llt : target_lanes) {
        if (lanelet::utils::isInLanelet(obj.kinematics.initial_pose_with_covariance.pose, llt)) {
          is_object_in_target = true;
        }
      }
    }

    if (is_object_in_target) {
      for (const auto & obj_path : predicted_paths) {
        const double distance = util::getDistanceBetweenPredictedPaths(
          obj_path, vehicle_predicted_path, target_lane_check_start_time,
          target_lane_check_end_time, time_resolution);
        double thresh;
        if (isObjectFront(current_pose, obj.kinematics.initial_pose_with_covariance.pose)) {
          thresh = util::l2Norm(current_twist.linear) * stop_time;
        } else {
          thresh =
            util::l2Norm(obj.kinematics.initial_twist_with_covariance.twist.linear) * stop_time;
        }
        thresh = std::max(thresh, min_thresh);
        thresh += buffer;
        if (distance < thresh) {
          return false;
        }
      }
    } else {
      const double distance = util::getDistanceBetweenPredictedPathAndObject(
        obj, vehicle_predicted_path, target_lane_check_start_time, target_lane_check_end_time,
        time_resolution);
      double thresh = min_thresh;
      if (isObjectFront(current_pose, obj.kinematics.initial_pose_with_covariance.pose)) {
        thresh = std::max(thresh, util::l2Norm(current_twist.linear) * stop_time);
      }
      thresh += buffer;
      if (distance < thresh) {
        return false;
      }
    }
  }

  return true;
}

bool isObjectFront(const Pose & ego_pose, const Pose & obj_pose)
{
  tf2::Transform tf_map2ego{};
  tf2::Transform tf_map2obj{};
  Pose obj_from_ego{};
  tf2::fromMsg(ego_pose, tf_map2ego);
  tf2::fromMsg(obj_pose, tf_map2obj);
  tf2::toMsg(tf_map2ego.inverse() * tf_map2obj, obj_from_ego);

  return obj_from_ego.position.x > 0;
}

}  // namespace lane_change_utils
}  // namespace behavior_path_planner
