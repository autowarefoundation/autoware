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

#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"
#include "behavior_path_planner/utilities.hpp"

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

namespace behavior_path_planner::lane_change_utils
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
double getExpectedVelocityWhenDecelerate(
  const double & velocity, const double & expected_acceleration, const double & duration)
{
  return velocity + expected_acceleration * duration;
}

double getDistanceWhenDecelerate(
  const double & velocity, const double & expected_acceleration, const double & duration,
  const double & minimum_distance)
{
  const auto distance = velocity * duration + 0.5 * expected_acceleration * std::pow(duration, 2);
  return std::max(distance, minimum_distance);
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
  const auto & backward_path_length = common_parameter.backward_path_length;
  const auto & forward_path_length = common_parameter.forward_path_length;
  const auto & lane_change_prepare_duration = parameter.lane_change_prepare_duration;
  const auto & lane_changing_duration = parameter.lane_changing_duration;
  const auto & minimum_lane_change_prepare_distance =
    parameter.minimum_lane_change_prepare_distance;
  const auto & minimum_lane_change_length = common_parameter.minimum_lane_change_length;
  const auto & minimum_lane_change_velocity = parameter.minimum_lane_change_velocity;
  const auto & maximum_deceleration = parameter.maximum_deceleration;
  const auto & lane_change_sampling_num = parameter.lane_change_sampling_num;

  // get velocity
  const double current_velocity = util::l2Norm(twist.linear);

  constexpr double buffer = 1.0;  // buffer for min_lane_change_length
  const double acceleration_resolution = std::abs(maximum_deceleration) / lane_change_sampling_num;

  const double target_distance =
    util::getArcLengthToTargetLanelet(original_lanelets, target_lanelets.front(), pose);

  for (double acceleration = 0.0; acceleration >= -maximum_deceleration;
       acceleration -= acceleration_resolution) {
    const double lane_change_prepare_velocity = getExpectedVelocityWhenDecelerate(
      current_velocity, acceleration, lane_change_prepare_duration);
    const double lane_changing_velocity = getExpectedVelocityWhenDecelerate(
      lane_change_prepare_velocity, acceleration, lane_changing_duration);

    // skip if velocity becomes less than zero before finishing lane change
    if (lane_changing_velocity < 0.0) {
      continue;
    }

    // get path on original lanes
    const double prepare_distance = getDistanceWhenDecelerate(
      current_velocity, acceleration, lane_change_prepare_duration,
      minimum_lane_change_prepare_distance);
    const double lane_changing_distance = getDistanceWhenDecelerate(
      lane_change_prepare_velocity, acceleration, lane_changing_duration,
      minimum_lane_change_length + buffer);

    if (prepare_distance < target_distance) {
      continue;
    }

    PathWithLaneId reference_path1;
    {
      const auto arc_position = lanelet::utils::getArcCoordinates(original_lanelets, pose);
      const double s_start = arc_position.length - backward_path_length;
      const double s_end = arc_position.length + prepare_distance;
      reference_path1 = route_handler.getCenterLinePath(original_lanelets, s_start, s_end);
    }

    reference_path1.points.back().point.longitudinal_velocity_mps = std::min(
      reference_path1.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(
        std::max(prepare_distance / lane_change_prepare_duration, minimum_lane_change_velocity)));

    PathWithLaneId reference_path2{};
    {
      const double lane_length = lanelet::utils::getLaneletLength2d(target_lanelets);
      const auto arc_position = lanelet::utils::getArcCoordinates(target_lanelets, pose);
      const int num = std::abs(route_handler.getNumLaneToPreferredLane(target_lanelets.back()));
      double s_start = arc_position.length + prepare_distance + lane_changing_distance;
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
          std::max(lane_changing_distance / lane_changing_duration, minimum_lane_change_velocity)));
    }

    if (reference_path1.points.empty() || reference_path2.points.empty()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
        "reference path is empty!! something wrong...");
      continue;
    }

    LaneChangePath candidate_path;
    candidate_path.acceleration = acceleration;
    candidate_path.preparation_length = prepare_distance;
    candidate_path.lane_change_length = lane_changing_distance;

    PathWithLaneId target_lane_reference_path;
    {
      const lanelet::ArcCoordinates lane_change_start_arc_position =
        lanelet::utils::getArcCoordinates(
          target_lanelets, reference_path1.points.back().point.pose);
      double s_start = lane_change_start_arc_position.length;
      double s_end = s_start + prepare_distance + lane_changing_distance + forward_path_length;
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
      shift_point.start_idx = motion_utils::findNearestIndex(
        target_lane_reference_path.points, lane_change_start_on_self_lane.position);
      shift_point.end_idx = motion_utils::findNearestIndex(
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
    const auto lanechange_end_idx = motion_utils::findNearestIndex(
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
          static_cast<float>(std::max(
            lane_changing_distance / lane_changing_duration, minimum_lane_change_velocity)));
        const auto nearest_idx =
          motion_utils::findNearestIndex(reference_path2.points, point.point.pose);
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
  const PredictedObjects::ConstSharedPtr dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & ros_parameters, LaneChangePath * selected_path)
{
  for (const auto & path : paths) {
    if (isLaneChangePathSafe(
          path.path, current_lanes, target_lanes, dynamic_objects, current_pose, current_twist,
          common_parameters, ros_parameters, true, path.acceleration)) {
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
  const PredictedObjects::ConstSharedPtr dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const bool use_buffer,
  const double acceleration)
{
  if (dynamic_objects == nullptr) {
    return true;
  }

  if (path.points.empty() || target_lanes.empty() || current_lanes.empty()) {
    return false;
  }

  const double time_resolution = lane_change_parameters.prediction_time_resolution;
  const auto & lane_change_prepare_duration = lane_change_parameters.lane_change_prepare_duration;
  const auto & lane_changing_duration = lane_change_parameters.lane_changing_duration;
  const double current_lane_check_start_time =
    (!lane_change_parameters.enable_collision_check_at_prepare_phase) ? lane_change_prepare_duration
                                                                      : 0.0;
  const double current_lane_check_end_time = lane_change_prepare_duration + lane_changing_duration;
  double target_lane_check_start_time =
    (!lane_change_parameters.enable_collision_check_at_prepare_phase) ? lane_change_prepare_duration
                                                                      : 0.0;
  const double target_lane_check_end_time = lane_change_prepare_duration + lane_changing_duration;
  constexpr double ego_predicted_path_min_speed{1.0};
  const auto vehicle_predicted_path = util::convertToPredictedPath(
    path, current_twist, current_pose, target_lane_check_end_time, time_resolution, acceleration,
    ego_predicted_path_min_speed);

  const auto arc = lanelet::utils::getArcCoordinates(current_lanes, current_pose);

  // find obstacle in lane change target lanes
  // retrieve lanes that are merging target lanes as well
  const auto target_lane_object_indices =
    util::filterObjectsByLanelets(*dynamic_objects, target_lanes);

  // find objects in current lane
  constexpr double check_distance = 100.0;
  const auto current_lane_object_indices_lanelet = util::filterObjectsByLanelets(
    *dynamic_objects, current_lanes, arc.length, arc.length + check_distance);

  const double lateral_buffer = (use_buffer) ? 0.5 : 0.0;
  const auto & vehicle_width = common_parameters.vehicle_width;
  const auto & vehicle_length = common_parameters.vehicle_length;
  const auto current_lane_object_indices = util::filterObjectsByPath(
    *dynamic_objects, current_lane_object_indices_lanelet, path,
    vehicle_width / 2 + lateral_buffer);

  for (const auto & i : current_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    const auto predicted_paths =
      util::getPredictedPathFromObj(obj, lane_change_parameters.use_all_predicted_path);
    for (const auto & obj_path : predicted_paths) {
      if (!util::isSafeInLaneletCollisionCheck(
            current_pose, current_twist, vehicle_predicted_path, vehicle_length, vehicle_width,
            current_lane_check_start_time, current_lane_check_end_time, time_resolution, obj,
            obj_path, common_parameters)) {
        return false;
      }
    }
  }

  // Collision check for objects in lane change target lane
  for (const auto & i : target_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    bool is_object_in_target = false;
    if (lane_change_parameters.use_predicted_path_outside_lanelet) {
      is_object_in_target = true;
    } else {
      for (const auto & llt : target_lanes) {
        if (lanelet::utils::isInLanelet(obj.kinematics.initial_pose_with_covariance.pose, llt)) {
          is_object_in_target = true;
        }
      }
    }

    const auto predicted_paths =
      util::getPredictedPathFromObj(obj, lane_change_parameters.use_all_predicted_path);

    if (is_object_in_target) {
      for (const auto & obj_path : predicted_paths) {
        if (!util::isSafeInLaneletCollisionCheck(
              current_pose, current_twist, vehicle_predicted_path, vehicle_length, vehicle_width,
              target_lane_check_start_time, target_lane_check_end_time, time_resolution, obj,
              obj_path, common_parameters)) {
          return false;
        }
      }
    } else {
      if (!util::isSafeInFreeSpaceCollisionCheck(
            current_pose, current_twist, vehicle_predicted_path, vehicle_length, vehicle_width,
            target_lane_check_start_time, target_lane_check_end_time, time_resolution, obj,
            common_parameters)) {
        return false;
      }
    }
  }
  return true;
}
}  // namespace behavior_path_planner::lane_change_utils
