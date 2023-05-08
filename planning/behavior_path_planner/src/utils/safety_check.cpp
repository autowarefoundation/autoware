// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/safety_check.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "perception_utils/predicted_path_utils.hpp"

namespace behavior_path_planner::utils::safety_check
{
bool isTargetObjectFront(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info, const Polygon2d & obj_polygon)
{
  const double base_to_front = vehicle_info.max_longitudinal_offset_m;
  const auto ego_point =
    tier4_autoware_utils::calcOffsetPose(ego_pose, base_to_front, 0.0, 0.0).position;

  // check all edges in the polygon
  for (const auto & obj_edge : obj_polygon.outer()) {
    const auto obj_point = tier4_autoware_utils::createPoint(obj_edge.x(), obj_edge.y(), 0.0);
    if (motion_utils::isTargetPointFront(path.points, ego_point, obj_point)) {
      return true;
    }
  }

  return false;
}

bool hasEnoughDistance(
  const Polygon2d & front_object_polygon, const double front_object_velocity,
  const Polygon2d & rear_object_polygon, const double rear_object_velocity,
  const bool is_object_front, const BehaviorPathPlannerParameters & param,
  const double front_object_deceleration, const double rear_object_deceleration)
{
  // 1. Check lateral distance between ego and target object
  const double lat_dist = calcLateralDistance(front_object_polygon, rear_object_polygon);
  if (lat_dist > param.lateral_distance_max_threshold) {
    return true;
  }

  // 2. Check physical distance between ego and target object
  const double lon_dist = calcLongitudinalDistance(front_object_polygon, rear_object_polygon);
  const auto is_unsafe_dist_between_vehicle = std::invoke([&]() {
    // ignore this for parked vehicle.
    const double object_velocity = is_object_front ? front_object_velocity : rear_object_velocity;
    if (object_velocity < 0.1) {
      return false;
    }

    // the value guarantee distance between vehicles are always more than dist
    const auto max_vel = std::max(front_object_velocity, rear_object_velocity);
    constexpr auto scale = 0.8;
    const auto dist = scale * std::abs(max_vel) + param.longitudinal_distance_min_threshold;

    // return value rounded to the nearest two floating point
    return lon_dist < dist;
  });

  if (is_unsafe_dist_between_vehicle) {
    return false;
  }

  // 3. Check longitudinal distance after deceleration
  const auto stoppingDistance = [](const auto vehicle_velocity, const auto vehicle_accel) {
    // compensate if user accidentally set the deceleration to some positive value
    const auto deceleration = (vehicle_accel < -1e-3) ? vehicle_accel : -1.0;
    return -std::pow(vehicle_velocity, 2) / (2.0 * deceleration);
  };

  const auto front_vehicle_stop_threshold =
    stoppingDistance(front_object_velocity, front_object_deceleration) + lon_dist;

  // longitudinal_distance_min_threshold here guarantee future stopping distance must be more than
  // longitudinal_distance_min_threshold
  const auto rear_vehicle_stop_threshold = std::invoke([&]() {
    const auto reaction_buffer = rear_object_velocity * param.rear_vehicle_reaction_time;
    const auto safety_buffer = rear_object_velocity * param.rear_vehicle_safety_time_margin;
    return std::max(
      reaction_buffer + safety_buffer +
        stoppingDistance(rear_object_velocity, rear_object_deceleration),
      param.longitudinal_distance_min_threshold);
  });

  return rear_vehicle_stop_threshold <= front_vehicle_stop_threshold;
}

void getTransformedPolygon(
  const Pose & front_object_pose, const Pose & rear_object_pose,
  const vehicle_info_util::VehicleInfo & ego_vehicle_info, const Shape & object_shape,
  const bool is_object_front, Polygon2d & transformed_front_object_polygon,
  Polygon2d & transformed_rear_object_polygon)
{
  // transform from map to base_link based on rear pose
  const auto transformed_front_pose =
    tier4_autoware_utils::inverseTransformPose(front_object_pose, rear_object_pose);
  Pose transformed_rear_pose;
  transformed_rear_pose.position = tier4_autoware_utils::createPoint(0.0, 0.0, 0.0);
  transformed_rear_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(0.0);

  if (is_object_front) {
    transformed_front_object_polygon =
      tier4_autoware_utils::toPolygon2d(transformed_front_pose, object_shape);
    transformed_rear_object_polygon = tier4_autoware_utils::toFootprint(
      transformed_rear_pose, ego_vehicle_info.max_longitudinal_offset_m,
      ego_vehicle_info.rear_overhang_m, ego_vehicle_info.vehicle_width_m);
  } else {
    transformed_front_object_polygon = tier4_autoware_utils::toFootprint(
      transformed_front_pose, ego_vehicle_info.max_longitudinal_offset_m,
      ego_vehicle_info.rear_overhang_m, ego_vehicle_info.vehicle_width_m);
    transformed_rear_object_polygon =
      tier4_autoware_utils::toPolygon2d(transformed_rear_pose, object_shape);
  }

  return;
}

double calcLateralDistance(
  const Polygon2d & front_object_polygon, const Polygon2d & rear_object_polygon)
{
  double min_dist = 0.0;
  for (const auto & rear_point : rear_object_polygon.outer()) {
    double max_lateral_dist = std::numeric_limits<double>::min();
    double min_lateral_dist = std::numeric_limits<double>::max();
    for (const auto & front_point : front_object_polygon.outer()) {
      const double lat_dist = front_point.y() - rear_point.y();
      max_lateral_dist = std::max(max_lateral_dist, lat_dist);
      min_lateral_dist = std::min(min_lateral_dist, lat_dist);
    }

    // sort lateral distance
    if (max_lateral_dist * min_lateral_dist < 0) {
      // if the sign is different, it means the object is crossing
      return 0.0;
    }

    const double dist = std::min(std::abs(max_lateral_dist), std::abs(min_lateral_dist));
    min_dist = std::min(min_dist, dist);
  }

  return min_dist;
}

double calcLongitudinalDistance(
  const Polygon2d & front_object_polygon, const Polygon2d & rear_object_polygon)
{
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & rear_point : rear_object_polygon.outer()) {
    double min_lon_dist = std::numeric_limits<double>::max();
    for (const auto & front_point : front_object_polygon.outer()) {
      const double lon_dist = front_point.x() - rear_point.x();
      if (lon_dist < 0.0) {
        return 0.0;
      }
      min_lon_dist = std::min(lon_dist, min_lon_dist);
    }

    min_dist = std::min(min_dist, min_lon_dist);
  }

  return min_dist;
}

bool isSafeInLaneletCollisionCheck(
  const PathWithLaneId & path,
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const PredictedPath & target_object_path, const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_velocity_thresh, const double front_object_deceleration,
  const double rear_object_deceleration, Pose & ego_pose_before_collision,
  CollisionCheckDebug & debug)
{
  debug.lerped_path.reserve(check_duration.size());

  const auto & ego_velocity = ego_current_twist.linear.x;
  const auto & object_velocity =
    target_object.kinematics.initial_twist_with_covariance.twist.linear.x;

  for (size_t i = 0; i < check_duration.size(); ++i) {
    const auto current_time = check_duration.at(i);

    if (
      current_time < prepare_duration &&
      object_velocity < prepare_phase_ignore_target_velocity_thresh) {
      continue;
    }

    const auto obj_pose = perception_utils::calcInterpolatedPose(target_object_path, current_time);
    if (!obj_pose) {
      continue;
    }

    const auto obj_polygon = tier4_autoware_utils::toPolygon2d(*obj_pose, target_object.shape);
    const auto & ego_pose = interpolated_ego.at(i).first;
    const auto & ego_polygon = interpolated_ego.at(i).second;

    // check overlap
    debug.ego_polygon = ego_polygon;
    debug.obj_polygon = obj_polygon;
    if (boost::geometry::overlaps(ego_polygon, obj_polygon)) {
      debug.failed_reason = "overlap_polygon";
      return false;
    }

    // compute which one is at the front of the other
    const bool is_object_front =
      isTargetObjectFront(path, ego_pose, common_parameters.vehicle_info, obj_polygon);
    const auto & [front_object_pose, rear_object_pose] =
      is_object_front ? std::make_pair(*obj_pose, ego_pose) : std::make_pair(ego_pose, *obj_pose);
    const auto & [front_object_velocity, rear_object_velocity] =
      is_object_front ? std::make_pair(object_velocity, ego_velocity)
                      : std::make_pair(ego_velocity, object_velocity);

    // get front and rear object polygon
    Polygon2d front_object_polygon;
    Polygon2d rear_object_polygon;
    getTransformedPolygon(
      front_object_pose, rear_object_pose, common_parameters.vehicle_info, target_object.shape,
      is_object_front, front_object_polygon, rear_object_polygon);

    debug.lerped_path.push_back(ego_pose);
    debug.expected_ego_pose = ego_pose;
    debug.expected_obj_pose = *obj_pose;
    debug.relative_to_ego = front_object_pose;
    debug.is_front = is_object_front;

    if (!hasEnoughDistance(
          front_object_polygon, front_object_velocity, rear_object_polygon, rear_object_velocity,
          is_object_front, common_parameters, front_object_deceleration,
          rear_object_deceleration)) {
      debug.failed_reason = "not_enough_longitudinal";
      return false;
    }
    ego_pose_before_collision = ego_pose;
  }
  return true;
}

bool isSafeInFreeSpaceCollisionCheck(
  const PathWithLaneId & path,
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_velocity_thresh, const double front_object_deceleration,
  const double rear_object_deceleration, CollisionCheckDebug & debug)
{
  const auto & obj_pose = target_object.kinematics.initial_pose_with_covariance.pose;
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(target_object);
  const auto & object_velocity =
    target_object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const auto & ego_velocity = ego_current_twist.linear.x;

  for (size_t i = 0; i < check_duration.size(); ++i) {
    const auto current_time = check_duration.at(i);

    if (
      current_time < prepare_duration &&
      object_velocity < prepare_phase_ignore_target_velocity_thresh) {
      continue;
    }

    const auto & ego_pose = interpolated_ego.at(i).first;
    const auto & ego_polygon = interpolated_ego.at(i).second;

    debug.ego_polygon = ego_polygon;
    debug.obj_polygon = obj_polygon;
    if (boost::geometry::overlaps(ego_polygon, obj_polygon)) {
      debug.failed_reason = "overlap_polygon";
      return false;
    }

    debug.expected_ego_pose = ego_pose;
    debug.expected_obj_pose = obj_pose;

    // compute which one is at the front of the other
    const bool is_object_front =
      isTargetObjectFront(path, ego_pose, common_parameters.vehicle_info, obj_polygon);
    const auto & [front_object_pose, rear_object_pose] =
      is_object_front ? std::make_pair(obj_pose, ego_pose) : std::make_pair(ego_pose, obj_pose);
    const auto & [front_object_velocity, rear_object_velocity] =
      is_object_front ? std::make_pair(object_velocity, ego_velocity)
                      : std::make_pair(ego_velocity, object_velocity);

    // get front and rear object polygon
    Polygon2d front_object_polygon;
    Polygon2d rear_object_polygon;
    getTransformedPolygon(
      front_object_pose, rear_object_pose, common_parameters.vehicle_info, target_object.shape,
      is_object_front, front_object_polygon, rear_object_polygon);

    if (!hasEnoughDistance(
          front_object_polygon, front_object_velocity, rear_object_polygon, rear_object_velocity,
          is_object_front, common_parameters, front_object_deceleration,
          rear_object_deceleration)) {
      debug.failed_reason = "not_enough_longitudinal";
      return false;
    }
  }
  return true;
}
}  // namespace behavior_path_planner::utils::safety_check
