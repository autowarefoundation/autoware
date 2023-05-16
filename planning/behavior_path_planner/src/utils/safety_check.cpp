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
void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

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

Polygon2d createExtendedPolygon(
  const Pose & base_link_pose, const vehicle_info_util::VehicleInfo & vehicle_info,
  const double lon_length, const double lat_margin)
{
  const double & base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double & width = vehicle_info.vehicle_width_m;
  const double & base_to_rear = vehicle_info.rear_overhang_m;

  const double lon_offset = std::max(lon_length + base_to_front, base_to_front);
  const double lat_offset = width / 2.0 + lat_margin;
  const auto p1 = tier4_autoware_utils::calcOffsetPose(base_link_pose, lon_offset, lat_offset, 0.0);
  const auto p2 =
    tier4_autoware_utils::calcOffsetPose(base_link_pose, lon_offset, -lat_offset, 0.0);
  const auto p3 =
    tier4_autoware_utils::calcOffsetPose(base_link_pose, -base_to_rear, -lat_offset, 0.0);
  const auto p4 =
    tier4_autoware_utils::calcOffsetPose(base_link_pose, -base_to_rear, lat_offset, 0.0);

  Polygon2d polygon;
  appendPointToPolygon(polygon, p1.position);
  appendPointToPolygon(polygon, p2.position);
  appendPointToPolygon(polygon, p3.position);
  appendPointToPolygon(polygon, p4.position);
  appendPointToPolygon(polygon, p1.position);
  return tier4_autoware_utils::isClockwise(polygon)
           ? polygon
           : tier4_autoware_utils::inverseClockwise(polygon);
}

Polygon2d createExtendedPolygon(
  const Pose & obj_pose, const Shape & shape, const double lon_length, const double lat_margin)
{
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(obj_pose, shape);
  if (obj_polygon.outer().empty()) {
    return obj_polygon;
  }

  double max_x = std::numeric_limits<double>::lowest();
  double min_x = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  for (const auto & polygon_p : obj_polygon.outer()) {
    const auto obj_p = tier4_autoware_utils::createPoint(polygon_p.x(), polygon_p.y(), 0.0);
    const auto transformed_p = tier4_autoware_utils::inverseTransformPoint(obj_p, obj_pose);

    max_x = std::max(transformed_p.x, max_x);
    min_x = std::min(transformed_p.x, min_x);
    max_y = std::max(transformed_p.y, max_y);
    min_y = std::min(transformed_p.y, min_y);
  }

  const double lon_offset = max_x + lon_length;
  const double left_lat_offset = max_y + lat_margin;
  const double right_lat_offset = min_y - lat_margin;
  const auto p1 = tier4_autoware_utils::calcOffsetPose(obj_pose, lon_offset, left_lat_offset, 0.0);
  const auto p2 = tier4_autoware_utils::calcOffsetPose(obj_pose, lon_offset, right_lat_offset, 0.0);
  const auto p3 = tier4_autoware_utils::calcOffsetPose(obj_pose, min_x, right_lat_offset, 0.0);
  const auto p4 = tier4_autoware_utils::calcOffsetPose(obj_pose, min_x, left_lat_offset, 0.0);

  Polygon2d polygon;
  appendPointToPolygon(polygon, p1.position);
  appendPointToPolygon(polygon, p2.position);
  appendPointToPolygon(polygon, p3.position);
  appendPointToPolygon(polygon, p4.position);
  appendPointToPolygon(polygon, p1.position);
  return tier4_autoware_utils::isClockwise(polygon)
           ? polygon
           : tier4_autoware_utils::inverseClockwise(polygon);
}

double calcRssDistance(
  const double front_object_velocity, const double rear_object_velocity,
  const double front_object_deceleration, const double rear_object_deceleration,
  const BehaviorPathPlannerParameters & params)
{
  const auto stoppingDistance = [](const auto vehicle_velocity, const auto vehicle_accel) {
    // compensate if user accidentally set the deceleration to some positive value
    const auto deceleration = (vehicle_accel < -1e-3) ? vehicle_accel : -1.0;
    return -std::pow(vehicle_velocity, 2) / (2.0 * deceleration);
  };

  const double & reaction_time =
    params.rear_vehicle_reaction_time + params.rear_vehicle_safety_time_margin;

  const double front_object_stop_length =
    stoppingDistance(front_object_velocity, front_object_deceleration);
  const double rear_object_stop_length =
    rear_object_velocity * reaction_time +
    stoppingDistance(rear_object_velocity, rear_object_deceleration);
  return rear_object_stop_length - front_object_stop_length;
}

double calcMinimumLongitudinalLength(
  const double front_object_velocity, const double rear_object_velocity,
  const BehaviorPathPlannerParameters & params)
{
  const double & lon_threshold = params.longitudinal_distance_min_threshold;
  const auto max_vel = std::max(front_object_velocity, rear_object_velocity);
  constexpr auto scale = 0.8;
  return scale * std::abs(max_vel) + lon_threshold;
}

bool isSafeInLaneletCollisionCheck(
  const PathWithLaneId & path,
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const PredictedPath & target_object_path, const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_velocity_thresh, const double front_object_deceleration,
  const double rear_object_deceleration, CollisionCheckDebug & debug)
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
    const auto & [front_object_velocity, rear_object_velocity] =
      is_object_front ? std::make_pair(object_velocity, ego_velocity)
                      : std::make_pair(ego_velocity, object_velocity);

    // compute rss dist
    const auto rss_dist = calcRssDistance(
      front_object_velocity, rear_object_velocity, front_object_deceleration,
      rear_object_deceleration, common_parameters);

    // minimum longitudinal length
    const auto min_lon_length =
      calcMinimumLongitudinalLength(front_object_velocity, rear_object_velocity, common_parameters);

    const auto & lon_offset = std::max(rss_dist, min_lon_length);
    const auto & ego_vehicle_info = common_parameters.vehicle_info;
    const auto & lat_margin = common_parameters.lateral_distance_max_threshold;
    const auto & extended_ego_polygon =
      is_object_front ? createExtendedPolygon(ego_pose, ego_vehicle_info, lon_offset, lat_margin)
                      : ego_polygon;
    const auto & extended_obj_polygon =
      is_object_front
        ? obj_polygon
        : createExtendedPolygon(*obj_pose, target_object.shape, lon_offset, lat_margin);

    debug.lerped_path.push_back(ego_pose);
    debug.expected_ego_pose = ego_pose;
    debug.expected_obj_pose = *obj_pose;
    debug.is_front = is_object_front;

    if (boost::geometry::overlaps(extended_ego_polygon, extended_obj_polygon)) {
      debug.failed_reason = "overlap_extended_polygon";
      return false;
    }
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
    const auto & [front_object_velocity, rear_object_velocity] =
      is_object_front ? std::make_pair(object_velocity, ego_velocity)
                      : std::make_pair(ego_velocity, object_velocity);
    // compute rss dist
    const auto rss_dist = calcRssDistance(
      front_object_velocity, rear_object_velocity, front_object_deceleration,
      rear_object_deceleration, common_parameters);

    // minimum longitudinal length
    const auto min_lon_length =
      calcMinimumLongitudinalLength(front_object_velocity, rear_object_velocity, common_parameters);

    const auto & lon_offset = std::max(rss_dist, min_lon_length);
    const auto & ego_vehicle_info = common_parameters.vehicle_info;
    const auto & lat_margin = common_parameters.lateral_distance_max_threshold;
    const auto & extended_ego_polygon =
      is_object_front ? createExtendedPolygon(ego_pose, ego_vehicle_info, lon_offset, lat_margin)
                      : ego_polygon;
    const auto & extended_obj_polygon =
      is_object_front
        ? obj_polygon
        : createExtendedPolygon(obj_pose, target_object.shape, lon_offset, lat_margin);

    debug.lerped_path.push_back(ego_pose);
    debug.expected_ego_pose = ego_pose;
    debug.expected_obj_pose = obj_pose;
    debug.is_front = is_object_front;

    if (boost::geometry::overlaps(extended_ego_polygon, extended_obj_polygon)) {
      debug.failed_reason = "overlap_extended_polygon";
      return false;
    }
  }
  return true;
}
}  // namespace behavior_path_planner::utils::safety_check
