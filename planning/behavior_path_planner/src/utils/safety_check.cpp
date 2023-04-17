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

#include "perception_utils/predicted_path_utils.hpp"

namespace behavior_path_planner::utils::safety_check
{
template <typename Pythagoras>
ProjectedDistancePoint pointToSegment(
  const Point2d & reference_point, const Point2d & polygon_segment_start,
  const Point2d & polygon_segment_end)
{
  auto copied_point_from_object = polygon_segment_end;
  auto copied_point_from_reference = reference_point;
  bg::subtract_point(copied_point_from_object, polygon_segment_start);
  bg::subtract_point(copied_point_from_reference, polygon_segment_start);

  const auto c1 = bg::dot_product(copied_point_from_reference, copied_point_from_object);
  if (!(c1 > 0)) {
    return {polygon_segment_start, Pythagoras::apply(reference_point, polygon_segment_start)};
  }

  const auto c2 = bg::dot_product(copied_point_from_object, copied_point_from_object);
  if (!(c2 > c1)) {
    return {polygon_segment_end, Pythagoras::apply(reference_point, polygon_segment_end)};
  }

  Point2d projected = polygon_segment_start;
  bg::multiply_value(copied_point_from_object, c1 / c2);
  bg::add_point(projected, copied_point_from_object);

  return {projected, Pythagoras::apply(reference_point, projected)};
}

void getProjectedDistancePointFromPolygons(
  const Polygon2d & ego_polygon, const Polygon2d & object_polygon, Pose & point_on_ego,
  Pose & point_on_object)
{
  ProjectedDistancePoint nearest;
  std::unique_ptr<Point2d> current_point;

  bool points_in_ego{false};

  const auto findPoints = [&nearest, &current_point, &points_in_ego](
                            const Polygon2d & polygon_for_segment,
                            const Polygon2d & polygon_for_points, const bool & ego_is_points) {
    const auto segments = boost::make_iterator_range(
      bg::segments_begin(polygon_for_segment), bg::segments_end(polygon_for_segment));
    const auto points = boost::make_iterator_range(
      bg::points_begin(polygon_for_points), bg::points_end(polygon_for_points));

    for (auto && segment : segments) {
      for (auto && point : points) {
        const auto projected = pointToSegment(point, *segment.first, *segment.second);
        if (!current_point || projected.distance < nearest.distance) {
          current_point = std::make_unique<Point2d>(point);
          nearest = projected;
          points_in_ego = ego_is_points;
        }
      }
    }
  };

  std::invoke(findPoints, ego_polygon, object_polygon, false);
  std::invoke(findPoints, object_polygon, ego_polygon, true);

  if (!points_in_ego) {
    point_on_object.position.x = current_point->x();
    point_on_object.position.y = current_point->y();
    point_on_ego.position.x = nearest.projected_point.x();
    point_on_ego.position.y = nearest.projected_point.y();
  } else {
    point_on_ego.position.x = current_point->x();
    point_on_ego.position.y = current_point->y();
    point_on_object.position.x = nearest.projected_point.x();
    point_on_object.position.y = nearest.projected_point.y();
  }
}
Pose projectCurrentPoseToTarget(const Pose & desired_object, const Pose & target_object)
{
  tf2::Transform tf_map_desired_to_global{};
  tf2::Transform tf_map_target_to_global{};

  tf2::fromMsg(desired_object, tf_map_desired_to_global);
  tf2::fromMsg(target_object, tf_map_target_to_global);

  Pose desired_obj_pose_projected_to_target{};
  tf2::toMsg(
    tf_map_desired_to_global.inverse() * tf_map_target_to_global,
    desired_obj_pose_projected_to_target);

  return desired_obj_pose_projected_to_target;
}

bool hasEnoughDistance(
  const Pose & expected_ego_pose, const Twist & ego_current_twist,
  const Pose & expected_object_pose, const Twist & object_current_twist,
  const BehaviorPathPlannerParameters & param, const double front_decel, const double rear_decel,
  CollisionCheckDebug & debug)
{
  const auto front_vehicle_pose =
    projectCurrentPoseToTarget(expected_ego_pose, expected_object_pose);
  debug.relative_to_ego = front_vehicle_pose;

  // 1. Check lateral distance between ego and target object
  if (std::abs(front_vehicle_pose.position.y) > param.lateral_distance_max_threshold) {
    return true;
  }

  const auto is_obj_in_front = front_vehicle_pose.position.x > -1e-3;
  debug.is_front = is_obj_in_front;

  const auto [front_vehicle_velocity, rear_vehicle_velocity] = std::invoke([&]() {
    debug.object_twist.linear = object_current_twist.linear;
    if (is_obj_in_front) {
      return std::make_pair(object_current_twist.linear.x, ego_current_twist.linear.x);
    }
    return std::make_pair(ego_current_twist.linear.x, object_current_twist.linear.x);
  });

  // 2. Check physical distance between ego and target object
  const auto is_unsafe_dist_between_vehicle = std::invoke([&]() {
    // ignore this for parked vehicle.
    if (object_current_twist.linear.x < 0.1) {
      return false;
    }

    // the value guarantee distance between vehicles are always more than dist
    const auto max_vel = std::max(front_vehicle_velocity, rear_vehicle_velocity);
    constexpr auto scale = 0.8;
    const auto dist = scale * std::abs(max_vel) + param.longitudinal_distance_min_threshold;

    // return value rounded to the nearest two floating point
    return std::abs(front_vehicle_pose.position.x) < dist;
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
    stoppingDistance(front_vehicle_velocity, front_decel) + std::abs(front_vehicle_pose.position.x);

  // longitudinal_distance_min_threshold here guarantee future stopping distance must be more than
  // longitudinal_distance_min_threshold
  const auto rear_vehicle_stop_threshold = std::invoke([&]() {
    const auto reaction_buffer = rear_vehicle_velocity * param.rear_vehicle_reaction_time;
    const auto safety_buffer = rear_vehicle_velocity * param.rear_vehicle_safety_time_margin;
    return std::max(
      reaction_buffer + safety_buffer + stoppingDistance(rear_vehicle_velocity, rear_decel),
      param.longitudinal_distance_min_threshold);
  });

  return rear_vehicle_stop_threshold <= front_vehicle_stop_threshold;
}

bool isSafeInLaneletCollisionCheck(
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const PredictedPath & target_object_path, const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_speed_thresh, const double front_decel,
  const double rear_decel, Pose & ego_pose_before_collision, CollisionCheckDebug & debug)
{
  debug.lerped_path.reserve(check_duration.size());

  const auto & object_twist = target_object.kinematics.initial_twist_with_covariance.twist;
  const auto object_speed = object_twist.linear.x;
  const auto ignore_check_at_time = [&](const double current_time) {
    return (
      (current_time < prepare_duration) &&
      (object_speed < prepare_phase_ignore_target_speed_thresh));
  };

  for (size_t i = 0; i < check_duration.size(); ++i) {
    const auto current_time = check_duration.at(i);

    if (ignore_check_at_time(current_time)) {
      continue;
    }

    const auto found_expected_obj_pose =
      perception_utils::calcInterpolatedPose(target_object_path, current_time);

    if (!found_expected_obj_pose) {
      continue;
    }

    auto expected_obj_pose = *found_expected_obj_pose;
    const auto & obj_polygon =
      tier4_autoware_utils::toPolygon2d(expected_obj_pose, target_object.shape);
    const auto & ego_info = interpolated_ego.at(i);
    auto expected_ego_pose = ego_info.first;
    const auto & ego_polygon = ego_info.second;

    debug.ego_polygon = ego_polygon;
    debug.obj_polygon = obj_polygon;
    if (boost::geometry::overlaps(ego_polygon, obj_polygon)) {
      debug.failed_reason = "overlap_polygon";
      return false;
    }

    debug.lerped_path.push_back(expected_ego_pose);

    getProjectedDistancePointFromPolygons(
      ego_polygon, obj_polygon, expected_ego_pose, expected_obj_pose);
    debug.expected_ego_pose = expected_ego_pose;
    debug.expected_obj_pose = expected_obj_pose;

    if (!hasEnoughDistance(
          expected_ego_pose, ego_current_twist, expected_obj_pose, object_twist, common_parameters,
          front_decel, rear_decel, debug)) {
      debug.failed_reason = "not_enough_longitudinal";
      return false;
    }
    ego_pose_before_collision = expected_ego_pose;
  }
  return true;
}

bool isSafeInFreeSpaceCollisionCheck(
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_speed_thresh, const double front_decel,
  const double rear_decel, CollisionCheckDebug & debug)
{
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(target_object);
  const auto & object_twist = target_object.kinematics.initial_twist_with_covariance.twist;
  const auto object_speed = object_twist.linear.x;
  const auto ignore_check_at_time = [&](const double current_time) {
    return (
      (current_time < prepare_duration) &&
      (object_speed < prepare_phase_ignore_target_speed_thresh));
  };
  for (size_t i = 0; i < check_duration.size(); ++i) {
    const auto current_time = check_duration.at(i);

    if (ignore_check_at_time(current_time)) {
      continue;
    }
    const auto & ego_info = interpolated_ego.at(i);
    auto expected_ego_pose = ego_info.first;
    const auto & ego_polygon = ego_info.second;

    debug.ego_polygon = ego_polygon;
    debug.obj_polygon = obj_polygon;
    if (boost::geometry::overlaps(ego_polygon, obj_polygon)) {
      debug.failed_reason = "overlap_polygon";
      return false;
    }

    auto expected_obj_pose = target_object.kinematics.initial_pose_with_covariance.pose;
    getProjectedDistancePointFromPolygons(
      ego_polygon, obj_polygon, expected_ego_pose, expected_obj_pose);

    debug.expected_ego_pose = expected_ego_pose;
    debug.expected_obj_pose = expected_obj_pose;

    const auto object_twist = target_object.kinematics.initial_twist_with_covariance.twist;
    if (!hasEnoughDistance(
          expected_ego_pose, ego_current_twist,
          target_object.kinematics.initial_pose_with_covariance.pose, object_twist,
          common_parameters, front_decel, rear_decel, debug)) {
      debug.failed_reason = "not_enough_longitudinal";
      return false;
    }
  }
  return true;
}
}  // namespace behavior_path_planner::utils::safety_check
