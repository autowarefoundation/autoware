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

#include "behavior_path_planner/utils/path_safety_checker/safety_check.hpp"

#include "behavior_path_planner/marker_utils/utils.hpp"
#include "interpolation/linear_interpolation.hpp"
#include "motion_utils/trajectory/path_with_lane_id.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

namespace behavior_path_planner::utils::path_safety_checker
{
void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

bool isTargetObjectOncoming(
  const geometry_msgs::msg::Pose & vehicle_pose, const geometry_msgs::msg::Pose & object_pose)
{
  return std::abs(calcYawDeviation(vehicle_pose, object_pose)) > M_PI_2;
}

bool isTargetObjectFront(
  const geometry_msgs::msg::Pose & ego_pose, const Polygon2d & obj_polygon,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  const double base_to_front = vehicle_info.max_longitudinal_offset_m;
  const auto ego_offset_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, base_to_front, 0.0, 0.0);

  // check all edges in the polygon
  for (const auto & obj_edge : obj_polygon.outer()) {
    const auto obj_point = tier4_autoware_utils::createPoint(obj_edge.x(), obj_edge.y(), 0.0);
    if (tier4_autoware_utils::calcLongitudinalDeviation(ego_offset_pose, obj_point) > 0.0) {
      return true;
    }
  }

  return false;
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
  const double lon_length, const double lat_margin, CollisionCheckDebug & debug)
{
  const double & base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double & width = vehicle_info.vehicle_width_m;
  const double & base_to_rear = vehicle_info.rear_overhang_m;

  const double lon_offset = std::max(lon_length + base_to_front, base_to_front);

  const double lat_offset = width / 2.0 + lat_margin;

  {
    debug.extended_polygon_lon_offset = lon_offset;
    debug.extended_polygon_lat_offset = lat_offset;
  }

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
  const Pose & obj_pose, const Shape & shape, const double lon_length, const double lat_margin,
  CollisionCheckDebug & debug)
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

  {
    debug.extended_polygon_lon_offset = lon_offset;
    debug.extended_polygon_lat_offset = (left_lat_offset + right_lat_offset) / 2;
  }

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

PredictedPath convertToPredictedPath(
  const std::vector<PoseWithVelocityStamped> & path, const double time_resolution)
{
  PredictedPath predicted_path;
  predicted_path.time_step = rclcpp::Duration::from_seconds(time_resolution);
  predicted_path.path.resize(path.size());

  for (size_t i = 0; i < path.size(); ++i) {
    predicted_path.path.at(i) = path.at(i).pose;
  }
  return predicted_path;
}

double calcRssDistance(
  const double front_object_velocity, const double rear_object_velocity,
  const RSSparams & rss_params)
{
  const auto stoppingDistance = [](const auto vehicle_velocity, const auto vehicle_accel) {
    // compensate if user accidentally set the deceleration to some positive value
    const auto deceleration = (vehicle_accel < -1e-3) ? vehicle_accel : -1.0;
    return -std::pow(vehicle_velocity, 2) / (2.0 * deceleration);
  };

  const double & reaction_time =
    rss_params.rear_vehicle_reaction_time + rss_params.rear_vehicle_safety_time_margin;

  const double front_object_stop_length =
    stoppingDistance(front_object_velocity, rss_params.front_vehicle_deceleration);
  const double rear_object_stop_length =
    rear_object_velocity * reaction_time +
    stoppingDistance(rear_object_velocity, rss_params.rear_vehicle_deceleration);
  return rear_object_stop_length - front_object_stop_length;
}

double calcMinimumLongitudinalLength(
  const double front_object_velocity, const double rear_object_velocity,
  const RSSparams & rss_params)
{
  const double & lon_threshold = rss_params.longitudinal_distance_min_threshold;
  const auto max_vel = std::max(front_object_velocity, rear_object_velocity);
  return rss_params.longitudinal_velocity_delta_time * std::abs(max_vel) + lon_threshold;
}

boost::optional<PoseWithVelocityStamped> calcInterpolatedPoseWithVelocity(
  const std::vector<PoseWithVelocityStamped> & path, const double relative_time)
{
  // Check if relative time is in the valid range
  if (path.empty() || relative_time < 0.0) {
    return boost::none;
  }

  constexpr double epsilon = 1e-6;
  for (size_t path_idx = 1; path_idx < path.size(); ++path_idx) {
    const auto & pt = path.at(path_idx);
    const auto & prev_pt = path.at(path_idx - 1);
    if (relative_time < pt.time + epsilon) {
      const double offset = relative_time - prev_pt.time;
      const double time_step = pt.time - prev_pt.time;
      const double ratio = std::clamp(offset / time_step, 0.0, 1.0);
      const auto interpolated_pose =
        tier4_autoware_utils::calcInterpolatedPose(prev_pt.pose, pt.pose, ratio, false);
      const double interpolated_velocity =
        interpolation::lerp(prev_pt.velocity, pt.velocity, ratio);
      return PoseWithVelocityStamped{relative_time, interpolated_pose, interpolated_velocity};
    }
  }

  return boost::none;
}

boost::optional<PoseWithVelocityAndPolygonStamped> getInterpolatedPoseWithVelocityAndPolygonStamped(
  const std::vector<PoseWithVelocityStamped> & pred_path, const double current_time,
  const VehicleInfo & ego_info)
{
  const auto interpolation_result = calcInterpolatedPoseWithVelocity(pred_path, current_time);

  if (!interpolation_result) {
    return {};
  }

  const auto & i = ego_info;
  const auto & base_to_front = i.max_longitudinal_offset_m;
  const auto & base_to_rear = i.rear_overhang_m;
  const auto & width = i.vehicle_width_m;
  const auto & pose = interpolation_result->pose;
  const auto & velocity = interpolation_result->velocity;

  const auto ego_polygon =
    tier4_autoware_utils::toFootprint(pose, base_to_front, base_to_rear, width);

  return PoseWithVelocityAndPolygonStamped{current_time, pose, velocity, ego_polygon};
}

bool checkCollision(
  [[maybe_unused]] const PathWithLaneId & planned_path,
  const std::vector<PoseWithVelocityStamped> & predicted_ego_path,
  const ExtendedPredictedObject & target_object,
  const PredictedPathWithPolygon & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const RSSparams & rss_parameters,
  double hysteresis_factor, CollisionCheckDebug & debug)
{
  {
    debug.ego_predicted_path = predicted_ego_path;
    debug.obj_predicted_path = target_object_path.path;
    debug.current_obj_pose = target_object.initial_pose.pose;
  }

  for (const auto & obj_pose_with_poly : target_object_path.path) {
    const auto & current_time = obj_pose_with_poly.time;

    // get object information at current time
    const auto & obj_pose = obj_pose_with_poly.pose;
    const auto & obj_polygon = obj_pose_with_poly.poly;
    const auto & object_velocity = obj_pose_with_poly.velocity;

    // get ego information at current time
    // Note: we can create these polygons in advance. However, it can decrease the readability and
    // variability
    const auto & ego_vehicle_info = common_parameters.vehicle_info;
    const auto interpolated_data = getInterpolatedPoseWithVelocityAndPolygonStamped(
      predicted_ego_path, current_time, ego_vehicle_info);
    if (!interpolated_data) {
      continue;
    }
    const auto & ego_pose = interpolated_data->pose;
    const auto & ego_polygon = interpolated_data->poly;
    const auto & ego_velocity = interpolated_data->velocity;

    {
      debug.expected_ego_pose = ego_pose;
      debug.expected_obj_pose = obj_pose;
      debug.extended_ego_polygon = ego_polygon;
      debug.extended_obj_polygon = obj_polygon;
    }

    // check overlap
    if (boost::geometry::overlaps(ego_polygon, obj_polygon)) {
      debug.unsafe_reason = "overlap_polygon";
      return false;
    }

    // compute which one is at the front of the other
    const bool is_object_front = isTargetObjectFront(ego_pose, obj_polygon, ego_vehicle_info);
    const auto & [front_object_velocity, rear_object_velocity] =
      is_object_front ? std::make_pair(object_velocity, ego_velocity)
                      : std::make_pair(ego_velocity, object_velocity);

    // compute rss dist
    const auto rss_dist =
      calcRssDistance(front_object_velocity, rear_object_velocity, rss_parameters);

    // minimum longitudinal length
    const auto min_lon_length =
      calcMinimumLongitudinalLength(front_object_velocity, rear_object_velocity, rss_parameters);

    const auto & lon_offset = std::max(rss_dist, min_lon_length) * hysteresis_factor;
    const auto & lat_margin = rss_parameters.lateral_distance_max_threshold * hysteresis_factor;
    const auto & extended_ego_polygon =
      is_object_front
        ? createExtendedPolygon(ego_pose, ego_vehicle_info, lon_offset, lat_margin, debug)
        : ego_polygon;
    const auto & extended_obj_polygon =
      is_object_front
        ? obj_polygon
        : createExtendedPolygon(obj_pose, target_object.shape, lon_offset, lat_margin, debug);

    {
      debug.rss_longitudinal = rss_dist;
      debug.inter_vehicle_distance = min_lon_length;
      debug.extended_ego_polygon = extended_ego_polygon;
      debug.extended_obj_polygon = extended_obj_polygon;
      debug.is_front = is_object_front;
    }

    // check overlap with extended polygon
    if (boost::geometry::overlaps(extended_ego_polygon, extended_obj_polygon)) {
      debug.unsafe_reason = "overlap_extended_polygon";
      return false;
    }
  }

  return true;
}

bool checkCollisionWithExtraStoppingMargin(
  const PathWithLaneId & ego_path, const PredictedObjects & dynamic_objects,
  const double base_to_front, const double base_to_rear, const double width,
  const double maximum_deceleration, const double collision_check_margin,
  const double max_extra_stopping_margin)
{
  for (const auto & p : ego_path.points) {
    const double extra_stopping_margin = std::min(
      std::pow(p.point.longitudinal_velocity_mps, 2) * 0.5 / maximum_deceleration,
      max_extra_stopping_margin);

    const auto ego_polygon = tier4_autoware_utils::toFootprint(
      p.point.pose, base_to_front + extra_stopping_margin, base_to_rear, width);

    for (const auto & object : dynamic_objects.objects) {
      const auto obj_polygon = tier4_autoware_utils::toPolygon2d(object);
      const double distance = boost::geometry::distance(obj_polygon, ego_polygon);
      if (distance < collision_check_margin) return true;
    }
  }

  return false;
}
}  // namespace behavior_path_planner::utils::path_safety_checker
