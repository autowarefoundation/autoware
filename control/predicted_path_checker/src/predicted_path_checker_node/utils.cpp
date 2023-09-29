// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#include "predicted_path_checker/utils.hpp"

#include <boost/format.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>

namespace utils
{

using motion_utils::findFirstNearestIndexWithSoftConstraints;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getRPY;

// Utils Functions
Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double expand_width)
{
  Polygon2d polygon;

  const double longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0 + expand_width;
  const double rear_overhang = vehicle_info.rear_overhang_m;

  {  // base step
    appendPointToPolygon(
      polygon, tier4_autoware_utils::calcOffsetPose(base_step_pose, longitudinal_offset, width, 0.0)
                 .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(base_step_pose, longitudinal_offset, -width, 0.0)
        .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(base_step_pose, -rear_overhang, -width, 0.0).position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(base_step_pose, -rear_overhang, width, 0.0).position);
  }

  {  // next step
    appendPointToPolygon(
      polygon, tier4_autoware_utils::calcOffsetPose(next_step_pose, longitudinal_offset, width, 0.0)
                 .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(next_step_pose, longitudinal_offset, -width, 0.0)
        .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(next_step_pose, -rear_overhang, -width, 0.0).position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(next_step_pose, -rear_overhang, width, 0.0).position);
  }

  polygon = tier4_autoware_utils::isClockwise(polygon)
              ? polygon
              : tier4_autoware_utils::inverseClockwise(polygon);

  Polygon2d hull_polygon;
  boost::geometry::convex_hull(polygon, hull_polygon);
  boost::geometry::correct(hull_polygon);

  return hull_polygon;
}

TrajectoryPoint calcInterpolatedPoint(
  const TrajectoryPoints & trajectory, const geometry_msgs::msg::Point & target_point,
  const size_t segment_idx, const bool use_zero_order_hold_for_twist)
{
  if (trajectory.empty()) {
    TrajectoryPoint interpolated_point{};
    interpolated_point.pose.position = target_point;
    return interpolated_point;
  } else if (trajectory.size() == 1) {
    return trajectory.front();
  }

  // Calculate interpolation ratio
  const auto & curr_pt = trajectory.at(segment_idx);
  const auto & next_pt = trajectory.at(segment_idx + 1);
  const auto v1 = tier4_autoware_utils::point2tfVector(curr_pt, next_pt);
  const auto v2 = tier4_autoware_utils::point2tfVector(curr_pt, target_point);
  if (v1.length2() < 1e-3) {
    return curr_pt;
  }

  const double ratio = v1.dot(v2) / v1.length2();
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  // Interpolate
  TrajectoryPoint interpolated_point{};

  // pose interpolation
  interpolated_point.pose =
    tier4_autoware_utils::calcInterpolatedPose(curr_pt, next_pt, clamped_ratio);

  // twist interpolation
  if (use_zero_order_hold_for_twist) {
    interpolated_point.longitudinal_velocity_mps = curr_pt.longitudinal_velocity_mps;
    interpolated_point.lateral_velocity_mps = curr_pt.lateral_velocity_mps;
    interpolated_point.acceleration_mps2 = curr_pt.acceleration_mps2;
  } else {
    interpolated_point.longitudinal_velocity_mps = interpolation::lerp(
      curr_pt.longitudinal_velocity_mps, next_pt.longitudinal_velocity_mps, clamped_ratio);
    interpolated_point.lateral_velocity_mps = interpolation::lerp(
      curr_pt.lateral_velocity_mps, next_pt.lateral_velocity_mps, clamped_ratio);
    interpolated_point.acceleration_mps2 =
      interpolation::lerp(curr_pt.acceleration_mps2, next_pt.acceleration_mps2, clamped_ratio);
  }

  // heading rate interpolation
  interpolated_point.heading_rate_rps =
    interpolation::lerp(curr_pt.heading_rate_rps, next_pt.heading_rate_rps, clamped_ratio);

  // wheel interpolation
  interpolated_point.front_wheel_angle_rad = interpolation::lerp(
    curr_pt.front_wheel_angle_rad, next_pt.front_wheel_angle_rad, clamped_ratio);
  interpolated_point.rear_wheel_angle_rad =
    interpolation::lerp(curr_pt.rear_wheel_angle_rad, next_pt.rear_wheel_angle_rad, clamped_ratio);

  // time interpolation
  const double interpolated_time = interpolation::lerp(
    rclcpp::Duration(curr_pt.time_from_start).seconds(),
    rclcpp::Duration(next_pt.time_from_start).seconds(), clamped_ratio);
  interpolated_point.time_from_start = rclcpp::Duration::from_seconds(interpolated_time);

  return interpolated_point;
}

std::pair<size_t, TrajectoryPoint> findStopPoint(
  TrajectoryPoints & trajectory_array, const size_t collision_idx, const double stop_margin,
  vehicle_info_util::VehicleInfo & vehicle_info)
{
  // It returns the stop point and segment of the point on trajectory.

  double desired_distance_base_link_to_collision =
    vehicle_info.max_longitudinal_offset_m + stop_margin;
  size_t stop_segment_idx = 0;
  bool found_stop_point = false;
  double distance_point_to_collision = 0.0;

  for (size_t i = collision_idx; i > 0; i--) {
    distance_point_to_collision =
      motion_utils::calcSignedArcLength(trajectory_array, i - 1, collision_idx);
    if (distance_point_to_collision >= desired_distance_base_link_to_collision) {
      stop_segment_idx = i - 1;
      found_stop_point = true;
      break;
    }
  }
  if (found_stop_point) {
    const auto & base_point = trajectory_array.at(stop_segment_idx);
    const auto & next_point = trajectory_array.at(stop_segment_idx + 1);

    double ratio = (distance_point_to_collision - desired_distance_base_link_to_collision) /
                   (std::hypot(
                     base_point.pose.position.x - next_point.pose.position.x,
                     base_point.pose.position.y - next_point.pose.position.y));

    geometry_msgs::msg::Pose interpolated_pose =
      tier4_autoware_utils::calcInterpolatedPose(base_point.pose, next_point.pose, ratio, false);
    TrajectoryPoint output;
    output.set__pose(interpolated_pose);
    return std::make_pair(stop_segment_idx, output);
  } else {
    // It means that there is no enough distance between vehicle and collision point.
    // So, we insert a stop at the first point of the trajectory.
    return std::make_pair(0, trajectory_array.front());
  }
}

bool isInBrakeDistance(
  const TrajectoryPoints & trajectory, const size_t stop_idx, const double relative_velocity,
  const double relative_acceleration, const double max_deceleration, const double delay_time_sec)
{
  if (relative_velocity < 0.0) {
    return false;
  }

  const auto distance_to_obstacle = motion_utils::calcSignedArcLength(
    trajectory, trajectory.front().pose.position, trajectory.at(stop_idx).pose.position);

  const double distance_in_delay = relative_velocity * delay_time_sec +
                                   relative_acceleration * delay_time_sec * delay_time_sec * 0.5;

  const double velocity_after_delay = relative_velocity + relative_acceleration * delay_time_sec;

  const double time_to_stop = velocity_after_delay / std::abs(max_deceleration);
  const double distance_after_delay =
    velocity_after_delay * time_to_stop - 0.5 * abs(max_deceleration) * time_to_stop * time_to_stop;
  const double brake_distance = distance_in_delay + distance_after_delay;

  return brake_distance > distance_to_obstacle;
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point)
{
  tf2::Transform map2goal;
  tf2::fromMsg(goal_point.pose, map2goal);
  tf2::Transform local_extend_point;
  local_extend_point.setOrigin(tf2::Vector3(extend_distance, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  local_extend_point.setRotation(q);
  const auto map2extend_point = map2goal * local_extend_point;
  Pose extend_pose;
  tf2::toMsg(map2extend_point, extend_pose);
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

bool intersectsInZAxis(const PredictedObject & object, const double z_min, const double z_max)
{
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance;
  const auto & obj_height = object.shape.dimensions.z;
  return obj_pose.pose.position.z - obj_height / 2.0 <= z_max &&
         obj_pose.pose.position.z + obj_height / 2.0 >= z_min;
}

double getNearestPointAndDistanceForPredictedObject(
  const PointArray & points, const Pose & base_pose,
  geometry_msgs::msg::Point * nearest_collision_point)
{
  double min_norm = 0.0;
  bool is_init = false;

  for (const auto & p : points) {
    double norm = tier4_autoware_utils::calcDistance2d(p, base_pose);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = p;
      is_init = true;
    }
  }
  return min_norm;
}

Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & base_to_front, const double & base_to_rear,
  const double & base_to_width)
{
  const auto mapped_point = [](const double & length_scalar, const double & width_scalar) {
    tf2::Vector3 map;
    map.setX(length_scalar);
    map.setY(width_scalar);
    map.setZ(0.0);
    map.setW(1.0);
    return map;
  };

  // set vertices at map coordinate
  const tf2::Vector3 p1_map = std::invoke(mapped_point, base_to_front, -base_to_width);
  const tf2::Vector3 p2_map = std::invoke(mapped_point, base_to_front, base_to_width);
  const tf2::Vector3 p3_map = std::invoke(mapped_point, -base_to_rear, base_to_width);
  const tf2::Vector3 p4_map = std::invoke(mapped_point, -base_to_rear, -base_to_width);

  // transform vertices from map coordinate to object coordinate
  tf2::Transform tf_map2obj;
  tf2::fromMsg(current_pose, tf_map2obj);
  const tf2::Vector3 p1_obj = tf_map2obj * p1_map;
  const tf2::Vector3 p2_obj = tf_map2obj * p2_map;
  const tf2::Vector3 p3_obj = tf_map2obj * p3_map;
  const tf2::Vector3 p4_obj = tf_map2obj * p4_map;

  Polygon2d object_polygon;
  object_polygon.outer().reserve(5);
  object_polygon.outer().emplace_back(p1_obj.x(), p1_obj.y());
  object_polygon.outer().emplace_back(p2_obj.x(), p2_obj.y());
  object_polygon.outer().emplace_back(p3_obj.x(), p3_obj.y());
  object_polygon.outer().emplace_back(p4_obj.x(), p4_obj.y());

  object_polygon.outer().push_back(object_polygon.outer().front());
  boost::geometry::correct(object_polygon);

  return object_polygon;
}

Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_auto_perception_msgs::msg::Shape & obj_shape)
{
  Polygon2d object_polygon;

  const double obj_x = current_pose.position.x;
  const double obj_y = current_pose.position.y;

  constexpr int N = 20;
  const double r = obj_shape.dimensions.x / 2;
  object_polygon.outer().reserve(N + 1);
  for (int i = 0; i < N; ++i) {
    object_polygon.outer().emplace_back(
      obj_x + r * std::cos(2.0 * M_PI / N * i), obj_y + r * std::sin(2.0 * M_PI / N * i));
  }

  object_polygon.outer().push_back(object_polygon.outer().front());
  boost::geometry::correct(object_polygon);

  return object_polygon;
}

Polygon2d convertPolygonObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_auto_perception_msgs::msg::Shape & obj_shape)
{
  Polygon2d object_polygon;
  tf2::Transform tf_map2obj;
  fromMsg(current_pose, tf_map2obj);
  const auto obj_points = obj_shape.footprint.points;
  object_polygon.outer().reserve(obj_points.size() + 1);
  for (const auto & obj_point : obj_points) {
    tf2::Vector3 obj(obj_point.x, obj_point.y, obj_point.z);
    tf2::Vector3 tf_obj = tf_map2obj * obj;
    object_polygon.outer().emplace_back(tf_obj.x(), tf_obj.y());
  }
  object_polygon.outer().push_back(object_polygon.outer().front());
  boost::geometry::correct(object_polygon);

  return object_polygon;
}

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  boost::geometry::append(polygon.outer(), point);
}

Polygon2d convertObjToPolygon(const PredictedObject & obj)
{
  Polygon2d object_polygon{};
  if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    object_polygon = utils::convertCylindricalObjectToGeometryPolygon(
      obj.kinematics.initial_pose_with_covariance.pose, obj.shape);
  } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const double & length_m = obj.shape.dimensions.x / 2;
    const double & width_m = obj.shape.dimensions.y / 2;
    object_polygon = utils::convertBoundingBoxObjectToGeometryPolygon(
      obj.kinematics.initial_pose_with_covariance.pose, length_m, length_m, width_m);
  } else if (obj.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    object_polygon = utils::convertPolygonObjectToGeometryPolygon(
      obj.kinematics.initial_pose_with_covariance.pose, obj.shape);
  } else {
    throw std::runtime_error("Unsupported shape type");
  }
  return object_polygon;
}

bool isFrontObstacle(const Pose & ego_pose, const geometry_msgs::msg::Point & obstacle_pos)
{
  const auto yaw = tier4_autoware_utils::getRPY(ego_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));
  const Eigen::Vector2d obstacle_vec(
    obstacle_pos.x - ego_pose.position.x, obstacle_pos.y - ego_pose.position.y);

  return base_pose_vec.dot(obstacle_vec) >= 0;
}

double calcObstacleMaxLength(const autoware_auto_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return std::hypot(shape.dimensions.x / 2.0, shape.dimensions.y / 2.0);
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  } else if (shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    double max_length_to_point = 0.0;
    for (const auto rel_point : shape.footprint.points) {
      const double length_to_point = std::hypot(rel_point.x, rel_point.y);
      if (max_length_to_point < length_to_point) {
        max_length_to_point = length_to_point;
      }
    }
    return max_length_to_point;
  }

  throw std::logic_error("The shape type is not supported in obstacle_cruise_planner.");
}

void getCurrentObjectPose(
  PredictedObject & predicted_object, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time)
{
  const double yaw = tier4_autoware_utils::getRPY(
                       predicted_object.kinematics.initial_pose_with_covariance.pose.orientation)
                       .z;
  const double vx = predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const double ax = predicted_object.kinematics.initial_acceleration_with_covariance.accel.linear.x;

  const double dt = (current_time - obj_base_time).seconds();
  const double ds = vx * dt + 0.5 * ax * dt * dt;
  const double delta_yaw =
    predicted_object.kinematics.initial_twist_with_covariance.twist.angular.z * dt;
  geometry_msgs::msg::Transform transform;
  transform.translation = tier4_autoware_utils::createTranslation(ds, 0.0, 0.0);
  transform.rotation = tier4_autoware_utils::createQuaternionFromRPY(0.0, 0.0, yaw);

  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(predicted_object.kinematics.initial_pose_with_covariance.pose, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, predicted_object.kinematics.initial_pose_with_covariance.pose);
  predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x += ax * dt;
  predicted_object.kinematics.initial_pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(
      0.0, 0.0, tier4_autoware_utils::normalizeRadian(yaw + delta_yaw));
}

}  // namespace utils
