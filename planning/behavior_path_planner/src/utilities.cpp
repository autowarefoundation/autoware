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

#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"

#include <boost/assign/list_of.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
double calcInterpolatedZ(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
  const geometry_msgs::msg::Point target_pos, const size_t seg_idx)
{
  const double closest_to_target_dist = motion_utils::calcSignedArcLength(
    input.points, input.points.at(seg_idx).point.pose.position,
    target_pos);  // TODO(murooka) implement calcSignedArcLength(points, idx, point)
  const double seg_dist = motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_z = input.points.at(seg_idx).point.pose.position.z;
  const double next_z = input.points.at(seg_idx + 1).point.pose.position.z;
  const double interpolated_z =
    std::abs(seg_dist) < 1e-6
      ? next_z
      : closest_z + (next_z - closest_z) * closest_to_target_dist / seg_dist;
  return interpolated_z;
}

double calcInterpolatedVelocity(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const size_t seg_idx)
{
  const double seg_dist = motion_utils::calcSignedArcLength(input.points, seg_idx, seg_idx + 1);

  const double closest_vel = input.points.at(seg_idx).point.longitudinal_velocity_mps;
  const double next_vel = input.points.at(seg_idx + 1).point.longitudinal_velocity_mps;
  const double interpolated_vel = std::abs(seg_dist) < 1e-06 ? next_vel : closest_vel;
  return interpolated_vel;
}

template <class T>
size_t findNearestSegmentIndex(
  const std::vector<T> & points, const geometry_msgs::msg::Pose & pose, const double dist_threshold,
  const double yaw_threshold)
{
  const auto nearest_idx =
    motion_utils::findNearestSegmentIndex(points, pose, dist_threshold, yaw_threshold);
  if (nearest_idx) {
    return nearest_idx.get();
  }

  return motion_utils::findNearestSegmentIndex(points, pose.position);
}

template <class T>
size_t findNearestSegmentIndexFromLateralDistance(
  const std::vector<T> & points, const geometry_msgs::msg::Point & target_point)
{
  size_t closest_idx = motion_utils::findNearestSegmentIndex(points, target_point);
  double min_lateral_dist =
    std::fabs(motion_utils::calcLateralOffset(points, target_point, closest_idx));

  for (size_t seg_idx = 0; seg_idx < points.size() - 1; ++seg_idx) {
    const double lon_dist =
      motion_utils::calcLongitudinalOffsetToSegment(points, seg_idx, target_point);
    const double segment_length =
      tier4_autoware_utils::calcDistance2d(points.at(seg_idx), points.at(seg_idx + 1));
    if (lon_dist < 0.0 || segment_length < lon_dist) {
      continue;
    }

    const double lat_dist =
      std::fabs(motion_utils::calcLateralOffset(points, target_point, seg_idx));
    if (lat_dist < min_lateral_dist) {
      closest_idx = seg_idx;
      min_lateral_dist = lat_dist;
    }
  }

  return closest_idx;
}

bool checkHasSameLane(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelet & target_lane)
{
  if (lanelets.empty()) return false;

  const auto has_same = [&](const auto & ll) { return ll.id() == target_lane.id(); };
  return std::find_if(lanelets.begin(), lanelets.end(), has_same) != lanelets.end();
}
}  // namespace

namespace behavior_path_planner::util
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using tf2::fromMsg;
using tier4_autoware_utils::Point2d;

std::vector<Pose> convertToPoseArray(const PathWithLaneId & path)
{
  std::vector<Pose> pose_array;
  pose_array.reserve(path.points.size());
  for (const auto & pt : path.points) {
    pose_array.push_back(pt.point.pose);
  }
  return pose_array;
}

double l2Norm(const Vector3 vector)
{
  return std::sqrt(std::pow(vector.x, 2) + std::pow(vector.y, 2) + std::pow(vector.z, 2));
}

std::vector<Point> convertToGeometryPointArray(const PathWithLaneId & path)
{
  std::vector<Point> converted_path;
  converted_path.reserve(path.points.size());
  for (const auto & point_with_id : path.points) {
    converted_path.push_back(point_with_id.point.pose.position);
  }
  return converted_path;
}

std::vector<Point> convertToGeometryPointArray(const PredictedPath & path)
{
  std::vector<Point> converted_path;

  converted_path.reserve(path.path.size());
  for (const auto & pose : path.path) {
    converted_path.push_back(pose.position);
  }
  return converted_path;
}

PoseArray convertToGeometryPoseArray(const PathWithLaneId & path)
{
  PoseArray converted_array;
  converted_array.header = path.header;

  converted_array.poses.reserve(path.points.size());
  for (const auto & point_with_id : path.points) {
    converted_array.poses.push_back(point_with_id.point.pose);
  }
  return converted_array;
}

PredictedPath convertToPredictedPath(
  const PathWithLaneId & path, const Twist & vehicle_twist, const Pose & vehicle_pose,
  const double nearest_seg_idx, const double duration, const double resolution,
  const double acceleration, const double min_speed)
{
  PredictedPath predicted_path{};
  predicted_path.time_step = rclcpp::Duration::from_seconds(resolution);
  predicted_path.path.reserve(std::min(path.points.size(), static_cast<size_t>(100)));
  if (path.points.empty()) {
    return predicted_path;
  }

  FrenetCoordinate3d vehicle_pose_frenet =
    convertToFrenetCoordinate3d(path.points, vehicle_pose.position, nearest_seg_idx);
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  rclcpp::Time start_time = clock.now();
  double vehicle_speed = std::abs(vehicle_twist.linear.x);
  if (vehicle_speed < min_speed) {
    vehicle_speed = min_speed;
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"), clock, 1000,
      "cannot convert PathWithLaneId with zero velocity, using minimum value " << min_speed
                                                                               << " [m/s] instead");
  }

  double length = 0;
  double prev_vehicle_speed = vehicle_speed;

  // first point
  const auto pt = lerpByLength(path.points, vehicle_pose_frenet.length);
  Pose predicted_pose;
  predicted_pose.position = pt;
  predicted_path.path.push_back(predicted_pose);

  for (double t = resolution; t < duration; t += resolution) {
    double accelerated_velocity = prev_vehicle_speed + acceleration * t;
    double travel_distance = 0;
    if (accelerated_velocity < min_speed) {
      travel_distance = min_speed * resolution;
    } else {
      travel_distance =
        prev_vehicle_speed * resolution + 0.5 * acceleration * resolution * resolution;
    }

    length += travel_distance;
    const auto pt = lerpByLength(path.points, vehicle_pose_frenet.length + length);
    Pose predicted_pose;
    predicted_pose.position = pt;
    predicted_path.path.push_back(predicted_pose);
    prev_vehicle_speed = accelerated_velocity;
  }
  return predicted_path;
}

PredictedPath resamplePredictedPath(
  const PredictedPath & input_path, const double resolution, const double duration)
{
  PredictedPath resampled_path{};

  for (double t = 0.0; t < duration; t += resolution) {
    Pose pose;
    if (!lerpByTimeStamp(input_path, t, &pose)) {
      continue;
    }
    resampled_path.path.push_back(pose);
  }

  return resampled_path;
}

Pose lerpByPose(const Pose & p1, const Pose & p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto & tf_quaternion =
    tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  Pose pose{};
  pose.position = tf2::toMsg(tf_point, pose.position);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

bool lerpByTimeStamp(const PredictedPath & path, const double t_query, Pose * lerped_pt)
{
  const rclcpp::Duration time_step(path.time_step);
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  if (lerped_pt == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"), clock, 1000,
      "failed to lerp by time due to nullptr pt");
    return false;
  }
  if (path.path.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"), clock, 1000,
      "Empty path. Failed to interpolate path by time!");
    return false;
  }

  const double t_final = time_step.seconds() * static_cast<double>(path.path.size() - 1);
  if (t_query > t_final) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "failed to interpolate path by time!" << std::endl
                                            << "t_final    : " << t_final
                                            << "query time : " << t_query);
    *lerped_pt = path.path.back();

    return false;
  }

  for (size_t i = 1; i < path.path.size(); i++) {
    const auto & pt = path.path.at(i);
    const auto & prev_pt = path.path.at(i - 1);
    const double t = time_step.seconds() * static_cast<double>(i);
    if (t_query <= t) {
      const double prev_t = time_step.seconds() * static_cast<double>(i - 1);
      const double duration = time_step.seconds();
      const double offset = t_query - prev_t;
      const double ratio = offset / duration;
      *lerped_pt = lerpByPose(prev_pt, pt, ratio);
      return true;
    }
  }

  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
    "Something failed in function: ");
  return false;
}

bool lerpByTimeStamp(
  const PredictedPath & path, const double t_query, Pose * lerped_pt, std::string & failed_reason)
{
  const rclcpp::Duration time_step(path.time_step);
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  if (lerped_pt == nullptr) {
    failed_reason = "nullptr_pt";
    return false;
  }
  if (path.path.empty()) {
    failed_reason = "empty_path";
    return false;
  }

  const double t_final = time_step.seconds() * static_cast<double>(path.path.size() - 1);
  if (t_query > t_final) {
    failed_reason = "query_exceed_t_final";
    *lerped_pt = path.path.back();

    return false;
  }

  for (size_t i = 1; i < path.path.size(); i++) {
    const auto & pt = path.path.at(i);
    const auto & prev_pt = path.path.at(i - 1);
    const double t = time_step.seconds() * static_cast<double>(i);
    if (t_query <= t) {
      const double prev_t = time_step.seconds() * static_cast<double>(i - 1);
      const double duration = time_step.seconds();
      const double offset = t_query - prev_t;
      const double ratio = offset / duration;
      *lerped_pt = lerpByPose(prev_pt, pt, ratio);
      return true;
    }
  }

  failed_reason = "unknown_failure";
  return false;
}

double getDistanceBetweenPredictedPaths(
  const PredictedPath & object_path, const PredictedPath & ego_path, const double start_time,
  const double end_time, const double resolution)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  for (double t = start_time; t < end_time; t += resolution) {
    Pose object_pose, ego_pose;
    if (!lerpByTimeStamp(object_path, t, &object_pose)) {
      continue;
    }
    if (!lerpByTimeStamp(ego_path, t, &ego_pose)) {
      continue;
    }
    double distance = tier4_autoware_utils::calcDistance3d(object_pose, ego_pose);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

double getDistanceBetweenPredictedPathAndObject(
  const PredictedObject & object, const PredictedPath & ego_path, const double start_time,
  const double end_time, const double resolution)
{
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  auto t_delta{rclcpp::Duration::from_seconds(resolution)};
  double min_distance = std::numeric_limits<double>::max();
  rclcpp::Time ros_start_time = clock.now() + rclcpp::Duration::from_seconds(start_time);
  rclcpp::Time ros_end_time = clock.now() + rclcpp::Duration::from_seconds(end_time);
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  Polygon2d obj_polygon;
  if (!calcObjectPolygon(object, &obj_polygon)) {
    return min_distance;
  }
  for (double t = start_time; t < end_time; t += resolution) {
    Pose ego_pose;
    if (!lerpByTimeStamp(ego_path, t, &ego_pose)) {
      continue;
    }
    Point2d ego_point{ego_pose.position.x, ego_pose.position.y};

    double distance = boost::geometry::distance(obj_polygon, ego_point);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

bool checkCollisionBetweenPathFootprintsAndObjects(
  const tier4_autoware_utils::LinearRing2d & local_vehicle_footprint,
  const PathWithLaneId & ego_path, const PredictedObjects & dynamic_objects, const double margin)
{
  for (const auto & p : ego_path.points) {
    if (checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, p.point.pose, dynamic_objects, margin)) {
      return true;
    }
  }
  return false;
}

bool checkCollisionBetweenFootprintAndObjects(
  const tier4_autoware_utils::LinearRing2d & local_vehicle_footprint, const Pose & ego_pose,
  const PredictedObjects & dynamic_objects, const double margin)
{
  const auto vehicle_footprint =
    transformVector(local_vehicle_footprint, tier4_autoware_utils::pose2transform(ego_pose));

  for (const auto & object : dynamic_objects.objects) {
    Polygon2d obj_polygon;
    if (!calcObjectPolygon(object, &obj_polygon)) {
      continue;
    }

    const double distance = boost::geometry::distance(obj_polygon, vehicle_footprint);
    if (distance < margin) return true;
  }
  return false;
}

double calcLateralDistanceFromEgoToObject(
  const Pose & ego_pose, const double vehicle_width, const PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  Polygon2d obj_polygon;
  if (!calcObjectPolygon(dynamic_object, &obj_polygon)) {
    return min_distance;
  }

  const auto vehicle_left_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, 0, vehicle_width / 2, 0);
  const auto vehicle_right_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, 0, -vehicle_width / 2, 0);

  for (const auto & p : obj_polygon.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    // left direction is positive
    const double signed_distance_from_left =
      tier4_autoware_utils::calcLateralDeviation(vehicle_left_pose, point);
    // right direction is positive
    const double signed_distance_from_right =
      tier4_autoware_utils::calcLateralDeviation(vehicle_right_pose, point);

    if (signed_distance_from_left < 0.0 && signed_distance_from_right < 0.0) {
      // point is between left and right
      return 0.0;
    }

    const double distance_from_ego =
      std::min(std::abs(signed_distance_from_left), std::abs(signed_distance_from_right));
    min_distance = std::min(min_distance, distance_from_ego);
  }
  return min_distance;
}

double calcLongitudinalDistanceFromEgoToObject(
  const Pose & ego_pose, const double base_link2front, const double base_link2rear,
  const PredictedObject & dynamic_object)
{
  double min_distance = std::numeric_limits<double>::max();
  Polygon2d obj_polygon;
  if (!calcObjectPolygon(dynamic_object, &obj_polygon)) {
    return min_distance;
  }

  const auto vehicle_front_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, base_link2front, 0, 0);
  const auto vehicle_rear_pose =
    tier4_autoware_utils::calcOffsetPose(ego_pose, base_link2rear, 0, 0);

  for (const auto & p : obj_polygon.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);

    // forward is positive
    const double signed_distance_from_front =
      tier4_autoware_utils::calcLongitudinalDeviation(vehicle_front_pose, point);
    // backward is positive
    const double signed_distance_from_rear =
      -tier4_autoware_utils::calcLongitudinalDeviation(vehicle_rear_pose, point);

    if (signed_distance_from_front < 0.0 && signed_distance_from_rear < 0.0) {
      // point is between front and rear
      return 0.0;
    }

    const double distance_from_ego =
      std::min(std::abs(signed_distance_from_front), std::abs(signed_distance_from_rear));
    min_distance = std::min(min_distance, distance_from_ego);
  }
  return min_distance;
}

double calcLongitudinalDistanceFromEgoToObjects(
  const Pose & ego_pose, double base_link2front, double base_link2rear,
  const PredictedObjects & dynamic_objects)
{
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & object : dynamic_objects.objects) {
    min_distance = std::min(
      min_distance,
      calcLongitudinalDistanceFromEgoToObject(ego_pose, base_link2front, base_link2rear, object));
  }
  return min_distance;
}

// only works with consecutive lanes
std::vector<size_t> filterObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets,
  const double start_arc_length, const double end_arc_length)
{
  std::vector<size_t> indices;
  if (target_lanelets.empty()) {
    return {};
  }
  const auto polygon =
    lanelet::utils::getPolygonFromArcLength(target_lanelets, start_arc_length, end_arc_length);
  const auto polygon2d = lanelet::utils::to2D(polygon).basicPolygon();
  if (polygon2d.empty()) {
    // no lanelet polygon
    return {};
  }

  for (size_t i = 0; i < objects.objects.size(); i++) {
    const auto & obj = objects.objects.at(i);
    // create object polygon
    Polygon2d obj_polygon;
    if (!calcObjectPolygon(obj, &obj_polygon)) {
      continue;
    }
    // create lanelet polygon
    Polygon2d lanelet_polygon;
    lanelet_polygon.outer().reserve(polygon2d.size() + 1);
    for (const auto & lanelet_point : polygon2d) {
      lanelet_polygon.outer().emplace_back(lanelet_point.x(), lanelet_point.y());
    }

    lanelet_polygon.outer().push_back(lanelet_polygon.outer().front());

    // check the object does not intersect the lanelet
    if (!boost::geometry::disjoint(lanelet_polygon, obj_polygon)) {
      indices.push_back(i);
      continue;
    }
  }
  return indices;
}

// works with random lanelets
std::vector<size_t> filterObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets)
{
  std::vector<size_t> indices;
  if (target_lanelets.empty()) {
    return {};
  }

  for (size_t i = 0; i < objects.objects.size(); i++) {
    // create object polygon
    const auto & obj = objects.objects.at(i);
    // create object polygon
    Polygon2d obj_polygon;
    if (!calcObjectPolygon(obj, &obj_polygon)) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
        "Failed to calcObjectPolygon...!!!");
      continue;
    }

    for (const auto & llt : target_lanelets) {
      // create lanelet polygon
      const auto polygon2d = llt.polygon2d().basicPolygon();
      if (polygon2d.empty()) {
        // no lanelet polygon
        continue;
      }
      Polygon2d lanelet_polygon;
      lanelet_polygon.outer().reserve(polygon2d.size() + 1);
      for (const auto & lanelet_point : polygon2d) {
        lanelet_polygon.outer().emplace_back(lanelet_point.x(), lanelet_point.y());
      }

      lanelet_polygon.outer().push_back(lanelet_polygon.outer().front());

      // check the object does not intersect the lanelet
      if (!boost::geometry::disjoint(lanelet_polygon, obj_polygon)) {
        indices.push_back(i);
        break;
      }
    }
  }
  return indices;
}

PredictedObjects filterObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets)
{
  PredictedObjects filtered_objects;
  const auto indices = filterObjectIndicesByLanelets(objects, target_lanelets);
  filtered_objects.objects.reserve(indices.size());
  for (const size_t i : indices) {
    filtered_objects.objects.push_back(objects.objects.at(i));
  }
  return filtered_objects;
}

bool calcObjectPolygon(const PredictedObject & object, Polygon2d * object_polygon)
{
  if (object.shape.type == Shape::BOUNDING_BOX) {
    const double & length_m = object.shape.dimensions.x / 2;
    const double & width_m = object.shape.dimensions.y / 2;
    *object_polygon = convertBoundingBoxObjectToGeometryPolygon(
      object.kinematics.initial_pose_with_covariance.pose, length_m, length_m, width_m);

  } else if (object.shape.type == Shape::CYLINDER) {
    *object_polygon = convertCylindricalObjectToGeometryPolygon(
      object.kinematics.initial_pose_with_covariance.pose, object.shape);
  } else if (object.shape.type == Shape::POLYGON) {
    *object_polygon = convertPolygonObjectToGeometryPolygon(
      object.kinematics.initial_pose_with_covariance.pose, object.shape);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"), "Object shape unknown!");
    return false;
  }

  return true;
}

bool calcObjectPolygon(
  const Shape & object_shape, const Pose & object_pose, Polygon2d * object_polygon)
{
  if (object_shape.type == Shape::BOUNDING_BOX) {
    const double & length_m = object_shape.dimensions.x / 2;
    const double & width_m = object_shape.dimensions.y / 2;
    *object_polygon =
      convertBoundingBoxObjectToGeometryPolygon(object_pose, length_m, length_m, width_m);

  } else if (object_shape.type == Shape::CYLINDER) {
    *object_polygon = convertCylindricalObjectToGeometryPolygon(object_pose, object_shape);
  } else if (object_shape.type == Shape::POLYGON) {
    *object_polygon = convertPolygonObjectToGeometryPolygon(object_pose, object_shape);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"), "Object shape unknown!");
    return false;
  }

  return true;
}

std::vector<double> calcObjectsDistanceToPath(
  const PredictedObjects & objects, const PathWithLaneId & ego_path)
{
  std::vector<double> distance_array;
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  for (const auto & obj : objects.objects) {
    Polygon2d obj_polygon;
    if (!calcObjectPolygon(obj, &obj_polygon)) {
      std::cerr << __func__ << ": fail to convert object to polygon" << std::endl;
      continue;
    }
    LineString2d ego_path_line;
    ego_path_line.reserve(ego_path_point_array.size());
    for (const auto & ego_path_point : ego_path_point_array) {
      boost::geometry::append(ego_path_line, Point2d(ego_path_point.x, ego_path_point.y));
    }
    const double distance = boost::geometry::distance(obj_polygon, ego_path_line);
    distance_array.push_back(distance);
  }
  return distance_array;
}

std::vector<size_t> filterObjectsIndicesByPath(
  const PredictedObjects & objects, const std::vector<size_t> & object_indices,
  const PathWithLaneId & ego_path, const double vehicle_width)
{
  std::vector<size_t> indices;
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  for (const auto & i : object_indices) {
    Polygon2d obj_polygon;
    if (!calcObjectPolygon(objects.objects.at(i), &obj_polygon)) {
      continue;
    }
    LineString2d ego_path_line;
    ego_path_line.reserve(ego_path_point_array.size());
    for (const auto & ego_path_point : ego_path_point_array) {
      boost::geometry::append(ego_path_line, Point2d(ego_path_point.x, ego_path_point.y));
    }
    const double distance = boost::geometry::distance(obj_polygon, ego_path_line);
    if (distance < vehicle_width) {
      indices.push_back(i);
    }
  }
  return indices;
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered_path;
  for (const auto & pt : input_path.points) {
    if (filtered_path.points.empty()) {
      filtered_path.points.push_back(pt);
      continue;
    }

    constexpr double min_dist = 0.001;
    if (
      tier4_autoware_utils::calcDistance3d(filtered_path.points.back().point, pt.point) <
      min_dist) {
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
      filtered_path.points.back().point.longitudinal_velocity_mps = std::min(
        pt.point.longitudinal_velocity_mps,
        filtered_path.points.back().point.longitudinal_velocity_mps);
    } else {
      filtered_path.points.push_back(pt);
    }
  }
  filtered_path.left_bound = input_path.left_bound;
  filtered_path.right_bound = input_path.right_bound;
  return filtered_path;
}

template <typename T>
bool exists(std::vector<T> vec, T element)
{
  return std::find(vec.begin(), vec.end(), element) != vec.end();
}

boost::optional<size_t> findIndexOutOfGoalSearchRange(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const Pose & goal, const int64_t goal_lane_id,
  const double max_dist = std::numeric_limits<double>::max())
{
  if (points.empty()) {
    return boost::none;
  }

  // find goal index
  size_t min_dist_index;
  double min_dist = std::numeric_limits<double>::max();
  {
    bool found = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & lane_ids = points.at(i).lane_ids;

      const double dist_to_goal =
        tier4_autoware_utils::calcDistance2d(points.at(i).point.pose, goal);
      const bool is_goal_lane_id_in_point =
        std::find(lane_ids.begin(), lane_ids.end(), goal_lane_id) != lane_ids.end();
      if (dist_to_goal < max_dist && dist_to_goal < min_dist && is_goal_lane_id_in_point) {
        min_dist_index = i;
        min_dist = dist_to_goal;
        found = true;
      }
    }
    if (!found) {
      return boost::none;
    }
  }

  // find index out of goal search range
  size_t min_dist_out_of_range_index = min_dist_index;
  for (int i = min_dist_index; 0 <= i; --i) {
    const double dist = tier4_autoware_utils::calcDistance2d(points.at(i).point, goal);
    min_dist_out_of_range_index = i;
    if (max_dist < dist) {
      break;
    }
  }

  return min_dist_out_of_range_index;
}

// goal does not have z
bool setGoal(
  const double search_radius_range, [[maybe_unused]] const double search_rad_range,
  const PathWithLaneId & input, const Pose & goal, const int64_t goal_lane_id,
  PathWithLaneId * output_ptr)
{
  try {
    if (input.points.empty()) {
      return false;
    }

    // calculate refined_goal with interpolation
    // NOTE: goal does not have valid z, that will be calculated by interpolation here
    PathPointWithLaneId refined_goal{};
    const size_t closest_seg_idx_for_goal =
      findNearestSegmentIndex(input.points, goal, 3.0, M_PI_4);
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z =
      calcInterpolatedZ(input, goal.position, closest_seg_idx_for_goal);
    refined_goal.point.longitudinal_velocity_mps = 0.0;

    // calculate pre_refined_goal with interpolation
    // NOTE: z and velocity are filled
    PathPointWithLaneId pre_refined_goal{};
    constexpr double goal_to_pre_goal_distance = -1.0;
    pre_refined_goal.point.pose =
      tier4_autoware_utils::calcOffsetPose(goal, goal_to_pre_goal_distance, 0.0, 0.0);
    const size_t closest_seg_idx_for_pre_goal =
      findNearestSegmentIndex(input.points, pre_refined_goal.point.pose, 3.0, M_PI_4);
    pre_refined_goal.point.pose.position.z =
      calcInterpolatedZ(input, pre_refined_goal.point.pose.position, closest_seg_idx_for_pre_goal);
    pre_refined_goal.point.longitudinal_velocity_mps =
      calcInterpolatedVelocity(input, closest_seg_idx_for_pre_goal);

    // find min_dist_out_of_circle_index whose distance to goal is longer than search_radius_range
    const auto min_dist_out_of_circle_index_opt =
      findIndexOutOfGoalSearchRange(input.points, goal, goal_lane_id, search_radius_range);
    if (!min_dist_out_of_circle_index_opt) {
      return false;
    }
    const size_t min_dist_out_of_circle_index = min_dist_out_of_circle_index_opt.get();

    // create output points
    output_ptr->points.reserve(output_ptr->points.size() + min_dist_out_of_circle_index + 3);
    for (size_t i = 0; i <= min_dist_out_of_circle_index; ++i) {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(refined_goal);

    {  // fill skipped lane ids
      // pre refined goal
      auto & pre_goal = output_ptr->points.at(output_ptr->points.size() - 2);
      for (size_t i = min_dist_out_of_circle_index + 1; i < input.points.size(); ++i) {
        for (const auto target_lane_id : input.points.at(i).lane_ids) {
          const bool is_lane_id_found =
            std::find(pre_goal.lane_ids.begin(), pre_goal.lane_ids.end(), target_lane_id) !=
            pre_goal.lane_ids.end();
          if (!is_lane_id_found) {
            pre_goal.lane_ids.push_back(target_lane_id);
          }
        }
      }

      // goal
      output_ptr->points.back().lane_ids = input.points.back().lane_ids;
    }

    output_ptr->left_bound = input.left_bound;
    output_ptr->right_bound = input.right_bound;
    return true;
  } catch (std::out_of_range & ex) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "failed to set goal: " << ex.what());
    return false;
  }
}

const Pose refineGoal(const Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  // return goal;
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(lanelet_point), goal_lanelet.centerline());
  if (segment.empty()) {
    return goal;
  }

  Pose refined_goal;
  {
    // find position
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    refined_goal.position.x = refined_point.x();
    refined_goal.position.y = refined_point.y();
    refined_goal.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    refined_goal.orientation = tf2::toMsg(tf_quat);
  }
  return refined_goal;
}

PathWithLaneId refinePathForGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const Pose & goal, const int64_t goal_lane_id)
{
  PathWithLaneId filtered_path, path_with_goal;
  filtered_path = removeOverlappingPoints(input);

  // always set zero velocity at the end of path for safety
  if (!filtered_path.points.empty()) {
    filtered_path.points.back().point.longitudinal_velocity_mps = 0.0;
  }

  if (setGoal(
        search_radius_range, search_rad_range, filtered_path, goal, goal_lane_id,
        &path_with_goal)) {
    return path_with_goal;
  } else {
    return filtered_path;
  }
}

bool containsGoal(const lanelet::ConstLanelets & lanes, const lanelet::Id & goal_id)
{
  for (const auto & lane : lanes) {
    if (lane.id() == goal_id) {
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets transformToLanelets(const DrivableLanes & drivable_lanes)
{
  lanelet::ConstLanelets lanes;

  const auto has_same_lane = [&](const auto & lane) {
    if (lanes.empty()) return false;
    const auto has_same = [&](const auto & ll) { return ll.id() == lane.id(); };
    return std::find_if(lanes.begin(), lanes.end(), has_same) != lanes.end();
  };

  lanes.push_back(drivable_lanes.right_lane);
  if (!has_same_lane(drivable_lanes.left_lane)) {
    lanes.push_back(drivable_lanes.left_lane);
  }

  for (const auto & ml : drivable_lanes.middle_lanes) {
    if (!has_same_lane(ml)) {
      lanes.push_back(ml);
    }
  }

  return lanes;
}

lanelet::ConstLanelets transformToLanelets(const std::vector<DrivableLanes> & drivable_lanes)
{
  lanelet::ConstLanelets lanes;

  for (const auto & drivable_lane : drivable_lanes) {
    const auto transformed_lane = transformToLanelets(drivable_lane);
    lanes.insert(lanes.end(), transformed_lane.begin(), transformed_lane.end());
  }

  return lanes;
}

boost::optional<lanelet::ConstLanelet> getRightLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.leftBound().id() == current_lane.rightBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

boost::optional<lanelet::ConstLanelet> getLeftLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes)
{
  for (const auto & shoulder_lane : shoulder_lanes) {
    if (shoulder_lane.rightBound().id() == current_lane.leftBound().id()) {
      return shoulder_lane;
    }
  }

  return {};
}

std::vector<DrivableLanes> generateDrivableLanes(const lanelet::ConstLanelets & lanes)
{
  std::vector<DrivableLanes> drivable_lanes(lanes.size());
  for (size_t i = 0; i < lanes.size(); ++i) {
    drivable_lanes.at(i).left_lane = lanes.at(i);
    drivable_lanes.at(i).right_lane = lanes.at(i);
  }
  return drivable_lanes;
}

std::vector<DrivableLanes> generateDrivableLanesWithShoulderLanes(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & shoulder_lanes)
{
  std::vector<DrivableLanes> drivable_lanes;
  for (const auto & current_lane : current_lanes) {
    DrivableLanes drivable_lane;

    const auto right_lane = getRightLanelet(current_lane, shoulder_lanes);
    const auto left_lane = getLeftLanelet(current_lane, shoulder_lanes);

    if (right_lane && left_lane) {
      drivable_lane.right_lane = *right_lane;
      drivable_lane.left_lane = *left_lane;
      drivable_lane.middle_lanes.push_back(current_lane);
    } else if (right_lane) {
      drivable_lane.right_lane = *right_lane;
      drivable_lane.left_lane = current_lane;
    } else if (left_lane) {
      drivable_lane.right_lane = current_lane;
      drivable_lane.left_lane = *left_lane;
    } else {
      drivable_lane.right_lane = current_lane;
      drivable_lane.left_lane = current_lane;
    }

    drivable_lanes.push_back(drivable_lane);
  }

  return drivable_lanes;
}

boost::optional<size_t> getOverlappedLaneletId(const std::vector<DrivableLanes> & lanes)
{
  auto overlaps = [](const DrivableLanes & lanes, const DrivableLanes & target_lanes) {
    const auto lanelets = transformToLanelets(lanes);
    const auto target_lanelets = transformToLanelets(target_lanes);

    for (const auto & lanelet : lanelets) {
      for (const auto & target_lanelet : target_lanelets) {
        if (boost::geometry::intersects(
              lanelet.polygon2d().basicPolygon(), target_lanelet.polygon2d().basicPolygon())) {
          return true;
        }
      }
    }

    // No overlapping
    return false;
  };

  if (lanes.size() <= 2) {
    return {};
  }

  size_t overlapped_idx = lanes.size();
  for (size_t i = 0; i < lanes.size() - 2; ++i) {
    for (size_t j = i + 2; j < lanes.size(); ++j) {
      if (overlaps(lanes.at(i), lanes.at(j))) {
        overlapped_idx = std::min(overlapped_idx, j);
      }
    }
  }

  return overlapped_idx;
}

std::vector<DrivableLanes> cutOverlappedLanes(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes)
{
  const auto overlapped_lanelet_id = getOverlappedLaneletId(lanes);
  if (!overlapped_lanelet_id) {
    return lanes;
  }

  const std::vector<DrivableLanes> shorten_lanes{
    lanes.begin(), lanes.begin() + *overlapped_lanelet_id};
  const std::vector<DrivableLanes> removed_lanes{
    lanes.begin() + *overlapped_lanelet_id, lanes.end()};

  const auto transformed_lanes = util::transformToLanelets(removed_lanes);

  auto isIncluded = [&transformed_lanes](const std::vector<int64_t> & lane_ids) {
    if (transformed_lanes.empty() || lane_ids.empty()) return false;

    for (const auto & transformed_lane : transformed_lanes) {
      for (const auto & lane_id : lane_ids) {
        if (lane_id == transformed_lane.id()) {
          return true;
        }
      }
    }

    return false;
  };

  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & lane_ids = path.points.at(i).lane_ids;
    if (isIncluded(lane_ids)) {
      path.points.erase(path.points.begin() + i, path.points.end());
      break;
    }
  }

  return shorten_lanes;
}

geometry_msgs::msg::Point calcLongitudinalOffsetStartPoint(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const size_t nearest_segment_idx, const double offset)
{
  const double offset_length =
    motion_utils::calcLongitudinalOffsetToSegment(points, nearest_segment_idx, pose.position);
  const auto offset_point =
    motion_utils::calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length + offset);

  return offset_point ? offset_point.get() : points.at(nearest_segment_idx);
}

geometry_msgs::msg::Point calcLongitudinalOffsetGoalPoint(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const size_t nearest_segment_idx, const double offset)
{
  const double offset_length =
    motion_utils::calcLongitudinalOffsetToSegment(points, nearest_segment_idx, pose.position);
  const auto offset_point =
    motion_utils::calcLongitudinalOffsetPoint(points, nearest_segment_idx, offset_length + offset);

  return offset_point ? offset_point.get() : points.at(nearest_segment_idx + 1);
}

void generateDrivableArea(
  PathWithLaneId & path, const std::vector<DrivableLanes> & lanes, const double vehicle_length,
  const std::shared_ptr<const PlannerData> planner_data, const bool is_driving_forward)
{
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;

  // extract data
  const auto transformed_lanes = util::transformToLanelets(lanes);
  const auto current_pose = planner_data->self_pose;
  const auto route_handler = planner_data->route_handler;
  constexpr double overlap_threshold = 0.01;

  auto addPoints =
    [](const lanelet::ConstLineString3d & points, std::vector<geometry_msgs::msg::Point> & bound) {
      for (const auto & bound_p : points) {
        const auto cp = lanelet::utils::conversion::toGeomMsgPt(bound_p);
        if (bound.empty()) {
          bound.push_back(cp);
        } else if (tier4_autoware_utils::calcDistance2d(cp, bound.back()) > overlap_threshold) {
          bound.push_back(cp);
        }
      }
    };

  const auto has_overlap =
    [&](const lanelet::ConstLanelet & lane, const lanelet::Id & ignore_lane_id = lanelet::InvalId) {
      for (const auto & transformed_lane : transformed_lanes) {
        if (transformed_lane.id() == ignore_lane_id) {
          continue;
        }
        if (boost::geometry::intersects(
              lane.polygon2d().basicPolygon(), transformed_lane.polygon2d().basicPolygon())) {
          return true;
        }
      }
      return false;
    };

  // Insert Position
  for (const auto & lane : lanes) {
    addPoints(lane.left_lane.leftBound3d(), left_bound);
    addPoints(lane.right_lane.rightBound3d(), right_bound);
  }

  // Insert points after goal
  lanelet::ConstLanelet goal_lanelet;
  if (
    route_handler->getGoalLanelet(&goal_lanelet) &&
    checkHasSameLane(transformed_lanes, goal_lanelet)) {
    const auto lanes_after_goal = route_handler->getLanesAfterGoal(vehicle_length);
    const auto next_lanes_after_goal = route_handler->getNextLanelets(goal_lanelet);
    for (const auto & lane : lanes_after_goal) {
      // If lane is already in the transformed lanes, ignore it
      if (checkHasSameLane(transformed_lanes, lane)) {
        continue;
      }
      // Check if overlapped
      const bool is_overlapped =
        (checkHasSameLane(next_lanes_after_goal, lane)
           ? has_overlap(lane, route_handler->getGoalLaneId())
           : has_overlap(lane));
      if (is_overlapped) {
        continue;
      }

      addPoints(lane.leftBound3d(), left_bound);
      addPoints(lane.rightBound3d(), right_bound);
    }
  }

  if (!is_driving_forward) {
    std::reverse(left_bound.begin(), left_bound.end());
    std::reverse(right_bound.begin(), right_bound.end());
  }

  // Get Closest segment for the start point
  constexpr double front_length = 0.5;
  const auto front_pose = path.points.empty() ? current_pose->pose : path.points.front().point.pose;
  const size_t front_left_start_idx =
    findNearestSegmentIndexFromLateralDistance(left_bound, front_pose.position);
  const size_t front_right_start_idx =
    findNearestSegmentIndexFromLateralDistance(right_bound, front_pose.position);
  const auto left_start_point =
    calcLongitudinalOffsetStartPoint(left_bound, front_pose, front_left_start_idx, -front_length);
  const auto right_start_point =
    calcLongitudinalOffsetStartPoint(right_bound, front_pose, front_right_start_idx, -front_length);
  const size_t left_start_idx =
    findNearestSegmentIndexFromLateralDistance(left_bound, left_start_point);
  const size_t right_start_idx =
    findNearestSegmentIndexFromLateralDistance(right_bound, right_start_point);

  // Get Closest segment for the goal point
  const auto goal_pose = path.points.empty() ? current_pose->pose : path.points.back().point.pose;
  const size_t goal_left_start_idx =
    findNearestSegmentIndexFromLateralDistance(left_bound, goal_pose.position);
  const size_t goal_right_start_idx =
    findNearestSegmentIndexFromLateralDistance(right_bound, goal_pose.position);
  const auto left_goal_point =
    calcLongitudinalOffsetGoalPoint(left_bound, goal_pose, goal_left_start_idx, vehicle_length);
  const auto right_goal_point =
    calcLongitudinalOffsetGoalPoint(right_bound, goal_pose, goal_right_start_idx, vehicle_length);
  const size_t left_goal_idx =
    findNearestSegmentIndexFromLateralDistance(left_bound, left_goal_point);
  const size_t right_goal_idx =
    findNearestSegmentIndexFromLateralDistance(right_bound, right_goal_point);

  // Store Data
  path.left_bound.clear();
  path.right_bound.clear();

  // Insert a start point
  path.left_bound.push_back(left_start_point);
  path.right_bound.push_back(right_start_point);

  // Insert middle points
  for (size_t i = left_start_idx + 1; i <= left_goal_idx; ++i) {
    const auto & next_point = left_bound.at(i);
    const double dist = tier4_autoware_utils::calcDistance2d(path.left_bound.back(), next_point);
    if (dist > overlap_threshold) {
      path.left_bound.push_back(next_point);
    }
  }
  for (size_t i = right_start_idx + 1; i <= right_goal_idx; ++i) {
    const auto & next_point = right_bound.at(i);
    const double dist = tier4_autoware_utils::calcDistance2d(path.right_bound.back(), next_point);
    if (dist > overlap_threshold) {
      path.right_bound.push_back(next_point);
    }
  }

  // Insert a goal point
  if (
    tier4_autoware_utils::calcDistance2d(path.left_bound.back(), left_goal_point) >
    overlap_threshold) {
    path.left_bound.push_back(left_goal_point);
  }
  if (
    tier4_autoware_utils::calcDistance2d(path.right_bound.back(), right_goal_point) >
    overlap_threshold) {
    path.right_bound.push_back(right_goal_point);
  }
}

double getDistanceToEndOfLane(const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const double lanelet_length = lanelet::utils::getLaneletLength3d(lanelets);
  return lanelet_length - arc_coordinates.length;
}

double getDistanceToNextIntersection(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::max();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }
    if (is_after_current_lanelet && llt.hasAttribute("turn_direction")) {
      bool is_lane_change_yes = false;
      const auto right_line = llt.rightBound();
      if (right_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = right_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      const auto left_line = llt.leftBound();
      if (left_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
        const auto attr = left_line.attribute(lanelet::AttributeNamesString::LaneChange);
        if (attr.value() == std::string("yes")) {
          is_lane_change_yes = true;
        }
      }
      if (!is_lane_change_yes) {
        return distance - arc_coordinates.length;
      }
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::max();
}

double getDistanceToCrosswalk(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::max();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }
    // check lane change tag
    bool is_lane_change_yes = false;
    const auto right_line = llt.rightBound();
    if (right_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
      const auto attr = right_line.attribute(lanelet::AttributeNamesString::LaneChange);
      if (attr.value() == std::string("yes")) {
        is_lane_change_yes = true;
      }
    }
    const auto left_line = llt.leftBound();
    if (left_line.hasAttribute(lanelet::AttributeNamesString::LaneChange)) {
      const auto attr = left_line.attribute(lanelet::AttributeNamesString::LaneChange);
      if (attr.value() == std::string("yes")) {
        is_lane_change_yes = true;
      }
    }

    if (is_after_current_lanelet && !is_lane_change_yes) {
      const auto conflicting_crosswalks = overall_graphs.conflictingInGraph(llt, 1);
      if (!(conflicting_crosswalks.empty())) {
        // create centerline
        const lanelet::ConstLineString2d lanelet_centerline = llt.centerline2d();
        LineString2d centerline;
        centerline.reserve(lanelet_centerline.size());
        for (const auto & point : lanelet_centerline) {
          boost::geometry::append(centerline, Point2d(point.x(), point.y()));
        }

        // create crosswalk polygon and calculate distance
        double min_distance_to_crosswalk = std::numeric_limits<double>::max();
        for (const auto & crosswalk : conflicting_crosswalks) {
          lanelet::CompoundPolygon2d lanelet_crosswalk_polygon = crosswalk.polygon2d();
          Polygon2d polygon;
          polygon.outer().reserve(lanelet_crosswalk_polygon.size() + 1);
          for (const auto & point : lanelet_crosswalk_polygon) {
            polygon.outer().emplace_back(point.x(), point.y());
          }
          polygon.outer().push_back(polygon.outer().front());

          std::vector<Point2d> points_intersection;
          boost::geometry::intersection(centerline, polygon, points_intersection);

          for (const auto & point : points_intersection) {
            lanelet::ConstLanelets lanelets = {llt};
            Pose pose_point;
            pose_point.position.x = point.x();
            pose_point.position.y = point.y();
            const lanelet::ArcCoordinates & arc_crosswalk =
              lanelet::utils::getArcCoordinates(lanelets, pose_point);

            const double distance_to_crosswalk = arc_crosswalk.length;
            if (distance_to_crosswalk < min_distance_to_crosswalk) {
              min_distance_to_crosswalk = distance_to_crosswalk;
            }
          }
        }
        if (distance + min_distance_to_crosswalk > arc_coordinates.length) {
          return distance + min_distance_to_crosswalk - arc_coordinates.length;
        }
      }
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::max();
}

double getSignedDistance(
  const Pose & current_pose, const Pose & goal_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto arc_current = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const auto arc_goal = lanelet::utils::getArcCoordinates(lanelets, goal_pose);

  return arc_goal.length - arc_current.length;
}

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets)
{
  std::vector<uint64_t> ids;
  ids.reserve(lanelets.size());
  for (const auto & llt : lanelets) {
    ids.push_back(llt.id());
  }
  return ids;
}

Path convertToPathFromPathWithLaneId(const PathWithLaneId & path_with_lane_id)
{
  Path path;
  path.header = path_with_lane_id.header;
  path.left_bound = path_with_lane_id.left_bound;
  path.right_bound = path_with_lane_id.right_bound;
  path.points.reserve(path_with_lane_id.points.size());
  for (const auto & pt_with_lane_id : path_with_lane_id.points) {
    path.points.push_back(pt_with_lane_id.point);
  }
  return path;
}

lanelet::Polygon3d getVehiclePolygon(
  const Pose & vehicle_pose, const double vehicle_width, const double base_link2front)
{
  tf2::Vector3 front_left, front_right, rear_left, rear_right;
  front_left.setValue(base_link2front, vehicle_width / 2, 0);
  front_right.setValue(base_link2front, -vehicle_width / 2, 0);
  rear_left.setValue(0, vehicle_width / 2, 0);
  rear_right.setValue(0, -vehicle_width / 2, 0);

  tf2::Transform tf;
  tf2::fromMsg(vehicle_pose, tf);
  const auto front_left_transformed = tf * front_left;
  const auto front_right_transformed = tf * front_right;
  const auto rear_left_transformed = tf * rear_left;
  const auto rear_right_transformed = tf * rear_right;

  lanelet::Polygon3d llt_poly;
  auto toPoint3d = [](const tf2::Vector3 & v) { return lanelet::Point3d(0, v.x(), v.y(), v.z()); };
  llt_poly.reserve(4);
  llt_poly.push_back(toPoint3d(front_left_transformed));
  llt_poly.push_back(toPoint3d(front_right_transformed));
  llt_poly.push_back(toPoint3d(rear_right_transformed));
  llt_poly.push_back(toPoint3d(rear_left_transformed));

  return llt_poly;
}

PathPointWithLaneId insertStopPoint(double length, PathWithLaneId * path)
{
  if (path->points.empty()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "failed to insert stop point. path points is empty.");
    return PathPointWithLaneId();
  }

  double accumulated_length = 0;
  size_t insert_idx = 0;
  Pose stop_pose;
  for (size_t i = 1; i < path->points.size(); i++) {
    const auto prev_pose = path->points.at(i - 1).point.pose;
    const auto curr_pose = path->points.at(i).point.pose;
    const double segment_length = tier4_autoware_utils::calcDistance3d(prev_pose, curr_pose);
    accumulated_length += segment_length;
    if (accumulated_length > length) {
      insert_idx = i;
      const double ratio = 1 - (accumulated_length - length) / segment_length;
      stop_pose = lerpByPose(prev_pose, curr_pose, ratio);
      break;
    }
  }
  if (accumulated_length <= length) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "failed to insert stop point. length is longer than path length");
    return PathPointWithLaneId();
  }

  PathPointWithLaneId stop_point;
  stop_point.lane_ids = path->points.at(insert_idx).lane_ids;
  stop_point.point.pose = stop_pose;
  path->points.insert(path->points.begin() + static_cast<int>(insert_idx), stop_point);
  for (size_t i = insert_idx; i < path->points.size(); i++) {
    path->points.at(i).point.longitudinal_velocity_mps = 0.0;
    path->points.at(i).point.lateral_velocity_mps = 0.0;
  }
  return stop_point;
}

double getSignedDistanceFromShoulderLeftBoundary(
  const lanelet::ConstLanelets & shoulder_lanelets, const Pose & pose)
{
  lanelet::ConstLanelet closest_shoulder_lanelet;
  lanelet::ArcCoordinates arc_coordinates;
  if (lanelet::utils::query::getClosestLanelet(
        shoulder_lanelets, pose, &closest_shoulder_lanelet)) {
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(pose.position);
    const auto & left_line_2d = lanelet::utils::to2D(closest_shoulder_lanelet.leftBound3d());
    arc_coordinates = lanelet::geometry::toArcCoordinates(
      left_line_2d, lanelet::utils::to2D(lanelet_point).basicPoint());

  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "closest shoulder lanelet not found.");
  }

  return arc_coordinates.distance;
}

std::optional<double> getSignedDistanceFromShoulderLeftBoundary(
  const lanelet::ConstLanelets & shoulder_lanelets, const LinearRing2d & footprint,
  const Pose & vehicle_pose)
{
  double min_distance = std::numeric_limits<double>::max();
  const auto transformed_footprint =
    transformVector(footprint, tier4_autoware_utils::pose2transform(vehicle_pose));
  bool found_neighbor_shoulder_bound = false;
  for (const auto & vehicle_corner_point : transformed_footprint) {
    // convert point of footprint to pose
    Pose vehicle_corner_pose{};
    vehicle_corner_pose.position = tier4_autoware_utils::toMsg(vehicle_corner_point.to_3d());
    vehicle_corner_pose.orientation = vehicle_pose.orientation;

    // calculate distance to the shoulder bound directly next to footprint points
    lanelet::ConstLanelet closest_shoulder_lanelet{};
    if (lanelet::utils::query::getClosestLanelet(
          shoulder_lanelets, vehicle_corner_pose, &closest_shoulder_lanelet)) {
      const auto & left_line_2d = lanelet::utils::to2D(closest_shoulder_lanelet.leftBound3d());

      for (size_t i = 1; i < left_line_2d.size(); ++i) {
        const Point p_front = lanelet::utils::conversion::toGeomMsgPt(left_line_2d[i - 1]);
        const Point p_back = lanelet::utils::conversion::toGeomMsgPt(left_line_2d[i]);

        const Point inverse_p_front =
          tier4_autoware_utils::inverseTransformPoint(p_front, vehicle_corner_pose);
        const Point inverse_p_back =
          tier4_autoware_utils::inverseTransformPoint(p_back, vehicle_corner_pose);

        const double dy_front = inverse_p_front.y;
        const double dy_back = inverse_p_back.y;
        const double dx_front = inverse_p_front.x;
        const double dx_back = inverse_p_back.x;
        // is in segment
        if (dx_front < 0 && dx_back > 0) {
          const double lateral_distance_from_pose_to_segment =
            (dy_front * dx_back + dy_back * -dx_front) / (dx_back - dx_front);
          min_distance = std::min(lateral_distance_from_pose_to_segment, min_distance);
          found_neighbor_shoulder_bound = true;
          break;
        }
      }
    }
  }

  if (!found_neighbor_shoulder_bound) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "neighbor shoulder bound to footprint is not found.");
    return {};
  }

  return -min_distance;
}

double getSignedDistanceFromRightBoundary(
  const lanelet::ConstLanelets & lanelets, const Pose & pose)
{
  lanelet::ConstLanelet closest_lanelet;
  lanelet::ArcCoordinates arc_coordinates;
  if (lanelet::utils::query::getClosestLanelet(lanelets, pose, &closest_lanelet)) {
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(pose.position);
    const auto & right_line_2d = lanelet::utils::to2D(closest_lanelet.rightBound3d());
    arc_coordinates = lanelet::geometry::toArcCoordinates(
      right_line_2d, lanelet::utils::to2D(lanelet_point).basicPoint());
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "closest shoulder lanelet not found.");
  }

  return arc_coordinates.distance;
}

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & lanelet_sequence, const lanelet::ConstLanelet & target_lane,
  const Pose & pose)
{
  const auto arc_pose = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);

  const auto target_center_line = target_lane.centerline().basicLineString();

  Pose front_pose, back_pose;

  {
    const auto front_point = lanelet::utils::conversion::toGeomMsgPt(target_center_line.front());
    const double front_yaw = lanelet::utils::getLaneletAngle(target_lane, front_point);
    front_pose.position = front_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, front_yaw);
    front_pose.orientation = tf2::toMsg(tf_quat);
  }

  {
    const auto back_point = lanelet::utils::conversion::toGeomMsgPt(target_center_line.back());
    const double back_yaw = lanelet::utils::getLaneletAngle(target_lane, back_point);
    back_pose.position = back_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, back_yaw);
    back_pose.orientation = tf2::toMsg(tf_quat);
  }

  const auto arc_front = lanelet::utils::getArcCoordinates(lanelet_sequence, front_pose);
  const auto arc_back = lanelet::utils::getArcCoordinates(lanelet_sequence, back_pose);

  return std::max(
    std::min(arc_front.length - arc_pose.length, arc_back.length - arc_pose.length), 0.0);
}

std::vector<Polygon2d> getTargetLaneletPolygons(
  const lanelet::PolygonLayer & map_polygons, lanelet::ConstLanelets & lanelets, const Pose & pose,
  const double check_length, const std::string & target_type)
{
  std::vector<Polygon2d> polygons;

  // create lanelet polygon
  const auto arclength = lanelet::utils::getArcCoordinates(lanelets, pose);
  const auto llt_polygon = lanelet::utils::getPolygonFromArcLength(
    lanelets, arclength.length, arclength.length + check_length);
  const auto llt_polygon_2d = lanelet::utils::to2D(llt_polygon).basicPolygon();

  // If the number of vertices is not enough to create polygon, return empty polygon container
  if (llt_polygon_2d.size() < 3) {
    return polygons;
  }

  Polygon2d llt_polygon_bg;
  llt_polygon_bg.outer().reserve(llt_polygon_2d.size() + 1);
  for (const auto & llt_pt : llt_polygon_2d) {
    llt_polygon_bg.outer().emplace_back(llt_pt.x(), llt_pt.y());
  }
  llt_polygon_bg.outer().push_back(llt_polygon_bg.outer().front());

  for (const auto & map_polygon : map_polygons) {
    const std::string type = map_polygon.attributeOr(lanelet::AttributeName::Type, "");
    // If the target_type is different
    // or the number of vertices is not enough to create polygon, skip the loop
    if (type == target_type && map_polygon.size() > 2) {
      // create map polygon
      Polygon2d map_polygon_bg;
      map_polygon_bg.outer().reserve(map_polygon.size() + 1);
      for (const auto & pt : map_polygon) {
        map_polygon_bg.outer().emplace_back(pt.x(), pt.y());
      }
      map_polygon_bg.outer().push_back(map_polygon_bg.outer().front());
      if (boost::geometry::intersects(llt_polygon_bg, map_polygon_bg)) {
        polygons.push_back(map_polygon_bg);
      }
    }
  }
  return polygons;
}

// TODO(Horibe) There is a similar function in route_handler.
std::shared_ptr<PathWithLaneId> generateCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  auto centerline_path = std::make_shared<PathWithLaneId>();

  const auto & p = planner_data->parameters;

  const auto & route_handler = planner_data->route_handler;
  const auto & pose = planner_data->self_pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(pose->pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) What should be returned?
  }

  // For lanelet_sequence with desired length
  lanelet::ConstLanelets lanelet_sequence = route_handler->getLaneletSequence(
    current_lane, pose->pose, p.backward_path_length, p.forward_path_length);

  std::vector<DrivableLanes> drivable_lanes(lanelet_sequence.size());
  for (size_t i = 0; i < lanelet_sequence.size(); ++i) {
    drivable_lanes.at(i).left_lane = lanelet_sequence.at(i);
    drivable_lanes.at(i).right_lane = lanelet_sequence.at(i);
  }

  *centerline_path = getCenterLinePath(
    *route_handler, lanelet_sequence, pose->pose, p.backward_path_length, p.forward_path_length, p);

  centerline_path->header = route_handler->getRouteHeader();

  util::generateDrivableArea(*centerline_path, drivable_lanes, p.vehicle_length, planner_data);

  return centerline_path;
}

// TODO(Azu) Some parts of is the same with generateCenterLinePath. Therefore it might be better if
// we can refactor some of the code for better readability
lanelet::ConstLineStrings3d getMaximumDrivableArea(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;
  const auto & ego_pose = planner_data->self_pose->pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "failed to find closest lanelet within route!!!");
    return {};
  }

  const auto current_lanes = route_handler->getLaneletSequence(
    current_lane, ego_pose, p.backward_path_length, p.forward_path_length);
  lanelet::ConstLineStrings3d linestring_shared;
  for (const auto & lane : current_lanes) {
    lanelet::ConstLineStrings3d furthest_line = route_handler->getFurthestLinestring(lane);
    linestring_shared.insert(linestring_shared.end(), furthest_line.begin(), furthest_line.end());
  }

  return linestring_shared;
}

std::vector<DrivableLanes> expandLanelets(
  const std::vector<DrivableLanes> & drivable_lanes, const double left_bound_offset,
  const double right_bound_offset, const std::vector<std::string> & types_to_skip)
{
  if (left_bound_offset == 0.0 && right_bound_offset == 0.0) return drivable_lanes;

  std::vector<DrivableLanes> expanded_drivable_lanes{};
  expanded_drivable_lanes.reserve(drivable_lanes.size());
  for (const auto & lanes : drivable_lanes) {
    const std::string l_type =
      lanes.left_lane.leftBound().attributeOr(lanelet::AttributeName::Type, "none");
    const std::string r_type =
      lanes.right_lane.rightBound().attributeOr(lanelet::AttributeName::Type, "none");

    const bool l_skip =
      std::find(types_to_skip.begin(), types_to_skip.end(), l_type) != types_to_skip.end();
    const bool r_skip =
      std::find(types_to_skip.begin(), types_to_skip.end(), r_type) != types_to_skip.end();
    const double l_offset = l_skip ? 0.0 : left_bound_offset;
    const double r_offset = r_skip ? 0.0 : -right_bound_offset;

    DrivableLanes expanded_lanes;
    if (lanes.left_lane.id() == lanes.right_lane.id()) {
      expanded_lanes.left_lane =
        lanelet::utils::getExpandedLanelet(lanes.left_lane, l_offset, r_offset);
      expanded_lanes.right_lane =
        lanelet::utils::getExpandedLanelet(lanes.right_lane, l_offset, r_offset);
    } else {
      expanded_lanes.left_lane = lanelet::utils::getExpandedLanelet(lanes.left_lane, l_offset, 0.0);
      expanded_lanes.right_lane =
        lanelet::utils::getExpandedLanelet(lanes.right_lane, 0.0, r_offset);
    }
    expanded_lanes.middle_lanes = lanes.middle_lanes;
    expanded_drivable_lanes.push_back(expanded_lanes);
  }
  return expanded_drivable_lanes;
}

PredictedObjects filterObjectsByVelocity(const PredictedObjects & objects, double lim_v)
{
  return filterObjectsByVelocity(objects, -lim_v, lim_v);
}

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, double min_v, double max_v)
{
  PredictedObjects filtered;
  filtered.header = objects.header;
  for (const auto & obj : objects.objects) {
    const auto v = std::abs(obj.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (min_v < v && v < max_v) {
      filtered.objects.push_back(obj);
    }
  }
  return filtered;
}

void shiftPose(Pose * pose, double shift_length)
{
  auto yaw = tf2::getYaw(pose->orientation);
  pose->position.x -= std::sin(yaw) * shift_length;
  pose->position.y += std::cos(yaw) * shift_length;
}

PathWithLaneId getCenterLinePath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & lanelet_sequence,
  const Pose & pose, const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameter, const double optional_length)
{
  PathWithLaneId reference_path;

  if (lanelet_sequence.empty()) {
    return reference_path;
  }

  const auto arc_coordinates = lanelet::utils::getArcCoordinates(lanelet_sequence, pose);
  const double s = arc_coordinates.length;
  const double s_backward = std::max(0., s - backward_path_length);
  double s_forward = s + forward_path_length;

  const int num_lane_change =
    std::abs(route_handler.getNumLaneToPreferredLane(lanelet_sequence.back()));
  const double lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
  const double lane_change_buffer =
    calcLaneChangeBuffer(parameter, std::abs(num_lane_change), optional_length);

  if (route_handler.isDeadEndLanelet(lanelet_sequence.back())) {
    s_forward = std::min(s_forward, lane_length - lane_change_buffer);
  }

  if (route_handler.isInGoalRouteSection(lanelet_sequence.back())) {
    const auto goal_arc_coordinates =
      lanelet::utils::getArcCoordinates(lanelet_sequence, route_handler.getGoalPose());
    s_forward = std::min(s_forward, goal_arc_coordinates.length - lane_change_buffer);
  }

  return route_handler.getCenterLinePath(lanelet_sequence, s_backward, s_forward, true);
}

bool checkLaneIsInIntersection(
  const RouteHandler & route_handler, const PathWithLaneId & reference_path,
  const lanelet::ConstLanelets & lanelet_sequence, const BehaviorPathPlannerParameters & parameter,
  const int num_lane_change, double & additional_length_to_add)
{
  if (num_lane_change == 0) {
    return false;
  }

  if (lanelet_sequence.size() < 2 || reference_path.points.empty()) {
    return false;
  }

  const auto goal_pose = route_handler.getGoalPose();
  lanelet::ConstLanelet goal_lanelet;
  if (!route_handler.getGoalLanelet(&goal_lanelet)) {
    std::cerr << "fail to get goal lanelet for intersection check.\n";
    return false;
  }
  const auto goal_lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal_pose.position);

  const auto min_lane_change_distance =
    num_lane_change *
    (parameter.minimum_lane_change_length + parameter.backward_length_buffer_for_end_of_lane);
  const double goal_distance_in_goal_lanelet = std::invoke([&goal_lanelet_point, &goal_lanelet]() {
    const auto goal_centerline = goal_lanelet.centerline2d();
    const auto arc_coordinate =
      lanelet::geometry::toArcCoordinates(goal_centerline, goal_lanelet_point.basicPoint2d());
    return arc_coordinate.length;
  });

  if (goal_distance_in_goal_lanelet > min_lane_change_distance) {
    return false;
  }

  const auto & path_points = reference_path.points;
  const auto & end_of_route_pose = path_points.back().point.pose;

  if (lanelet_sequence.back().id() != goal_lanelet.id()) {
    const auto get_signed_distance =
      getSignedDistance(end_of_route_pose, goal_pose, lanelet_sequence);

    if (get_signed_distance > min_lane_change_distance) {
      return false;
    }
  }
  // if you come to here, basically either back lane is goal, or back lane to goal is not enough
  // distance
  auto prev_lane = lanelet_sequence.crbegin();
  auto find_intersection_iter = lanelet_sequence.crbegin();
  for (; find_intersection_iter != lanelet_sequence.crend(); ++find_intersection_iter) {
    prev_lane = std::next(find_intersection_iter);
    if (prev_lane == lanelet_sequence.crend()) {
      return false;
    }
    const auto lanes = route_handler.getNextLanelets(*prev_lane);
    const auto is_neighbor_with_turn_direction = std::any_of(
      lanes.cbegin(), lanes.cend(),
      [](const lanelet::ConstLanelet & lane) { return lane.hasAttribute("turn_direction"); });

    if (is_neighbor_with_turn_direction) {
      const auto & intersection_back = find_intersection_iter->centerline2d().back();
      geometry_msgs::msg::Pose intersection_back_pose;
      intersection_back_pose.position.x = intersection_back.x();
      intersection_back_pose.position.y = intersection_back.y();
      intersection_back_pose.position.z = 0.0;
      const auto get_signed_distance =
        getSignedDistance(intersection_back_pose, goal_pose, lanelet_sequence);

      if (get_signed_distance > min_lane_change_distance) {
        return false;
      }
      break;
    }
  }

  const auto checkAttribute = [](const lanelet::ConstLineString3d & linestring) noexcept {
    const auto & attribute_name = lanelet::AttributeNamesString::LaneChange;
    if (linestring.hasAttribute(attribute_name)) {
      const auto attr = linestring.attribute(attribute_name);
      if (attr.value() == std::string("yes")) {
        return true;
      }
    }
    return false;
  };

  const auto isLaneChangeAttributeYes =
    [checkAttribute](const lanelet::ConstLanelet & lanelet) noexcept {
      return (checkAttribute(lanelet.rightBound()) || checkAttribute(lanelet.leftBound()));
    };

  lanelet::ConstLanelets lane_change_prohibited_lanes{*find_intersection_iter};
  for (auto prev_ll_itr = prev_lane; prev_ll_itr != lanelet_sequence.crend(); ++prev_ll_itr) {
    if (isLaneChangeAttributeYes(*prev_ll_itr)) {
      break;
    }
    lane_change_prohibited_lanes.push_back(*prev_ll_itr);
  }

  std::reverse(lane_change_prohibited_lanes.begin(), lane_change_prohibited_lanes.end());
  const auto prohibited_arc_coordinate =
    lanelet::utils::getArcCoordinates(lane_change_prohibited_lanes, goal_pose);

  additional_length_to_add =
    (num_lane_change > 0 ? 1 : -1) *
    (parameter.backward_length_buffer_for_end_of_lane + prohibited_arc_coordinate.length);

  return true;
}

// for lane following
PathWithLaneId setDecelerationVelocity(
  const RouteHandler & route_handler, const PathWithLaneId & input,
  const lanelet::ConstLanelets & lanelet_sequence, const double lane_change_prepare_duration,
  const double lane_change_buffer)
{
  auto reference_path = input;
  if (
    route_handler.isDeadEndLanelet(lanelet_sequence.back()) &&
    lane_change_prepare_duration > std::numeric_limits<double>::epsilon()) {
    for (auto & point : reference_path.points) {
      const double lane_length = lanelet::utils::getLaneletLength2d(lanelet_sequence);
      const auto arclength = lanelet::utils::getArcCoordinates(lanelet_sequence, point.point.pose);
      const double distance_to_end =
        std::max(0.0, lane_length - std::abs(lane_change_buffer) - arclength.length);
      point.point.longitudinal_velocity_mps = std::min(
        point.point.longitudinal_velocity_mps,
        static_cast<float>(distance_to_end / lane_change_prepare_duration));
    }
  }
  return reference_path;
}

// TODO(murooka) remove calcSignedArcLength using findNearestSegmentIndex inside the
// function
PathWithLaneId setDecelerationVelocity(
  const PathWithLaneId & input, const double target_velocity, const Pose target_pose,
  const double buffer, const double deceleration_interval)
{
  auto reference_path = input;

  for (auto & point : reference_path.points) {
    const auto arclength_to_target = std::max(
      0.0, motion_utils::calcSignedArcLength(
             reference_path.points, point.point.pose.position, target_pose.position) +
             buffer);
    if (arclength_to_target > deceleration_interval) continue;
    point.point.longitudinal_velocity_mps = std::min(
      point.point.longitudinal_velocity_mps,
      static_cast<float>(
        (arclength_to_target / deceleration_interval) *
          (point.point.longitudinal_velocity_mps - target_velocity) +
        target_velocity));
  }

  const auto stop_point_length =
    motion_utils::calcSignedArcLength(reference_path.points, 0, target_pose.position) + buffer;
  constexpr double eps{0.01};
  if (std::abs(target_velocity) < eps && stop_point_length > 0.0) {
    const auto stop_point = util::insertStopPoint(stop_point_length, &reference_path);
  }

  return reference_path;
}

// TODO(murooka) remove calcSignedArcLength using findNearestSegmentIndex inside the
// function
PathWithLaneId setDecelerationVelocityForTurnSignal(
  const PathWithLaneId & input, const Pose target_pose, const double turn_light_on_threshold_time)
{
  auto reference_path = input;

  for (auto & point : reference_path.points) {
    const auto arclength_to_target = std::max(
      0.0, motion_utils::calcSignedArcLength(
             reference_path.points, point.point.pose.position, target_pose.position));
    point.point.longitudinal_velocity_mps = std::min(
      point.point.longitudinal_velocity_mps,
      static_cast<float>(arclength_to_target / turn_light_on_threshold_time));
  }

  const auto stop_point_length =
    motion_utils::calcSignedArcLength(reference_path.points, 0, target_pose.position);
  if (stop_point_length > 0) {
    const auto stop_point = util::insertStopPoint(stop_point_length, &reference_path);
  }

  return reference_path;
}

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification)
{
  std::uint8_t label = ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }
  return label;
}

lanelet::ConstLanelets getCurrentLanes(const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_pose->pose;
  const auto & common_parameters = planner_data->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    // RCLCPP_ERROR(getLogger(), "failed to find closest lanelet within route!!!");
    std::cerr << "failed to find closest lanelet within route!!!" << std::endl;
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler->getLaneletSequence(
    current_lane, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length);
}

lanelet::ConstLanelets extendLanes(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes)
{
  auto extended_lanes = lanes;

  // Add next lane
  const auto next_lanes = route_handler->getNextLanelets(extended_lanes.back());
  if (!next_lanes.empty()) {
    extended_lanes.push_back(next_lanes.front());
  }

  // Add previous lane
  const auto prev_lanes = route_handler->getPreviousLanelets(extended_lanes.front());
  if (!prev_lanes.empty()) {
    extended_lanes.insert(extended_lanes.begin(), prev_lanes.front());
  }

  return extended_lanes;
}

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_pose->pose;
  const auto common_parameters = planner_data->parameters;
  const auto routing_graph_ptr = route_handler->getRoutingGraphPtr();

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  const auto current_lanes = route_handler->getLaneletSequence(
    current_lane, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length);

  return extendLanes(route_handler, current_lanes);
}

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<RouteHandler> route_handler, const Pose & pose, const double forward_length,
  const double backward_length)
{
  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes =
    route_handler->getLaneletSequence(current_lane, pose, backward_length, forward_length);

  return current_lanes;
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

  return object_polygon;
}

Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const Shape & obj_shape)
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

  return object_polygon;
}

Polygon2d convertPolygonObjectToGeometryPolygon(const Pose & current_pose, const Shape & obj_shape)
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

  return object_polygon;
}

std::string getUuidStr(const PredictedObject & obj)
{
  std::stringstream hex_value;
  for (const auto & uuid : obj.object_id.uuid) {
    hex_value << std::hex << std::setfill('0') << std::setw(2) << +uuid;
  }
  return hex_value.str();
}

template <typename Pythagoras>
ProjectedDistancePoint pointToSegment(
  const Point2d & reference_point, const Point2d & point_from_ego,
  const Point2d & point_from_object)
{
  auto copied_point_from_object = point_from_object;
  auto copied_point_from_reference = reference_point;
  bg::subtract_point(copied_point_from_object, point_from_ego);
  bg::subtract_point(copied_point_from_reference, point_from_ego);

  const auto c1 = bg::dot_product(copied_point_from_reference, copied_point_from_object);
  if (!(c1 > 0)) {
    return {point_from_ego, Pythagoras::apply(reference_point, point_from_ego)};
  }

  const auto c2 = bg::dot_product(copied_point_from_object, copied_point_from_object);
  if (!(c2 > c1)) {
    return {point_from_object, Pythagoras::apply(reference_point, point_from_object)};
  }

  Point2d projected = point_from_ego;
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

bool getEgoExpectedPoseAndConvertToPolygon(
  const Pose & current_pose, const PredictedPath & pred_path,
  tier4_autoware_utils::Polygon2d & ego_polygon, const double & check_current_time,
  const VehicleInfo & ego_info, Pose & expected_pose, std::string & failed_reason)
{
  bool is_lerped =
    util::lerpByTimeStamp(pred_path, check_current_time, &expected_pose, failed_reason);
  expected_pose.orientation = current_pose.orientation;

  const auto & i = ego_info;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0;
  const auto & back_m = i.rear_overhang_m;

  ego_polygon =
    util::convertBoundingBoxObjectToGeometryPolygon(expected_pose, front_m, back_m, width_m);

  return is_lerped;
}

bool getObjectExpectedPoseAndConvertToPolygon(
  const PredictedPath & pred_path, const PredictedObject & object, Polygon2d & obj_polygon,
  const double & check_current_time, Pose & expected_pose, std::string & failed_reason)
{
  bool is_lerped =
    util::lerpByTimeStamp(pred_path, check_current_time, &expected_pose, failed_reason);
  expected_pose.orientation = object.kinematics.initial_pose_with_covariance.pose.orientation;

  is_lerped = util::calcObjectPolygon(object.shape, expected_pose, &obj_polygon);
  return is_lerped;
}

std::vector<PredictedPath> getPredictedPathFromObj(
  const PredictedObject & obj, const bool & is_use_all_predicted_path)
{
  std::vector<PredictedPath> predicted_path_vec;
  if (is_use_all_predicted_path) {
    std::copy_if(
      obj.kinematics.predicted_paths.cbegin(), obj.kinematics.predicted_paths.cend(),
      std::back_inserter(predicted_path_vec),
      [](const PredictedPath & path) { return !path.path.empty(); });
  } else {
    const auto max_confidence_path = std::max_element(
      obj.kinematics.predicted_paths.begin(), obj.kinematics.predicted_paths.end(),
      [](const auto & path1, const auto & path2) { return path1.confidence > path2.confidence; });
    if (max_confidence_path != obj.kinematics.predicted_paths.end()) {
      predicted_path_vec.push_back(*max_confidence_path);
    }
  }
  return predicted_path_vec;
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

bool isObjectFront(const Pose & ego_pose, const Pose & obj_pose)
{
  const auto obj_from_ego = projectCurrentPoseToTarget(ego_pose, obj_pose);
  return obj_from_ego.position.x > -1e-3;
}

bool isObjectFront(const Pose & projected_ego_pose)
{
  return projected_ego_pose.position.x > -1e-3;
}

double stoppingDistance(const double & vehicle_velocity, const double & vehicle_accel)
{
  const auto deceleration = (vehicle_accel < -1e-3) ? vehicle_accel : -1.0;
  return -std::pow(vehicle_velocity, 2) / (2.0 * deceleration);
}

double frontVehicleStopDistance(
  const double & front_vehicle_velocity, const double & front_vehicle_accel,
  const double & distance_to_collision)
{
  return stoppingDistance(front_vehicle_velocity, front_vehicle_accel) + distance_to_collision;
}

double rearVehicleStopDistance(
  const double & rear_vehicle_velocity, const double & rear_vehicle_accel,
  const double & rear_vehicle_reaction_time, const double & rear_vehicle_safety_time_margin)
{
  return rear_vehicle_velocity * rear_vehicle_reaction_time +
         stoppingDistance(rear_vehicle_velocity, rear_vehicle_accel) +
         rear_vehicle_velocity * rear_vehicle_safety_time_margin;
}

bool isLongitudinalDistanceEnough(
  const double & rear_vehicle_stop_threshold, const double & front_vehicle_stop_threshold)
{
  return rear_vehicle_stop_threshold <= front_vehicle_stop_threshold;
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

  if (isLateralDistanceEnough(
        front_vehicle_pose.position.y, param.lateral_distance_max_threshold)) {
    return true;
  }

  const auto is_obj_in_front = isObjectFront(front_vehicle_pose);
  debug.is_front = is_obj_in_front;

  const auto front_vehicle_velocity =
    (is_obj_in_front) ? object_current_twist.linear : ego_current_twist.linear;

  const auto rear_vehicle_velocity =
    (is_obj_in_front) ? ego_current_twist.linear : object_current_twist.linear;
  debug.object_twist.linear = (is_obj_in_front) ? front_vehicle_velocity : rear_vehicle_velocity;

  const auto front_vehicle_stop_threshold = frontVehicleStopDistance(
    util::l2Norm(front_vehicle_velocity), front_decel, std::fabs(front_vehicle_pose.position.x));

  const auto rear_vehicle_stop_threshold = std::max(
    rearVehicleStopDistance(
      util::l2Norm(rear_vehicle_velocity), rear_decel, param.rear_vehicle_reaction_time,
      param.rear_vehicle_safety_time_margin),
    param.longitudinal_distance_min_threshold);

  return isLongitudinalDistanceEnough(rear_vehicle_stop_threshold, front_vehicle_stop_threshold);
}

bool isLateralDistanceEnough(
  const double & relative_lateral_distance, const double & lateral_distance_threshold)
{
  return std::fabs(relative_lateral_distance) > lateral_distance_threshold;
}

bool isSafeInLaneletCollisionCheck(
  const Pose & ego_current_pose, const Twist & ego_current_twist,
  const PredictedPath & ego_predicted_path, const VehicleInfo & ego_info,
  const double check_start_time, const double check_end_time, const double check_time_resolution,
  const PredictedObject & target_object, const PredictedPath & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const double front_decel,
  const double rear_decel, CollisionCheckDebug & debug)
{
  const auto lerp_path_reserve = (check_end_time - check_start_time) / check_time_resolution;
  if (lerp_path_reserve > 1e-3) {
    debug.lerped_path.reserve(static_cast<size_t>(lerp_path_reserve));
  }

  Pose expected_obj_pose = target_object.kinematics.initial_pose_with_covariance.pose;
  Pose expected_ego_pose = ego_current_pose;
  for (double t = check_start_time; t < check_end_time; t += check_time_resolution) {
    tier4_autoware_utils::Polygon2d obj_polygon;
    [[maybe_unused]] const auto get_obj_info = util::getObjectExpectedPoseAndConvertToPolygon(
      target_object_path, target_object, obj_polygon, t, expected_obj_pose, debug.failed_reason);

    tier4_autoware_utils::Polygon2d ego_polygon;
    [[maybe_unused]] const auto get_ego_info = util::getEgoExpectedPoseAndConvertToPolygon(
      ego_current_pose, ego_predicted_path, ego_polygon, t, ego_info, expected_ego_pose,
      debug.failed_reason);

    debug.ego_polygon = ego_polygon;
    debug.obj_polygon = obj_polygon;
    if (boost::geometry::overlaps(ego_polygon, obj_polygon)) {
      debug.failed_reason = "overlap_polygon";
      return false;
    }

    debug.lerped_path.push_back(expected_ego_pose);

    util::getProjectedDistancePointFromPolygons(
      ego_polygon, obj_polygon, expected_ego_pose, expected_obj_pose);
    debug.expected_ego_pose = expected_ego_pose;
    debug.expected_obj_pose = expected_obj_pose;

    const auto & object_twist = target_object.kinematics.initial_twist_with_covariance.twist;
    if (!util::hasEnoughDistance(
          expected_ego_pose, ego_current_twist, expected_obj_pose, object_twist, common_parameters,
          front_decel, rear_decel, debug)) {
      debug.failed_reason = "not_enough_longitudinal";
      return false;
    }
  }
  return true;
}

bool isSafeInFreeSpaceCollisionCheck(
  const Pose & ego_current_pose, const Twist & ego_current_twist,
  const PredictedPath & ego_predicted_path, const VehicleInfo & ego_info,
  const double check_start_time, const double check_end_time, const double check_time_resolution,
  const PredictedObject & target_object, const BehaviorPathPlannerParameters & common_parameters,
  const double front_decel, const double rear_decel, CollisionCheckDebug & debug)
{
  tier4_autoware_utils::Polygon2d obj_polygon;
  if (!util::calcObjectPolygon(target_object, &obj_polygon)) {
    return false;
  }
  Pose expected_ego_pose = ego_current_pose;
  for (double t = check_start_time; t < check_end_time; t += check_time_resolution) {
    tier4_autoware_utils::Polygon2d ego_polygon;
    [[maybe_unused]] const auto get_ego_info = util::getEgoExpectedPoseAndConvertToPolygon(
      ego_current_pose, ego_predicted_path, ego_polygon, t, ego_info, expected_ego_pose,
      debug.failed_reason);

    debug.ego_polygon = ego_polygon;
    debug.obj_polygon = obj_polygon;
    if (boost::geometry::overlaps(ego_polygon, obj_polygon)) {
      debug.failed_reason = "overlap_polygon";
      return false;
    }

    auto expected_obj_pose = target_object.kinematics.initial_pose_with_covariance.pose;
    util::getProjectedDistancePointFromPolygons(
      ego_polygon, obj_polygon, expected_ego_pose, expected_obj_pose);

    debug.expected_ego_pose = expected_ego_pose;
    debug.expected_obj_pose = expected_obj_pose;

    const auto object_twist = target_object.kinematics.initial_twist_with_covariance.twist;
    if (!util::hasEnoughDistance(
          expected_ego_pose, ego_current_twist,
          target_object.kinematics.initial_pose_with_covariance.pose, object_twist,
          common_parameters, front_decel, rear_decel, debug)) {
      debug.failed_reason = "not_enough_longitudinal";
      return false;
    }
  }
  return true;
}

bool checkPathRelativeAngle(const PathWithLaneId & path, const double angle_threshold)
{
  // We need at least three points to compute relative angle
  constexpr size_t relative_angle_points_num = 3;
  if (path.points.size() < relative_angle_points_num) {
    return true;
  }

  for (size_t p1_id = 0; p1_id <= path.points.size() - relative_angle_points_num; ++p1_id) {
    // Get Point1
    const auto & p1 = path.points.at(p1_id).point.pose.position;

    // Get Point2
    const auto & p2 = path.points.at(p1_id + 1).point.pose.position;

    // Get Point3
    const auto & p3 = path.points.at(p1_id + 2).point.pose.position;

    // ignore invert driving direction
    if (
      path.points.at(p1_id).point.longitudinal_velocity_mps < 0 ||
      path.points.at(p1_id + 1).point.longitudinal_velocity_mps < 0 ||
      path.points.at(p1_id + 2).point.longitudinal_velocity_mps < 0) {
      continue;
    }

    // convert to p1 coordinate
    const double x3 = p3.x - p1.x;
    const double x2 = p2.x - p1.x;
    const double y3 = p3.y - p1.y;
    const double y2 = p2.y - p1.y;

    // calculate relative angle of vector p3 based on p1p2 vector
    const double th = std::atan2(y2, x2);
    const double th2 =
      std::atan2(-x3 * std::sin(th) + y3 * std::cos(th), x3 * std::cos(th) + y3 * std::sin(th));
    if (std::abs(th2) > angle_threshold) {
      // invalid angle
      return false;
    }
  }

  return true;
}

double calcTotalLaneChangeDistanceWithBuffer(const BehaviorPathPlannerParameters & common_param)
{
  const double minimum_lane_change_distance =
    common_param.minimum_lane_change_prepare_distance + common_param.minimum_lane_change_length;
  const double end_of_lane_buffer = common_param.backward_length_buffer_for_end_of_lane;
  return minimum_lane_change_distance + end_of_lane_buffer;
}

double calcLaneChangeBuffer(
  const BehaviorPathPlannerParameters & common_param, const int num_lane_change,
  const double length_to_intersection)
{
  return num_lane_change * calcTotalLaneChangeDistanceWithBuffer(common_param) +
         length_to_intersection;
}
}  // namespace behavior_path_planner::util
