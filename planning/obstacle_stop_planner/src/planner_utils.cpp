// Copyright 2020 Tier IV, Inc.
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

#include "obstacle_stop_planner/planner_utils.hpp"

#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <diagnostic_msgs/msg/key_value.hpp>

#include <boost/format.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace motion_planning
{

using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using motion_utils::calcDecelDistWithJerkAndAccConstraints;
using motion_utils::findFirstNearestIndexWithSoftConstraints;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getRPY;

std::optional<std::pair<double, double>> calcFeasibleMarginAndVelocity(
  const SlowDownParam & slow_down_param, const double dist_baselink_to_obstacle,
  const double current_vel, const double current_acc)
{
  const auto & p = slow_down_param;
  const auto & logger = rclcpp::get_logger("calcFeasibleMarginAndVelocity");
  constexpr double epsilon = 1e-4;

  if (current_vel < p.slow_down_velocity + epsilon) {
    return std::make_pair(p.longitudinal_forward_margin, p.slow_down_velocity);
  }

  for (double planning_jerk = p.jerk_start; planning_jerk > p.slow_down_min_jerk - epsilon;
       planning_jerk += p.jerk_span) {
    const double jerk_dec = planning_jerk;
    const double jerk_acc = std::abs(planning_jerk);

    const auto planning_dec =
      planning_jerk > p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
    const auto stop_dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, p.slow_down_velocity, current_acc, planning_dec, jerk_acc, jerk_dec);

    if (!stop_dist) {
      continue;
    }

    if (stop_dist.value() + p.longitudinal_forward_margin < dist_baselink_to_obstacle) {
      RCLCPP_DEBUG(
        logger, "[found plan] dist:%-6.2f jerk:%-6.2f margin:%-6.2f v0:%-6.2f vt:%-6.2f",
        stop_dist.value(), planning_jerk, p.longitudinal_forward_margin, p.slow_down_velocity,
        current_vel);
      return std::make_pair(p.longitudinal_forward_margin, p.slow_down_velocity);
    }
  }

  {
    const double jerk_dec = p.slow_down_min_jerk;
    const double jerk_acc = std::abs(p.slow_down_min_jerk);

    const auto planning_dec =
      p.slow_down_min_jerk > p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
    const auto stop_dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, p.slow_down_velocity, current_acc, planning_dec, jerk_acc, jerk_dec);

    if (!stop_dist) {
      return {};
    }

    if (stop_dist.value() + p.min_longitudinal_forward_margin < dist_baselink_to_obstacle) {
      const auto planning_margin = dist_baselink_to_obstacle - stop_dist.value();
      RCLCPP_DEBUG(
        logger, "[relax margin] dist:%-6.2f jerk:%-6.2f margin:%-6.2f v0:%-6.2f vt%-6.2f",
        stop_dist.value(), p.slow_down_min_jerk, planning_margin, p.slow_down_velocity,
        current_vel);
      return std::make_pair(planning_margin, p.slow_down_velocity);
    }
  }

  RCLCPP_DEBUG(logger, "relax target slow down velocity");
  return {};
}

std::optional<std::pair<size_t, TrajectoryPoint>> getForwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin)
{
  if (base_idx + 1 > trajectory.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, trajectory.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; i < trajectory.size() - 1; ++i) {
    const auto & p_front = trajectory.at(i);
    const auto & p_back = trajectory.at(i + 1);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert = getBackwardPointFromBasePoint(p_back, p_front, p_back, length_residual);

      // p_front(trajectory.points.at(i)) is insert base point
      return std::make_pair(i, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(trajectory.size() - 1, trajectory.back());
  }

  return {};
}

std::optional<std::pair<size_t, TrajectoryPoint>> getBackwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin)
{
  if (base_idx + 1 > trajectory.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, trajectory.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; 0 < i; --i) {
    const auto & p_front = trajectory.at(i - 1);
    const auto & p_back = trajectory.at(i);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert =
        getBackwardPointFromBasePoint(p_front, p_back, p_front, length_residual);

      // p_front(trajectory.points.at(i-1)) is insert base point
      return std::make_pair(i - 1, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(size_t(0), trajectory.front());
  }

  return {};
}

std::optional<std::pair<size_t, double>> findNearestFrontIndex(
  const size_t start_idx, const TrajectoryPoints & trajectory, const Point & point)
{
  for (size_t i = start_idx; i < trajectory.size(); ++i) {
    const auto & p_traj = trajectory.at(i).pose;
    const auto yaw = getRPY(p_traj).z;
    const Point2d p_traj_direction(std::cos(yaw), std::sin(yaw));
    const Point2d p_traj_to_target(point.x - p_traj.position.x, point.y - p_traj.position.y);

    const auto is_in_front_of_target_point = p_traj_direction.dot(p_traj_to_target) < 0.0;
    const auto is_trajectory_end = i + 1 == trajectory.size();

    if (is_in_front_of_target_point || is_trajectory_end) {
      const auto dist_p_traj_to_target = p_traj_direction.normalized().dot(p_traj_to_target);
      return std::make_pair(i, dist_p_traj_to_target);
    }
  }

  return {};
}

bool isInFrontOfTargetPoint(const Pose & pose, const Point & point)
{
  const auto yaw = getRPY(pose).z;
  const Point2d pose_direction(std::cos(yaw), std::sin(yaw));
  const Point2d to_target(point.x - pose.position.x, point.y - pose.position.y);

  return pose_direction.dot(to_target) < 0.0;
}

bool checkValidIndex(const Pose & p_base, const Pose & p_next, const Pose & p_target)
{
  const Point2d base2target(
    p_target.position.x - p_base.position.x, p_target.position.y - p_base.position.y);
  const Point2d target2next(
    p_next.position.x - p_target.position.x, p_next.position.y - p_target.position.y);
  return base2target.dot(target2next) > 0.0;
}

std::string jsonDumpsPose(const Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

DiagnosticStatus makeStopReasonDiag(const std::string stop_reason, const Pose & stop_pose)
{
  DiagnosticStatus stop_reason_diag;
  KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}

TrajectoryPoint getBackwardPointFromBasePoint(
  const TrajectoryPoint & p_from, const TrajectoryPoint & p_to, const TrajectoryPoint & p_base,
  const double backward_length)
{
  TrajectoryPoint output;
  const double dx = p_to.pose.position.x - p_from.pose.position.x;
  const double dy = p_to.pose.position.y - p_from.pose.position.y;
  const double norm = std::hypot(dx, dy);

  output = p_base;
  output.pose.position.x += backward_length * dx / norm;
  output.pose.position.y += backward_length * dy / norm;

  return output;
}

rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}

bool withinPolygon(
  const Polygon2d & boost_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, PointCloud::Ptr candidate_points_ptr,
  PointCloud::Ptr within_points_ptr)
{
  bool find_within_points = false;

  for (size_t j = 0; j < candidate_points_ptr->size(); ++j) {
    Point2d point(candidate_points_ptr->at(j).x, candidate_points_ptr->at(j).y);
    if (bg::distance(prev_point, point) < radius || bg::distance(next_point, point) < radius) {
      if (bg::within(point, boost_polygon)) {
        within_points_ptr->push_back(candidate_points_ptr->at(j));
        find_within_points = true;
      }
    }
  }
  return find_within_points;
}

bool withinPolyhedron(
  const Polygon2d & boost_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, PointCloud::Ptr candidate_points_ptr,
  PointCloud::Ptr within_points_ptr, double z_min, double z_max)
{
  bool find_within_points = false;

  for (const auto & candidate_point : *candidate_points_ptr) {
    Point2d point(candidate_point.x, candidate_point.y);
    if (bg::distance(prev_point, point) < radius || bg::distance(next_point, point) < radius) {
      if (bg::within(point, boost_polygon)) {
        if (candidate_point.z < z_max && candidate_point.z > z_min) {
          within_points_ptr->push_back(candidate_point);
          find_within_points = true;
        }
      }
    }
  }
  return find_within_points;
}

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  boost::geometry::append(polygon.outer(), point);
}

void createOneStepPolygon(
  const Pose & base_step_pose, const Pose & next_step_pose, Polygon2d & hull_polygon,
  const VehicleInfo & vehicle_info, const double expand_width)
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

  boost::geometry::convex_hull(polygon, hull_polygon);
}

void insertStopPoint(
  const StopPoint & stop_point, TrajectoryPoints & output, DiagnosticStatus & stop_reason_diag)
{
  const auto traj_end_idx = output.size() - 1;
  const auto & stop_idx = stop_point.index;

  const auto & p_base = output.at(stop_idx);
  const auto & p_next = output.at(std::min(stop_idx + 1, traj_end_idx));
  const auto & p_insert = stop_point.point;

  constexpr double min_dist = 1e-3;

  const auto is_p_base_and_p_insert_overlap = calcDistance2d(p_base, p_insert) < min_dist;
  const auto is_p_next_and_p_insert_overlap = calcDistance2d(p_next, p_insert) < min_dist;
  const auto is_valid_index = checkValidIndex(p_base.pose, p_next.pose, p_insert.pose);

  auto update_stop_idx = stop_idx;

  if (!is_p_base_and_p_insert_overlap && !is_p_next_and_p_insert_overlap && is_valid_index) {
    // insert: start_idx and end_idx are shifted by one
    output.insert(output.begin() + stop_idx + 1, p_insert);
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  } else if (is_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  }

  for (size_t i = update_stop_idx; i < output.size(); ++i) {
    output.at(i).longitudinal_velocity_mps = 0.0;
  }

  stop_reason_diag = makeStopReasonDiag("obstacle", p_insert.pose);
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

TrajectoryPoints decimateTrajectory(
  const TrajectoryPoints & input, const double step_length,
  std::map<size_t /* decimate */, size_t /* origin */> & index_map)
{
  TrajectoryPoints output{};

  double trajectory_length_sum = 0.0;
  double next_length = 0.0;

  for (int i = 0; i < static_cast<int>(input.size()) - 1; ++i) {
    const auto & p_front = input.at(i);
    const auto & p_back = input.at(i + 1);
    constexpr double epsilon = 1e-3;

    if (next_length <= trajectory_length_sum + epsilon) {
      const auto p_interpolate =
        getBackwardPointFromBasePoint(p_front, p_back, p_back, next_length - trajectory_length_sum);
      output.push_back(p_interpolate);

      index_map.insert(std::make_pair(output.size() - 1, size_t(i)));
      next_length += step_length;
      continue;
    }

    trajectory_length_sum += calcDistance2d(p_front, p_back);
  }
  if (!input.empty()) {
    output.push_back(input.back());
    index_map.insert(std::make_pair(output.size() - 1, input.size() - 1));
  }

  return output;
}

TrajectoryPoints extendTrajectory(const TrajectoryPoints & input, const double extend_distance)
{
  TrajectoryPoints output = input;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output;
  }

  const auto goal_point = input.back();
  constexpr double interpolation_distance = 0.1;

  double extend_sum = interpolation_distance;
  while (extend_sum <= (extend_distance - interpolation_distance)) {
    const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_sum, goal_point);
    output.push_back(extend_trajectory_point);
    extend_sum += interpolation_distance;
  }
  const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_distance, goal_point);
  output.push_back(extend_trajectory_point);

  return output;
}

bool intersectsInZAxis(const PredictedObject & object, const double z_min, const double z_max)
{
  const auto & obj_pose = object.kinematics.initial_pose_with_covariance;
  const auto & obj_height = object.shape.dimensions.z;
  return obj_pose.pose.position.z - obj_height / 2.0 <= z_max &&
         obj_pose.pose.position.z + obj_height / 2.0 >= z_min;
}

pcl::PointXYZ pointToPcl(const double x, const double y, const double z)
{
  // Store the point components in a variant
  PointVariant x_variant = x;
  PointVariant y_variant = y;
  PointVariant z_variant = z;

  // Extract the corresponding components from the variant
  auto extract_float = [](const auto & value) -> float { return static_cast<float>(value); };

  float pcl_x = std::visit(extract_float, x_variant);
  float pcl_y = std::visit(extract_float, y_variant);
  float pcl_z = std::visit(extract_float, z_variant);

  // Create a new pcl::PointXYZ object
  return {pcl_x, pcl_y, pcl_z};
}

void getNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * nearest_collision_point,
  rclcpp::Time * nearest_collision_point_time)
{
  double min_norm = 0.0;
  bool is_init = false;
  const auto yaw = getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));

  for (const auto & p : pointcloud) {
    const Eigen::Vector2d pointcloud_vec(p.x - base_pose.position.x, p.y - base_pose.position.y);
    double norm = base_pose_vec.dot(pointcloud_vec);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = p;
      *nearest_collision_point_time = pcl_conversions::fromPCL(pointcloud.header).stamp;
      is_init = true;
    }
  }
}

void getLateralNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * lateral_nearest_point,
  double * deviation)
{
  double min_norm = std::numeric_limits<double>::max();
  const auto yaw = getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    const Eigen::Vector2d pointcloud_vec(
      pointcloud.at(i).x - base_pose.position.x, pointcloud.at(i).y - base_pose.position.y);
    double norm =
      std::abs(base_pose_vec.x() * pointcloud_vec.y() - base_pose_vec.y() * pointcloud_vec.x());
    if (norm < min_norm) {
      min_norm = norm;
      *lateral_nearest_point = pointcloud.at(i);
    }
  }
  *deviation = min_norm;
}

double getNearestPointAndDistanceForPredictedObject(
  const geometry_msgs::msg::PoseArray & points, const Pose & base_pose,
  geometry_msgs::msg::Point * nearest_collision_point)
{
  double min_norm = 0.0;
  bool is_init = false;

  for (const auto & p : points.poses) {
    double norm = tier4_autoware_utils::calcDistance2d(p, base_pose);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = p.position;
      is_init = true;
    }
  }
  return min_norm;
}

void getLateralNearestPointForPredictedObject(
  const PoseArray & object, const Pose & base_pose, pcl::PointXYZ * lateral_nearest_point,
  double * deviation)
{
  double min_norm = std::numeric_limits<double>::max();
  for (const auto & pose : object.poses) {
    double norm = calcDistance2d(pose, base_pose);
    if (norm < min_norm) {
      min_norm = norm;
      *lateral_nearest_point = pointToPcl(pose.position.x, pose.position.y, base_pose.position.z);
    }
  }
  *deviation = min_norm;
}

Pose getVehicleCenterFromBase(const Pose & base_pose, const VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;
  const auto yaw = getRPY(base_pose).z;

  Pose center_pose;
  center_pose.position.x =
    base_pose.position.x + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::cos(yaw);
  center_pose.position.y =
    base_pose.position.y + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::sin(yaw);
  center_pose.position.z = base_pose.position.z;
  center_pose.orientation = base_pose.orientation;
  return center_pose;
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
  object_polygon = tier4_autoware_utils::isClockwise(object_polygon)
                     ? object_polygon
                     : tier4_autoware_utils::inverseClockwise(object_polygon);
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
  object_polygon = tier4_autoware_utils::isClockwise(object_polygon)
                     ? object_polygon
                     : tier4_autoware_utils::inverseClockwise(object_polygon);
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
  object_polygon = tier4_autoware_utils::isClockwise(object_polygon)
                     ? object_polygon
                     : tier4_autoware_utils::inverseClockwise(object_polygon);
  return object_polygon;
}

std::optional<PredictedObject> getObstacleFromUuid(
  const PredictedObjects & obstacles, const unique_identifier_msgs::msg::UUID & target_object_id)
{
  const auto itr = std::find_if(
    obstacles.objects.begin(), obstacles.objects.end(), [&](PredictedObject predicted_object) {
      return predicted_object.object_id == target_object_id;
    });

  if (itr == obstacles.objects.end()) {
    return std::nullopt;
  }
  return *itr;
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

}  // namespace motion_planning
