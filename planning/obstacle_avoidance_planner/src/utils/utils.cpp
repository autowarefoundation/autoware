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

#include "obstacle_avoidance_planner/utils/utils.hpp"

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "tf2/utils.h"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "boost/optional.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <stack>
#include <vector>

namespace
{
std::vector<double> convertEulerAngleToMonotonic(const std::vector<double> & angle)
{
  if (angle.empty()) {
    return std::vector<double>{};
  }

  std::vector<double> monotonic_angle{angle.front()};
  for (size_t i = 1; i < angle.size(); ++i) {
    const double diff_angle = angle.at(i) - monotonic_angle.back();
    monotonic_angle.push_back(
      monotonic_angle.back() + tier4_autoware_utils::normalizeRadian(diff_angle));
  }

  return monotonic_angle;
}

std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y)
{
  if (x.size() != y.size()) {
    std::cerr << "x y vector size should be the same." << std::endl;
  }

  std::vector<double> dist_v;
  dist_v.push_back(0.0);
  for (unsigned int i = 0; i < x.size() - 1; ++i) {
    const double dx = x.at(i + 1) - x.at(i);
    const double dy = y.at(i + 1) - y.at(i);
    dist_v.push_back(dist_v.at(i) + std::hypot(dx, dy));
  }

  return dist_v;
}

std::array<std::vector<double>, 3> validateTrajectoryPoints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points)
{
  constexpr double epsilon = 1e-6;

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  for (size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      if (
        std::fabs(points[i].pose.position.x - points[i - 1].pose.position.x) < epsilon &&
        std::fabs(points[i].pose.position.y - points[i - 1].pose.position.y) < epsilon) {
        continue;
      }
    }
    x.push_back(points[i].pose.position.x);
    y.push_back(points[i].pose.position.y);
    yaw.push_back(tf2::getYaw(points[i].pose.orientation));
  }
  return {x, y, yaw};
}

std::array<std::vector<double>, 2> validatePoints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points)
{
  std::vector<double> x;
  std::vector<double> y;
  for (size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      if (
        std::fabs(points[i].pose.position.x - points[i - 1].pose.position.x) < 1e-6 &&
        std::fabs(points[i].pose.position.y - points[i - 1].pose.position.y) < 1e-6) {
        continue;
      }
    }
    x.push_back(points[i].pose.position.x);
    y.push_back(points[i].pose.position.y);
  }
  return {x, y};
}

// only two points is supported
std::vector<double> splineTwoPoints(
  std::vector<double> base_s, std::vector<double> base_x, const double begin_diff,
  const double end_diff, std::vector<double> new_s)
{
  const double h = base_s.at(1) - base_s.at(0);

  const double c = begin_diff;
  const double d = base_x.at(0);
  const double a = (end_diff * h - 2 * base_x.at(1) + c * h + 2 * d) / std::pow(h, 3);
  const double b = (3 * base_x.at(1) - end_diff * h - 2 * c * h - 3 * d) / std::pow(h, 2);

  std::vector<double> res;
  for (const auto & s : new_s) {
    const double ds = s - base_s.at(0);
    res.push_back(d + (c + (b + a * ds) * ds) * ds);
  }

  return res;
}
}  // namespace

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const ReferencePoint & p)
{
  return p.p;
}

template <>
geometry_msgs::msg::Pose getPose(const ReferencePoint & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position = p.p;
  pose.orientation = createQuaternionFromYaw(p.yaw);
  return pose;
}
}  // namespace tier4_autoware_utils

namespace geometry_utils
{
geometry_msgs::msg::Point transformToAbsoluteCoordinate2D(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
  // NOTE: implement transformation without defining yaw variable
  //       but directly sin/cos of yaw for fast calculation
  const auto & q = origin.orientation;
  const double cos_yaw = 1 - 2 * q.z * q.z;
  const double sin_yaw = 2 * q.w * q.z;

  geometry_msgs::msg::Point absolute_p;
  absolute_p.x = point.x * cos_yaw - point.y * sin_yaw + origin.position.x;
  absolute_p.y = point.x * sin_yaw + point.y * cos_yaw + origin.position.y;
  absolute_p.z = point.z;

  return absolute_p;
}

geometry_msgs::msg::Quaternion getQuaternionFromPoints(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & a_root)
{
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(a_root, a);
  return tier4_autoware_utils::createQuaternionFromYaw(yaw);
}

geometry_msgs::msg::Quaternion getQuaternionFromPoints(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  const double dx = (8.0 * (p3.x - p2.x) - (p4.x - p1.x)) / 12.0;
  const double dy = (8.0 * (p3.y - p2.y) - (p4.y - p1.y)) / 12.0;
  const double yaw = std::atan2(dy, dx);

  return tier4_autoware_utils::createQuaternionFromYaw(yaw);
}

boost::optional<geometry_msgs::msg::Point> transformMapToOptionalImage(
  const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & occupancy_grid_info)
{
  const geometry_msgs::msg::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  const double resolution = occupancy_grid_info.resolution;
  const double map_y_height = occupancy_grid_info.height;
  const double map_x_width = occupancy_grid_info.width;
  const double map_x_in_image_resolution = relative_p.x / resolution;
  const double map_y_in_image_resolution = relative_p.y / resolution;
  const double image_x = map_y_height - map_y_in_image_resolution;
  const double image_y = map_x_width - map_x_in_image_resolution;
  if (
    image_x >= 0 && image_x < static_cast<int>(map_y_height) && image_y >= 0 &&
    image_y < static_cast<int>(map_x_width)) {
    geometry_msgs::msg::Point image_point;
    image_point.x = image_x;
    image_point.y = image_y;
    return image_point;
  } else {
    return boost::none;
  }
}

bool transformMapToImage(
  const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & occupancy_grid_info, geometry_msgs::msg::Point & image_point)
{
  geometry_msgs::msg::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  const double map_y_height = occupancy_grid_info.height;
  const double map_x_width = occupancy_grid_info.width;
  const double scale = 1 / occupancy_grid_info.resolution;
  const double map_x_in_image_resolution = relative_p.x * scale;
  const double map_y_in_image_resolution = relative_p.y * scale;
  const double image_x = map_y_height - map_y_in_image_resolution;
  const double image_y = map_x_width - map_x_in_image_resolution;
  if (
    image_x >= 0 && image_x < static_cast<int>(map_y_height) && image_y >= 0 &&
    image_y < static_cast<int>(map_x_width)) {
    image_point.x = image_x;
    image_point.y = image_y;
    return true;
  } else {
    return false;
  }
}
}  // namespace geometry_utils

namespace interpolation_utils
{
std::vector<geometry_msgs::msg::Point> interpolate2DPoints(
  const std::vector<double> & base_x, const std::vector<double> & base_y, const double resolution,
  const double offset = 0.0)
{
  if (base_x.empty() || base_y.empty()) {
    return std::vector<geometry_msgs::msg::Point>{};
  }
  const std::vector<double> base_s = calcEuclidDist(base_x, base_y);
  if (base_s.empty() || base_s.size() == 1) {
    return std::vector<geometry_msgs::msg::Point>{};
  }

  std::vector<double> new_s;
  for (double i = offset; i < base_s.back() - 1e-6; i += resolution) {
    new_s.push_back(i);
  }
  if (new_s.empty()) {
    return std::vector<geometry_msgs::msg::Point>{};
  }

  // spline interpolation
  const std::vector<double> interpolated_x = interpolation::spline(base_s, base_x, new_s);
  const std::vector<double> interpolated_y = interpolation::spline(base_s, base_y, new_s);
  for (size_t i = 0; i < interpolated_x.size(); ++i) {
    if (std::isnan(interpolated_x[i]) || std::isnan(interpolated_y[i])) {
      return std::vector<geometry_msgs::msg::Point>{};
    }
  }

  std::vector<geometry_msgs::msg::Point> interpolated_points;
  for (size_t i = 0; i < interpolated_x.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = interpolated_x[i];
    point.y = interpolated_y[i];
    interpolated_points.push_back(point);
  }

  return interpolated_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> interpolateConnected2DPoints(
  const std::vector<double> & base_x, const std::vector<double> & base_y, const double resolution,
  const double begin_yaw, const double end_yaw)
{
  if (base_x.empty() || base_y.empty()) {
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  std::vector<double> base_s = calcEuclidDist(base_x, base_y);
  if (base_s.empty() || base_s.size() == 1) {
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  std::vector<double> new_s;
  for (double i = 0.0; i < base_s.back() - 1e-6; i += resolution) {
    new_s.push_back(i);
  }

  // spline interpolation
  const auto interpolated_x =
    splineTwoPoints(base_s, base_x, std::cos(begin_yaw), std::cos(end_yaw), new_s);
  const auto interpolated_y =
    splineTwoPoints(base_s, base_y, std::sin(begin_yaw), std::sin(end_yaw), new_s);

  for (size_t i = 0; i < interpolated_x.size(); i++) {
    if (std::isnan(interpolated_x[i]) || std::isnan(interpolated_y[i])) {
      return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
    }
  }

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> interpolated_points;
  for (size_t i = 0; i < interpolated_x.size(); i++) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = interpolated_x[i];
    point.pose.position.y = interpolated_y[i];

    const size_t front_idx = (i == interpolated_x.size() - 1) ? i - 1 : i;
    const double dx = interpolated_x[front_idx + 1] - interpolated_x[front_idx];
    const double dy = interpolated_y[front_idx + 1] - interpolated_y[front_idx];
    const double yaw = std::atan2(dy, dx);
    point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);

    interpolated_points.push_back(point);
  }

  return interpolated_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> interpolate2DTrajectoryPoints(
  const std::vector<double> & base_x, const std::vector<double> & base_y,
  const std::vector<double> & base_yaw, const double resolution)
{
  if (base_x.empty() || base_y.empty() || base_yaw.empty()) {
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  std::vector<double> base_s = calcEuclidDist(base_x, base_y);
  if (base_s.empty() || base_s.size() == 1) {
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  std::vector<double> new_s;
  for (double i = 0.0; i < base_s.back() - 1e-6; i += resolution) {
    new_s.push_back(i);
  }

  const auto monotonic_base_yaw = convertEulerAngleToMonotonic(base_yaw);

  // spline interpolation
  const auto interpolated_x = interpolation::spline(base_s, base_x, new_s);
  const auto interpolated_y = interpolation::spline(base_s, base_y, new_s);
  const auto interpolated_yaw = interpolation::spline(base_s, monotonic_base_yaw, new_s);

  for (size_t i = 0; i < interpolated_x.size(); i++) {
    if (std::isnan(interpolated_x[i]) || std::isnan(interpolated_y[i])) {
      return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
    }
  }

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> interpolated_points;
  for (size_t i = 0; i < interpolated_x.size(); i++) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = interpolated_x[i];
    point.pose.position.y = interpolated_y[i];
    point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(
      tier4_autoware_utils::normalizeRadian(interpolated_yaw[i]));
    interpolated_points.push_back(point);
  }

  return interpolated_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> interpolate2DTrajectoryPoints(
  const std::vector<double> & base_x, const std::vector<double> & base_y, const double resolution)
{
  if (base_x.empty() || base_y.empty()) {
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  std::vector<double> base_s = calcEuclidDist(base_x, base_y);
  if (base_s.empty() || base_s.size() == 1) {
    return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
  }
  std::vector<double> new_s;
  for (double i = 0.0; i < base_s.back() - 1e-6; i += resolution) {
    new_s.push_back(i);
  }

  // spline interpolation
  //  x = interpolated[0], y = interpolated[1], yaw = interpolated[2]
  std::array<std::vector<double>, 3> interpolated =
    interpolation::slerp2dFromXY(base_s, base_x, base_y, new_s);
  const auto & interpolated_x = interpolated[0];
  const auto & interpolated_y = interpolated[1];
  const auto & interpolated_yaw = interpolated[2];

  for (size_t i = 0; i < interpolated_x.size(); i++) {
    if (std::isnan(interpolated_x[i]) || std::isnan(interpolated_y[i])) {
      return std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>{};
    }
  }

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> interpolated_points;
  for (size_t i = 0; i < interpolated_x.size(); i++) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = interpolated_x[i];
    point.pose.position.y = interpolated_y[i];
    point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(
      tier4_autoware_utils::normalizeRadian(interpolated_yaw[i]));
    interpolated_points.push_back(point);
  }

  return interpolated_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> getInterpolatedTrajectoryPoints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const double delta_arc_length)
{
  std::array<std::vector<double>, 3> validated_pose = validateTrajectoryPoints(points);
  return interpolation_utils::interpolate2DTrajectoryPoints(
    validated_pose.at(0), validated_pose.at(1), delta_arc_length);
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> getConnectedInterpolatedPoints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const double delta_arc_length, const double begin_yaw, const double end_yaw)
{
  std::array<std::vector<double>, 2> validated_pose = validatePoints(points);
  return interpolation_utils::interpolateConnected2DPoints(
    validated_pose.at(0), validated_pose.at(1), delta_arc_length, begin_yaw, end_yaw);
}
}  // namespace interpolation_utils

namespace points_utils
{
// functions to convert to another type of points
std::vector<geometry_msgs::msg::Pose> convertToPosesWithYawEstimation(
  const std::vector<geometry_msgs::msg::Point> points)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  if (points.empty()) {
    return poses;
  } else if (points.size() == 1) {
    geometry_msgs::msg::Pose pose;
    pose.position = points.at(0);
    poses.push_back(pose);
    return poses;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position = points.at(i);

    const size_t front_idx = (i == points.size() - 1) ? i - 1 : i;
    const double points_yaw =
      tier4_autoware_utils::calcAzimuthAngle(points.at(front_idx), points.at(front_idx + 1));
    pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(points_yaw);

    poses.push_back(pose);
  }
  return poses;
}

template <typename T>
ReferencePoint convertToReferencePoint(const T & point)
{
  ReferencePoint ref_point;

  const auto & pose = tier4_autoware_utils::getPose(point);
  ref_point.p = pose.position;
  ref_point.yaw = tf2::getYaw(pose.orientation);

  return ref_point;
}

template ReferencePoint convertToReferencePoint<autoware_auto_planning_msgs::msg::TrajectoryPoint>(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & point);
template ReferencePoint convertToReferencePoint<geometry_msgs::msg::Pose>(
  const geometry_msgs::msg::Pose & point);
template <>
ReferencePoint convertToReferencePoint(const geometry_msgs::msg::Point & point)
{
  ReferencePoint ref_point;

  ref_point.p = point;

  return ref_point;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> concatTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & extended_traj_points)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> trajectory;
  trajectory.insert(trajectory.end(), traj_points.begin(), traj_points.end());
  trajectory.insert(trajectory.end(), extended_traj_points.begin(), extended_traj_points.end());
  return trajectory;
}

void compensateLastPose(
  const autoware_auto_planning_msgs::msg::PathPoint & last_path_point,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const double delta_dist_threshold, const double delta_yaw_threshold)
{
  if (traj_points.empty()) {
    traj_points.push_back(convertToTrajectoryPoint(last_path_point));
    return;
  }

  const geometry_msgs::msg::Pose last_traj_pose = traj_points.back().pose;

  const double dist =
    tier4_autoware_utils::calcDistance2d(last_path_point.pose.position, last_traj_pose.position);
  const double norm_diff_yaw = [&]() {
    const double diff_yaw =
      tf2::getYaw(last_path_point.pose.orientation) - tf2::getYaw(last_traj_pose.orientation);
    return tier4_autoware_utils::normalizeRadian(diff_yaw);
  }();
  if (dist > delta_dist_threshold || std::fabs(norm_diff_yaw) > delta_yaw_threshold) {
    traj_points.push_back(convertToTrajectoryPoint(last_path_point));
  }
}

int getNearestIdx(
  const std::vector<ReferencePoint> & points, const double target_s, const int begin_idx)
{
  double nearest_delta_s = std::numeric_limits<double>::max();
  int nearest_idx = begin_idx;
  for (size_t i = begin_idx; i < points.size(); i++) {
    double diff = std::fabs(target_s - points[i].s);
    if (diff < nearest_delta_s) {
      nearest_delta_s = diff;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}
}  // namespace points_utils

namespace utils
{
void logOSQPSolutionStatus(const int solution_status, const std::string & msg)
{
  /******************
   * Solver Status  *
   ******************/
  const int LOCAL_OSQP_SOLVED = 1;
  const int LOCAL_OSQP_SOLVED_INACCURATE = 2;
  const int LOCAL_OSQP_MAX_ITER_REACHED = -2;
  const int LOCAL_OSQP_PRIMAL_INFEASIBLE = -3;
  const int LOCAL_OSQP_PRIMAL_INFEASIBLE_INACCURATE = 3;
  const int LOCAL_OSQP_DUAL_INFEASIBLE = -4;
  const int LOCAL_OSQP_DUAL_INFEASIBLE_INACCURATE = 4;
  const int LOCAL_OSQP_SIGINT = -5;
  const int LOCAL_OSQP_TIME_LIMIT_REACHED = -6;
  const int LOCAL_OSQP_UNSOLVED = -10;
  const int LOCAL_OSQP_NON_CVX = -7;

  if (solution_status == LOCAL_OSQP_SOLVED) {
  } else if (solution_status == LOCAL_OSQP_DUAL_INFEASIBLE_INACCURATE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"),
      "[Avoidance] %s OSQP solution status: OSQP_DUAL_INFEASIBLE_INACCURATE", msg.c_str());
  } else if (solution_status == LOCAL_OSQP_PRIMAL_INFEASIBLE_INACCURATE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"),
      "[Avoidance] %s OSQP solution status: OSQP_PRIMAL_INFEASIBLE_INACCURATE", msg.c_str());
  } else if (solution_status == LOCAL_OSQP_SOLVED_INACCURATE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_SOLVED_INACCURATE",
      msg.c_str());
  } else if (solution_status == LOCAL_OSQP_MAX_ITER_REACHED) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_ITER_REACHED",
      msg.c_str());
  } else if (solution_status == LOCAL_OSQP_PRIMAL_INFEASIBLE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_PRIMAL_INFEASIBLE",
      msg.c_str());
  } else if (solution_status == LOCAL_OSQP_DUAL_INFEASIBLE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_DUAL_INFEASIBLE",
      msg.c_str());
  } else if (solution_status == LOCAL_OSQP_SIGINT) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_SIGINT", msg.c_str());
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] Interrupted by user, process will be finished.");
    std::exit(0);
  } else if (solution_status == LOCAL_OSQP_TIME_LIMIT_REACHED) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_TIME_LIMIT_REACHED",
      msg.c_str());
  } else if (solution_status == LOCAL_OSQP_UNSOLVED) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_UNSOLVED",
      msg.c_str());
  } else if (solution_status == LOCAL_OSQP_NON_CVX) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: OSQP_NON_CVX", msg.c_str());
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] %s OSQP solution status: Not defined %d",
      msg.c_str(), solution_status);
  }
}
}  // namespace utils
