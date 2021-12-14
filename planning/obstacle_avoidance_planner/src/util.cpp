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

#include "obstacle_avoidance_planner/util.hpp"

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"

#include <interpolation/spline_interpolation.hpp>

#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <boost/optional.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <stack>
#include <vector>

namespace util
{
template <typename T>
geometry_msgs::msg::Point transformToRelativeCoordinate2D(
  const T & point, const geometry_msgs::msg::Pose & origin)
{
  // NOTE: implement transformation without defining yaw variable
  //       but directly sin/cos of yaw for fast calculation
  const auto & q = origin.orientation;
  const double cos_yaw = 1 - 2 * q.z * q.z;
  const double sin_yaw = 2 * q.w * q.z;

  geometry_msgs::msg::Point relative_p;
  const double tmp_x = point.x - origin.position.x;
  const double tmp_y = point.y - origin.position.y;
  relative_p.x = tmp_x * cos_yaw + tmp_y * sin_yaw;
  relative_p.y = -tmp_x * sin_yaw + tmp_y * cos_yaw;
  relative_p.z = point.z;

  return relative_p;
}

template geometry_msgs::msg::Point transformToRelativeCoordinate2D<geometry_msgs::msg::Point>(
  const geometry_msgs::msg::Point &, const geometry_msgs::msg::Pose & origin);
template geometry_msgs::msg::Point transformToRelativeCoordinate2D<geometry_msgs::msg::Point32>(
  const geometry_msgs::msg::Point32 &, const geometry_msgs::msg::Pose & origin);

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

double calculate2DDistance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calculateSquaredDistance(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double getYawFromPoints(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & a_root)
{
  const double dx = a.x - a_root.x;
  const double dy = a.y - a_root.y;
  return std::atan2(dy, dx);
}

geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

double normalizeRadian(const double angle)
{
  double n_angle = std::fmod(angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  return n_angle;
}

geometry_msgs::msg::Quaternion getQuaternionFromPoints(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & a_root)
{
  const double roll = 0;
  const double pitch = 0;
  const double yaw = util::getYawFromPoints(a, a_root);
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  return tf2::toMsg(quaternion);
}

template <typename T>
geometry_msgs::msg::Point transformMapToImage(
  const T & map_point, const nav_msgs::msg::MapMetaData & occupancy_grid_info)
{
  geometry_msgs::msg::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x / resolution;
  double map_y_in_image_resolution = relative_p.y / resolution;
  geometry_msgs::msg::Point image_point;
  image_point.x = map_y_height - map_y_in_image_resolution;
  image_point.y = map_x_width - map_x_in_image_resolution;
  return image_point;
}
template geometry_msgs::msg::Point transformMapToImage<geometry_msgs::msg::Point>(
  const geometry_msgs::msg::Point &, const nav_msgs::msg::MapMetaData & map_info);
template geometry_msgs::msg::Point transformMapToImage<geometry_msgs::msg::Point32>(
  const geometry_msgs::msg::Point32 &, const nav_msgs::msg::MapMetaData & map_info);

boost::optional<geometry_msgs::msg::Point> transformMapToOptionalImage(
  const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & occupancy_grid_info)
{
  geometry_msgs::msg::Point relative_p =
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x / resolution;
  double map_y_in_image_resolution = relative_p.y / resolution;
  double image_x = map_y_height - map_y_in_image_resolution;
  double image_y = map_x_width - map_x_in_image_resolution;
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

bool interpolate2DPoints(
  const std::vector<double> & base_x, const std::vector<double> & base_y, const double resolution,
  std::vector<geometry_msgs::msg::Point> & interpolated_points)
{
  if (base_x.empty() || base_y.empty()) {
    return false;
  }
  std::vector<double> base_s = calcEuclidDist(base_x, base_y);
  if (base_s.empty() || base_s.size() == 1) {
    return false;
  }
  std::vector<double> new_s;
  for (double i = 0.0; i < base_s.back() - 1e-6; i += resolution) {
    new_s.push_back(i);
  }

  // spline interpolation
  const std::vector<double> interpolated_x = interpolation::slerp(base_s, base_x, new_s);
  const std::vector<double> interpolated_y = interpolation::slerp(base_s, base_y, new_s);

  for (size_t i = 0; i < interpolated_x.size(); i++) {
    if (std::isnan(interpolated_x[i]) || std::isnan(interpolated_y[i])) {
      return false;
    }
  }
  for (size_t i = 0; i < interpolated_x.size(); i++) {
    geometry_msgs::msg::Point point;
    point.x = interpolated_x[i];
    point.y = interpolated_y[i];
    interpolated_points.push_back(point);
  }
  return true;
}

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Pose> & first_points,
  const std::vector<geometry_msgs::msg::Pose> & second_points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  std::vector<geometry_msgs::msg::Point> concat_points;
  for (const auto point : first_points) {
    concat_points.push_back(point.position);
  }
  for (const auto & point : second_points) {
    concat_points.push_back(point.position);
  }

  for (std::size_t i = 0; i < concat_points.size(); i++) {
    if (i > 0) {
      if (
        std::fabs(concat_points[i].x - concat_points[i - 1].x) < 1e-6 &&
        std::fabs(concat_points[i].y - concat_points[i - 1].y) < 1e-6) {
        continue;
      }
    }
    tmp_x.push_back(concat_points[i].x);
    tmp_y.push_back(concat_points[i].y);
  }
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Point> & points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (std::size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      if (
        std::fabs(points[i].x - points[i - 1].x) < 1e-6 &&
        std::fabs(points[i].y - points[i - 1].y) < 1e-6) {
        continue;
      }
    }
    tmp_x.push_back(points[i].x);
    tmp_y.push_back(points[i].y);
  }
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<geometry_msgs::msg::Pose> & points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (std::size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      if (
        std::fabs(points[i].position.x - points[i - 1].position.x) < 1e-6 &&
        std::fabs(points[i].position.y - points[i - 1].position.y) < 1e-6) {
        continue;
      }
    }
    tmp_x.push_back(points[i].position.x);
    tmp_y.push_back(points[i].position.y);
  }
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}

std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const std::vector<ReferencePoint> & points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (std::size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      if (
        std::fabs(points[i].p.x - points[i - 1].p.x) < 1e-6 &&
        std::fabs(points[i].p.y - points[i - 1].p.y) < 1e-6) {
        continue;
      }
    }
    tmp_x.push_back(points[i].p.x);
    tmp_y.push_back(points[i].p.y);
  }
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}

template <typename T>
std::vector<geometry_msgs::msg::Point> getInterpolatedPoints(
  const T & points, const double delta_arc_length)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (std::size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      if (
        std::fabs(points[i].pose.position.x - points[i - 1].pose.position.x) < 1e-6 &&
        std::fabs(points[i].pose.position.y - points[i - 1].pose.position.y) < 1e-6) {
        continue;
      }
    }
    tmp_x.push_back(points[i].pose.position.x);
    tmp_y.push_back(points[i].pose.position.y);
  }
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, interpolated_points);
  return interpolated_points;
}
template std::vector<geometry_msgs::msg::Point>
getInterpolatedPoints<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &, const double);
template std::vector<geometry_msgs::msg::Point>
getInterpolatedPoints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &, const double);

template <typename T>
int getNearestIdx(
  const T & points, const geometry_msgs::msg::Pose & pose, const int default_idx,
  const double delta_yaw_threshold)
{
  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = default_idx;
  const double point_yaw = tf2::getYaw(pose.orientation);
  for (std::size_t i = 0; i < points.size(); i++) {
    const double dist = calculateSquaredDistance(points[i].pose.position, pose.position);
    const double points_yaw = tf2::getYaw(points[i].pose.orientation);
    const double diff_yaw = points_yaw - point_yaw;
    const double norm_diff_yaw = normalizeRadian(diff_yaw);
    if (dist < min_dist && std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}
template int getNearestIdx<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
  const geometry_msgs::msg::Pose &, const int, const double);
template int getNearestIdx<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &,
  const geometry_msgs::msg::Pose &, const int, const double);

int getNearestIdx(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const int default_idx, const double delta_yaw_threshold, const double delta_dist_threshold)
{
  int nearest_idx = default_idx;
  double min_dist = std::numeric_limits<double>::max();
  const double point_yaw = tf2::getYaw(pose.orientation);
  for (std::size_t i = 0; i < points.size(); i++) {
    const double dist = calculateSquaredDistance(points[i], pose.position);
    double points_yaw = 0;
    if (i > 0) {
      const double dx = points[i].x - points[i - 1].x;
      const double dy = points[i].y - points[i - 1].y;
      points_yaw = std::atan2(dy, dx);
    } else if (i == 0 && points.size() > 1) {
      const double dx = points[i + 1].x - points[i].x;
      const double dy = points[i + 1].y - points[i].y;
      points_yaw = std::atan2(dy, dx);
    }
    const double diff_yaw = points_yaw - point_yaw;
    const double norm_diff_yaw = normalizeRadian(diff_yaw);
    if (
      dist < min_dist && dist < delta_dist_threshold &&
      std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

int getNearestIdxOverPoint(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, [[maybe_unused]] const int default_idx,
  const double delta_yaw_threshold)
{
  double min_dist = std::numeric_limits<double>::max();
  const double point_yaw = tf2::getYaw(pose.orientation);
  const double pose_dx = std::cos(point_yaw);
  const double pose_dy = std::sin(point_yaw);
  int nearest_idx = 0;
  for (std::size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      const double dist = util::calculateSquaredDistance(points[i].pose.position, pose.position);
      const double points_yaw =
        getYawFromPoints(points[i].pose.position, points[i - 1].pose.position);
      const double diff_yaw = points_yaw - point_yaw;
      const double norm_diff_yaw = normalizeRadian(diff_yaw);
      const double dx = points[i].pose.position.x - pose.position.x;
      const double dy = points[i].pose.position.y - pose.position.y;
      const double ip = dx * pose_dx + dy * pose_dy;
      if (dist < min_dist && ip > 0 && std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
        min_dist = dist;
        nearest_idx = i;
      }
    }
  }
  return nearest_idx;
}

int getNearestIdx(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Pose & pose,
  const int default_idx, const double delta_yaw_threshold)
{
  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = default_idx;
  const double point_yaw = tf2::getYaw(pose.orientation);
  for (std::size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      const double dist = calculateSquaredDistance(points[i], pose.position);
      const double points_yaw = getYawFromPoints(points[i], points[i - 1]);
      const double diff_yaw = points_yaw - point_yaw;
      const double norm_diff_yaw = normalizeRadian(diff_yaw);
      if (dist < min_dist && std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
        min_dist = dist;
        nearest_idx = i;
      }
    }
  }
  return nearest_idx;
}

template <typename T>
int getNearestIdx(const T & points, const geometry_msgs::msg::Point & point, const int default_idx)
{
  double min_dist = std::numeric_limits<double>::max();
  int nearest_idx = default_idx;
  for (std::size_t i = 0; i < points.size(); i++) {
    if (i > 0) {
      const double dist = calculateSquaredDistance(points[i - 1].pose.position, point);
      const double points_dx = points[i].pose.position.x - points[i - 1].pose.position.x;
      const double points_dy = points[i].pose.position.y - points[i - 1].pose.position.y;
      const double points2pose_dx = point.x - points[i - 1].pose.position.x;
      const double points2pose_dy = point.y - points[i - 1].pose.position.y;
      const double ip = points_dx * points2pose_dx + points_dy * points2pose_dy;
      if (ip < 0) {
        return nearest_idx;
      }
      if (dist < min_dist && ip > 0) {
        min_dist = dist;
        nearest_idx = i - 1;
      }
    }
  }
  return nearest_idx;
}
template int getNearestIdx<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & point, const int default_idx);
template int getNearestIdx<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & point, const int default_idx);

int getNearestIdx(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Point & point)
{
  int nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < points.size(); i++) {
    const double dist = calculateSquaredDistance(points[i], point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

template <typename T>
int getNearestIdx(const T & points, const geometry_msgs::msg::Point & point)
{
  int nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < points.size(); i++) {
    const double dist = calculateSquaredDistance(points[i].pose.position, point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}
template int getNearestIdx<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
  const geometry_msgs::msg::Point &);
template int getNearestIdx<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &,
  const geometry_msgs::msg::Point &);

int getNearestIdx(
  const std::vector<ReferencePoint> & points, const double target_s, const int begin_idx)
{
  double nearest_delta_s = std::numeric_limits<double>::max();
  int nearest_idx = begin_idx;
  for (std::size_t i = begin_idx; i < points.size(); i++) {
    double diff = std::fabs(target_s - points[i].s);
    if (diff < nearest_delta_s) {
      nearest_delta_s = diff;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

template <typename T>
int getNearestPointIdx(const T & points, const geometry_msgs::msg::Point & point)
{
  int nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < points.size(); i++) {
    const double dist = calculateSquaredDistance(points[i].p, point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}
template int getNearestPointIdx<std::vector<ReferencePoint>>(
  const std::vector<ReferencePoint> &, const geometry_msgs::msg::Point &);
template int getNearestPointIdx<std::vector<Footprint>>(
  const std::vector<Footprint> &, const geometry_msgs::msg::Point &);

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertPathToTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points;
  for (const auto & point : path_points) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_point;
    tmp_point.pose = point.pose;
    tmp_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    traj_points.push_back(tmp_point);
  }
  return traj_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
convertPointsToTrajectoryPointsWithYaw(const std::vector<geometry_msgs::msg::Point> & points)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points;
  for (size_t i = 0; i < points.size(); i++) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose.position = points[i];
    double yaw = 0;
    if (i > 0) {
      const double dx = points[i].x - points[i - 1].x;
      const double dy = points[i].y - points[i - 1].y;
      yaw = std::atan2(dy, dx);
    } else if (i == 0 && points.size() > 1) {
      const double dx = points[i + 1].x - points[i].x;
      const double dy = points[i + 1].y - points[i].y;
      yaw = std::atan2(dy, dx);
    }
    const double roll = 0;
    const double pitch = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    traj_points.push_back(traj_point);
  }
  return traj_points;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> fillTrajectoryWithVelocity(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & traj_points,
  const double velocity)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_with_velocity;
  for (const auto & traj_point : traj_points) {
    auto tmp_point = traj_point;
    tmp_point.longitudinal_velocity_mps = velocity;
    traj_with_velocity.push_back(tmp_point);
  }
  return traj_with_velocity;
}

template <typename T>
std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> alignVelocityWithPoints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & base_traj_points,
  const T & points, const int zero_velocity_traj_idx, const int max_skip_comparison_idx)
{
  auto traj_points = base_traj_points;
  int prev_begin_idx = 0;
  for (std::size_t i = 0; i < traj_points.size(); i++) {
    const auto first = points.begin() + prev_begin_idx;

    // TODO(Horibe) could be replaced end() with some reasonable number to reduce computational time
    const auto last = points.end();
    const T truncated_points(first, last);

    const size_t closest_seg_idx =
      tier4_autoware_utils::findNearestSegmentIndex(truncated_points, traj_points[i].pose.position);
    // TODO(murooka) implement calcSignedArcLength(points, idx, point)
    const double closest_to_target_dist = tier4_autoware_utils::calcSignedArcLength(
      truncated_points, truncated_points.at(closest_seg_idx).pose.position,
      traj_points[i].pose.position);
    const double seg_dist = tier4_autoware_utils::calcSignedArcLength(
      truncated_points, closest_seg_idx, closest_seg_idx + 1);

    // interpolate 1st-nearest (v1) value and 2nd-nearest value (v2)
    const auto lerp = [&](const double v1, const double v2, const double ratio) {
      return std::abs(seg_dist) < 1e-6 ? v2 : v1 + (v2 - v1) * ratio;
    };

    // z
    {
      const double closest_z = truncated_points.at(closest_seg_idx).pose.position.z;
      const double next_z = truncated_points.at(closest_seg_idx + 1).pose.position.z;
      traj_points[i].pose.position.z = lerp(closest_z, next_z, closest_to_target_dist / seg_dist);
    }

    // vx
    {
      const double closest_vel = truncated_points[closest_seg_idx].longitudinal_velocity_mps;
      const double next_vel = truncated_points[closest_seg_idx + 1].longitudinal_velocity_mps;
      const double target_vel = lerp(closest_vel, next_vel, closest_to_target_dist / seg_dist);

      if (static_cast<int>(i) >= zero_velocity_traj_idx) {
        traj_points[i].longitudinal_velocity_mps = 0;
      } else if (target_vel < 1e-6) {
        const auto idx = std::max(static_cast<int>(i) - 1, 0);
        traj_points[i].longitudinal_velocity_mps = traj_points[idx].longitudinal_velocity_mps;
      } else if (static_cast<int>(i) <= max_skip_comparison_idx) {
        traj_points[i].longitudinal_velocity_mps = target_vel;
      } else {
        traj_points[i].longitudinal_velocity_mps =
          std::fmin(target_vel, traj_points[i].longitudinal_velocity_mps);
      }
    }
    // NOTE: closest_seg_idx is for the clipped trajectory. This operation must be "+=".
    prev_begin_idx += closest_seg_idx;
  }
  return traj_points;
}
template std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
alignVelocityWithPoints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &, const int, const int);
template std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
alignVelocityWithPoints<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &, const int, const int);

std::vector<std::vector<int>> getHistogramTable(const std::vector<std::vector<int>> & input)
{
  std::vector<std::vector<int>> histogram_table = input;
  for (std::size_t i = 0; i < input.size(); i++) {
    for (std::size_t j = 0; j < input[i].size(); j++) {
      if (input[i][j]) {
        histogram_table[i][j] = 0;
      } else {
        histogram_table[i][j] = (i > 0) ? histogram_table[i - 1][j] + 1 : 1;
      }
    }
  }
  return histogram_table;
}

struct HistogramBin
{
  int height;
  int variable_pos;
  int original_pos;
};

Rectangle getLargestRectangleInRow(
  const std::vector<int> & histo, const int current_row, [[maybe_unused]] const int row_size)
{
  std::vector<int> search_histo = histo;
  search_histo.push_back(0);
  std::stack<HistogramBin> stack;
  Rectangle largest_rect;
  for (std::size_t i = 0; i < search_histo.size(); i++) {
    HistogramBin bin;
    bin.height = search_histo[i];
    bin.variable_pos = i;
    bin.original_pos = i;
    if (stack.empty()) {
      stack.push(bin);
    } else {
      if (stack.top().height < bin.height) {
        stack.push(bin);
      } else if (stack.top().height >= bin.height) {
        int target_i = i;
        while (!stack.empty() && bin.height <= stack.top().height) {
          HistogramBin tmp_bin = stack.top();
          stack.pop();
          int area = (i - tmp_bin.variable_pos) * tmp_bin.height;
          if (area > largest_rect.area) {
            largest_rect.max_y_idx = tmp_bin.variable_pos;
            largest_rect.min_y_idx = i - 1;
            largest_rect.max_x_idx = current_row - tmp_bin.height + 1;
            largest_rect.min_x_idx = current_row;
            largest_rect.area = area;
          }

          target_i = tmp_bin.variable_pos;
        }
        bin.variable_pos = target_i;
        stack.push(bin);
      }
    }
  }
  return largest_rect;
}

Rectangle getLargestRectangle(const std::vector<std::vector<int>> & input)
{
  std::vector<std::vector<int>> histogram_table = getHistogramTable(input);
  Rectangle largest_rectangle;
  for (std::size_t i = 0; i < histogram_table.size(); i++) {
    Rectangle rect = getLargestRectangleInRow(histogram_table[i], i, input.size());
    if (rect.area > largest_rectangle.area) {
      largest_rectangle = rect;
    }
  }
  return largest_rectangle;
}

boost::optional<geometry_msgs::msg::Point> getLastExtendedPoint(
  const autoware_auto_planning_msgs::msg::PathPoint & path_point,
  const geometry_msgs::msg::Pose & pose, const double delta_yaw_threshold,
  const double delta_dist_threshold)
{
  const double dist = calculate2DDistance(path_point.pose.position, pose.position);
  const double diff_yaw = tf2::getYaw(path_point.pose.orientation) - tf2::getYaw(pose.orientation);
  const double norm_diff_yaw = normalizeRadian(diff_yaw);
  if (
    dist > 1e-6 && dist < delta_dist_threshold && std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
    return path_point.pose.position;
  } else {
    return boost::none;
  }
}

boost::optional<autoware_auto_planning_msgs::msg::TrajectoryPoint> getLastExtendedTrajPoint(
  const autoware_auto_planning_msgs::msg::PathPoint & path_point,
  const geometry_msgs::msg::Pose & pose, const double delta_yaw_threshold,
  const double delta_dist_threshold)
{
  const double dist = calculate2DDistance(path_point.pose.position, pose.position);
  const double diff_yaw = tf2::getYaw(path_point.pose.orientation) - tf2::getYaw(pose.orientation);
  const double norm_diff_yaw = normalizeRadian(diff_yaw);
  if (
    dist > 1e-6 && dist < delta_dist_threshold && std::fabs(norm_diff_yaw) < delta_yaw_threshold) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint tmp_traj_p;
    tmp_traj_p.pose.position = path_point.pose.position;
    tmp_traj_p.pose.orientation =
      util::getQuaternionFromPoints(path_point.pose.position, pose.position);
    tmp_traj_p.longitudinal_velocity_mps = path_point.longitudinal_velocity_mps;
    return tmp_traj_p;
  } else {
    return boost::none;
  }
}

std::vector<Footprint> getVehicleFootprints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const VehicleParam & vehicle_param)
{
  const double baselink_to_top = vehicle_param.length - vehicle_param.rear_overhang;
  const double half_width = vehicle_param.width * 0.5;
  std::vector<double> rel_lon_offset{baselink_to_top, baselink_to_top, 0, 0};
  std::vector<double> rel_lat_offset{half_width, -half_width, -half_width, half_width};
  std::vector<Footprint> rects;
  for (std::size_t i = 0; i < optimized_points.size(); i++) {
    // for (int i = nearest_idx; i < optimized_points.size(); i++) {
    std::vector<geometry_msgs::msg::Point> abs_points;
    for (std::size_t j = 0; j < rel_lon_offset.size(); j++) {
      geometry_msgs::msg::Point rel_point;
      rel_point.x = rel_lon_offset[j];
      rel_point.y = rel_lat_offset[j];
      abs_points.push_back(
        util::transformToAbsoluteCoordinate2D(rel_point, optimized_points[i].pose));
    }
    Footprint rect;
    rect.p = optimized_points[i].pose.position;
    rect.top_left = abs_points[0];
    rect.top_right = abs_points[1];
    rect.bottom_right = abs_points[2];
    rect.bottom_left = abs_points[3];
    rects.push_back(rect);
  }
  return rects;
}

/*
 * calculate distance in x-y 2D space
 */
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

bool hasValidNearestPointFromEgo(
  const geometry_msgs::msg::Pose & ego_pose, const Trajectories & trajs,
  const TrajectoryParam & traj_param)
{
  const auto traj = concatTraj(trajs);
  const auto interpolated_points =
    util::getInterpolatedPoints(traj, traj_param.delta_arc_length_for_trajectory);
  const int default_idx = -1;
  const int nearest_idx_with_thres = util::getNearestIdx(
    interpolated_points, ego_pose, default_idx, traj_param.delta_yaw_threshold_for_closest_point,
    traj_param.delta_dist_threshold_for_closest_point);
  if (nearest_idx_with_thres == default_idx) {
    return false;
  }
  return true;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> concatTraj(
  const Trajectories & trajs)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> trajectory;
  trajectory.insert(
    trajectory.end(), trajs.model_predictive_trajectory.begin(),
    trajs.model_predictive_trajectory.end());
  trajectory.insert(
    trajectory.end(), trajs.extended_trajectory.begin(), trajs.extended_trajectory.end());
  return trajectory;
}

int getZeroVelocityIdx(
  const bool is_showing_debug_info, const std::vector<geometry_msgs::msg::Point> & fine_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & opt_trajs, const TrajectoryParam & traj_param)
{
  const int default_idx = fine_points.size() - 1;
  const int zero_velocity_idx_from_path =
    getZeroVelocityIdxFromPoints(path_points, fine_points, default_idx, traj_param);

  int zero_velocity_idx_from_opt_points = zero_velocity_idx_from_path;
  if (opt_trajs) {
    zero_velocity_idx_from_opt_points = getZeroVelocityIdxFromPoints(
      util::concatTraj(*opt_trajs), fine_points, default_idx, traj_param);
  }
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("util"), is_showing_debug_info,
    "0 m/s idx from path: %d from opt: %d total size %zu", zero_velocity_idx_from_path,
    zero_velocity_idx_from_opt_points, fine_points.size());
  const int zero_velocity_idx =
    std::min(zero_velocity_idx_from_path, zero_velocity_idx_from_opt_points);

  if (is_showing_debug_info) {
    int zero_velocity_path_idx = path_points.size() - 1;
    for (std::size_t i = 0; i < path_points.size(); i++) {
      if (path_points[i].longitudinal_velocity_mps < 1e-6) {
        zero_velocity_path_idx = i;
        break;
      }
    }
    const float debug_dist = util::calculate2DDistance(
      path_points[zero_velocity_path_idx].pose.position, fine_points[zero_velocity_idx]);
    RCLCPP_INFO(
      rclcpp::get_logger("util"), "Dist from path 0 velocity point: = %f [m]", debug_dist);
  }
  return zero_velocity_idx;
}

template <typename T>
int getZeroVelocityIdxFromPoints(
  const T & points, const std::vector<geometry_msgs::msg::Point> & fine_points,
  const int /* default_idx */, const TrajectoryParam & traj_param)
{
  int zero_velocity_points_idx = points.size() - 1;
  for (std::size_t i = 0; i < points.size(); i++) {
    if (points[i].longitudinal_velocity_mps < 1e-6) {
      zero_velocity_points_idx = i;
      break;
    }
  }
  const int default_zero_velocity_fine_points_idx = fine_points.size() - 1;
  const int zero_velocity_fine_points_idx = util::getNearestIdx(
    fine_points, points[zero_velocity_points_idx].pose, default_zero_velocity_fine_points_idx,
    traj_param.delta_yaw_threshold_for_closest_point,
    traj_param.delta_dist_threshold_for_closest_point);
  return zero_velocity_fine_points_idx;
}
template int getZeroVelocityIdxFromPoints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &,
  const std::vector<geometry_msgs::msg::Point> &, const int, const TrajectoryParam &);
template int
getZeroVelocityIdxFromPoints<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
  const std::vector<geometry_msgs::msg::Point> &, const int, const TrajectoryParam &);

template <typename T>
double getArcLength(const T & points, const int initial_idx)
{
  double accum_arc_length = 0;
  for (size_t i = initial_idx; i < points.size(); i++) {
    if (i > 0) {
      const double dist =
        util::calculate2DDistance(points[i].pose.position, points[i - 1].pose.position);
      accum_arc_length += dist;
    }
  }
  return accum_arc_length;
}
template double getArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &, const int);
template double getArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &, const int);

double getArcLength(const std::vector<geometry_msgs::msg::Pose> & points, const int initial_idx)
{
  double accum_arc_length = 0;
  for (size_t i = initial_idx; i < points.size(); i++) {
    if (i > 0) {
      const double dist = util::calculate2DDistance(points[i].position, points[i - 1].position);
      accum_arc_length += dist;
    }
  }
  return accum_arc_length;
}

void logOSQPSolutionStatus(const int solution_status)
{
  /******************
   * Solver Status  *
   ******************/
  int OSQP_DUAL_INFEASIBLE_INACCURATE = 4;
  int OSQP_PRIMAL_INFEASIBLE_INACCURATE = 3;
  int OSQP_SOLVED_INACCURATE = 2;
  int OSQP_SOLVED = 1;
  int OSQP_MAX_ITER_REACHED = -2;
  int OSQP_PRIMAL_INFEASIBLE = -3; /* primal infeasible  */
  int OSQP_DUAL_INFEASIBLE = -4;   /* dual infeasible */
  int OSQP_SIGINT = -5;            /* interrupted by user */

  if (solution_status == OSQP_SOLVED) {
    // RCLCPP_INFO(rclcpp::get_logger("util"),"[Avoidance] OSQP solution status: OSQP_SOLVED");
  } else if (solution_status == OSQP_DUAL_INFEASIBLE_INACCURATE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"),
      "[Avoidance] OSQP solution status: OSQP_DUAL_INFEASIBLE_INACCURATE");
  } else if (solution_status == OSQP_PRIMAL_INFEASIBLE_INACCURATE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"),
      "[Avoidance] OSQP solution status: OSQP_PRIMAL_INFEASIBLE_INACCURATE");
  } else if (solution_status == OSQP_SOLVED_INACCURATE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] OSQP solution status: OSQP_SOLVED_INACCURATE");
  } else if (solution_status == OSQP_MAX_ITER_REACHED) {
    RCLCPP_WARN(rclcpp::get_logger("util"), "[Avoidance] OSQP solution status: OSQP_ITER_REACHED");
  } else if (solution_status == OSQP_PRIMAL_INFEASIBLE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] OSQP solution status: OSQP_PRIMAL_INFEASIBLE");
  } else if (solution_status == OSQP_DUAL_INFEASIBLE) {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] OSQP solution status: OSQP_DUAL_INFEASIBLE");
  } else if (solution_status == OSQP_SIGINT) {
    RCLCPP_WARN(rclcpp::get_logger("util"), "[Avoidance] OSQP solution status: OSQP_SIGINT");
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] Interrupted by user, process will be finished.");
    std::exit(0);
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("util"), "[Avoidance] OSQP solution status: Not defined %d",
      solution_status);
  }
}

}  // namespace util
