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

#include "obstacle_avoidance_planner/utils/trajectory_utils.hpp"

#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "obstacle_avoidance_planner/utils/geometry_utils.hpp"
#include "tf2/utils.h"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <stack>
#include <vector>

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const obstacle_avoidance_planner::ReferencePoint & p)
{
  return p.pose.position;
}

template <>
geometry_msgs::msg::Pose getPose(const obstacle_avoidance_planner::ReferencePoint & p)
{
  return p.pose;
}

template <>
double getLongitudinalVelocity(const obstacle_avoidance_planner::ReferencePoint & p)
{
  return p.longitudinal_velocity_mps;
}
}  // namespace tier4_autoware_utils

namespace obstacle_avoidance_planner
{
namespace trajectory_utils
{
ReferencePoint convertToReferencePoint(const TrajectoryPoint & traj_point)
{
  ReferencePoint ref_point;

  ref_point.pose = traj_point.pose;
  ref_point.longitudinal_velocity_mps = traj_point.longitudinal_velocity_mps;
  return ref_point;
}

std::vector<ReferencePoint> convertToReferencePoints(
  const std::vector<TrajectoryPoint> & traj_points)
{
  std::vector<ReferencePoint> ref_points;
  for (const auto & traj_point : traj_points) {
    const auto ref_point = convertToReferencePoint(traj_point);
    ref_points.push_back(ref_point);
  }

  return ref_points;
}

void compensateLastPose(
  const PathPoint & last_path_point, std::vector<TrajectoryPoint> & traj_points,
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

geometry_msgs::msg::Point getNearestPosition(
  const std::vector<ReferencePoint> & points, const int target_idx, const double offset)
{
  double sum_arc_length = 0.0;
  for (size_t i = target_idx; i < points.size(); ++i) {
    sum_arc_length += points.at(i).delta_arc_length;

    if (offset < sum_arc_length) {
      return points.at(i).pose.position;
    }
  }

  return points.back().pose.position;
}

Trajectory createTrajectory(
  const std_msgs::msg::Header & header, const std::vector<TrajectoryPoint> & traj_points)
{
  auto traj = motion_utils::convertToTrajectory(traj_points);
  traj.header = header;

  return traj;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> traj_points, const double interval)
{
  constexpr bool enable_resampling_stop_point = true;

  const auto traj = motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = motion_utils::resampleTrajectory(
    traj, interval, false, true, true, enable_resampling_stop_point);
  return motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

// NOTE: stop point will not be resampled
std::vector<TrajectoryPoint> resampleTrajectoryPointsWithoutStopPoint(
  const std::vector<TrajectoryPoint> traj_points, const double interval)
{
  constexpr bool enable_resampling_stop_point = false;

  const auto traj = motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = motion_utils::resampleTrajectory(
    traj, interval, false, true, true, enable_resampling_stop_point);
  return motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

std::vector<ReferencePoint> resampleReferencePoints(
  const std::vector<ReferencePoint> ref_points, const double interval)
{
  // resample pose and velocity
  const auto traj_points = convertToTrajectoryPoints(ref_points);
  const auto resampled_traj_points =
    resampleTrajectoryPointsWithoutStopPoint(traj_points, interval);
  const auto resampled_ref_points = convertToReferencePoints(resampled_traj_points);

  // resample curvature
  std::vector<double> base_keys;
  std::vector<double> base_values;
  for (size_t i = 0; i < ref_points.size(); ++i) {
    if (i == 0) {
      base_keys.push_back(0.0);
    } else {
      const double delta_arc_length =
        tier4_autoware_utils::calcDistance2d(ref_points.at(i), ref_points.at(i - 1));
      base_keys.push_back(base_keys.back() + delta_arc_length);
    }

    base_values.push_back(ref_points.at(i).curvature);
  }

  std::vector<double> query_keys;
  for (size_t i = 0; i < resampled_ref_points.size(); ++i) {
    if (i == 0) {
      query_keys.push_back(0.0);
    } else {
      const double delta_arc_length = tier4_autoware_utils::calcDistance2d(
        resampled_ref_points.at(i), resampled_ref_points.at(i - 1));
      const double key = query_keys.back() + delta_arc_length;
      if (base_keys.back() < key) {
        break;
      }

      query_keys.push_back(key);
    }
  }

  if (query_keys.size() != resampled_ref_points.size()) {
    // compensate last key
    constexpr double epsilon = 1e-6;
    query_keys.push_back(base_keys.back() - epsilon);
  }

  const auto query_values = interpolation::lerp(base_keys, base_values, query_keys);

  // create output reference points by updating curvature with resampled one
  std::vector<ReferencePoint> output_ref_points;
  for (size_t i = 0; i < query_values.size(); ++i) {
    output_ref_points.push_back(resampled_ref_points.at(i));
    output_ref_points.at(i).curvature = query_values.at(i);
  }

  return output_ref_points;
}

void insertStopPoint(
  std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & input_stop_pose,
  const size_t stop_seg_idx)
{
  const double offset_to_segment = motion_utils::calcLongitudinalOffsetToSegment(
    traj_points, stop_seg_idx, input_stop_pose.position);

  const auto traj_spline = SplineInterpolationPoints2d(traj_points);
  const auto stop_pose = traj_spline.getSplineInterpolatedPose(stop_seg_idx, offset_to_segment);

  if (geometry_utils::isSamePoint(traj_points.at(stop_seg_idx), stop_pose)) {
    traj_points.at(stop_seg_idx).longitudinal_velocity_mps = 0.0;
    return;
  }
  if (geometry_utils::isSamePoint(traj_points.at(stop_seg_idx + 1), stop_pose)) {
    traj_points.at(stop_seg_idx + 1).longitudinal_velocity_mps = 0.0;
    return;
  }

  TrajectoryPoint additional_traj_point;
  additional_traj_point.pose = stop_pose;
  additional_traj_point.longitudinal_velocity_mps = 0.0;

  traj_points.insert(traj_points.begin() + stop_seg_idx + 1, additional_traj_point);
}
}  // namespace trajectory_utils
}  // namespace obstacle_avoidance_planner
