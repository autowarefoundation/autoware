// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__RESAMPLE__RESAMPLE_HPP_
#define AUTOWARE__MOTION_UTILS__RESAMPLE__RESAMPLE_HPP_

#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "tier4_planning_msgs/msg/path_with_lane_id.hpp"

#include <vector>

namespace autoware::motion_utils
{
/**
 * @brief A resampling function for a path(points). Note that in a default setting, position xy are
 *        resampled by spline interpolation, position z are resampled by linear interpolation, and
 *        orientation of the resampled path are calculated by a forward difference method
 *        based on the interpolated position x and y.
 * @param input_path input path(point) to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 *        point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @return resampled path(poses)
 */
std::vector<geometry_msgs::msg::Point> resamplePointVector(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy = false,
  const bool use_lerp_for_z = true);

/**
 * @brief A resampling function for a path(position). Note that in a default setting, position xy
 * are resampled by spline interpolation, position z are resampled by linear interpolation, and
 *        orientation of the resampled path are calculated by a forward difference method
 *        based on the interpolated position x and y.
 * @param input_path input path(position) to resample
 * @param resample_interval resampling interval
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @return resampled path(poses)
 */
std::vector<geometry_msgs::msg::Point> resamplePointVector(
  const std::vector<geometry_msgs::msg::Point> & points, const double resample_interval,
  const bool use_akima_spline_for_xy = false, const bool use_lerp_for_z = true);

/**
 * @brief A resampling function for a path(poses). Note that in a default setting, position xy are
 *        resampled by spline interpolation, position z are resampled by linear interpolation, and
 *        orientation of the resampled path are calculated by a forward difference method
 *        based on the interpolated position x and y.
 * @param input_path input path(poses) to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 *        point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @return resampled path(poses)
 */
std::vector<geometry_msgs::msg::Pose> resamplePoseVector(
  const std::vector<geometry_msgs::msg::Pose> & points,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy = false,
  const bool use_lerp_for_z = true);

/**
 * @brief A resampling function for a path(poses). Note that in a default setting, position xy are
 *        resampled by spline interpolation, position z are resampled by linear interpolation, and
 *        orientation of the resampled path are calculated by a forward difference method
 *        based on the interpolated position x and y.
 * @param input_path input path(poses) to resample
 * @param resample_interval resampling interval
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @return resampled path(poses)
 */
std::vector<geometry_msgs::msg::Pose> resamplePoseVector(
  const std::vector<geometry_msgs::msg::Pose> & points, const double resample_interval,
  const bool use_akima_spline_for_xy = false, const bool use_lerp_for_z = true);

/**
 * @brief A resampling function for a path with lane id. Note that in a default setting, position xy
 *        are resampled by spline interpolation, position z are resampled by linear interpolation,
 *        longitudinal and lateral velocity are resampled by zero_order_hold, and heading rate is
 *        resampled by linear interpolation. Orientation of the resampled path are calculated by a
 *        forward difference method based on the interpolated position x and y. Moreover, lane_ids
 *        and is_final are also interpolated by zero order hold
 * @param input_path input path to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 *        point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_v If true, it uses zero_order_hold to resample
 *        longitudinal and lateral velocity. Otherwise, it uses linear interpolation
 * @return resampled path
 */
tier4_planning_msgs::msg::PathWithLaneId resamplePath(
  const tier4_planning_msgs::msg::PathWithLaneId & input_path,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy = false,
  const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_v = true);

/**
 * @brief A resampling function for a path with lane id. Note that in a default setting, position xy
 *        are resampled by spline interpolation, position z are resampled by linear interpolation,
 *        longitudinal and lateral velocity are resampled by zero_order_hold, and heading rate is
 *        resampled by linear interpolation. Orientation of the resampled path are calculated by a
 *        forward difference method based on the interpolated position x and y. Moreover, lane_ids
 *        and is_final are also interpolated by zero order hold
 * @param input_path input path to resample
 * @param resampled_interval resampling interval point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_v If true, it uses zero_order_hold to resample
 *        longitudinal and lateral velocity. Otherwise, it uses linear interpolation
 * @param resample_input_path_stop_point If true, resample closest stop point in input path
 * @return resampled path
 */
tier4_planning_msgs::msg::PathWithLaneId resamplePath(
  const tier4_planning_msgs::msg::PathWithLaneId & input_path, const double resample_interval,
  const bool use_akima_spline_for_xy = false, const bool use_lerp_for_z = true,
  const bool use_zero_order_hold_for_v = true, const bool resample_input_path_stop_point = true);

/**
 * @brief A resampling function for a path. Note that in a default setting, position xy are
 *        resampled by spline interpolation, position z are resampled by linear interpolation,
 *        longitudinal and lateral velocity are resampled by zero_order_hold, and heading rate is
 *        resampled by linear interpolation. Orientation of the resampled path are calculated by a
 *        forward difference method based on the interpolated position x and y.
 * @param input_path input path to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 *        point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_v If true, it uses zero_order_hold to resample
 *        longitudinal and lateral velocity. Otherwise, it uses linear interpolation
 * @return resampled path
 */
autoware_planning_msgs::msg::Path resamplePath(
  const autoware_planning_msgs::msg::Path & input_path,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy = false,
  const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_v = true);

/**
 * @brief A resampling function for a path. Note that in a default setting, position xy
 *        are resampled by spline interpolation, position z are resampled by linear interpolation,
 *        longitudinal and lateral velocity are resampled by zero_order_hold, and heading rate is
 *        resampled by linear interpolation. Orientation of the resampled path are calculated by a
 *        forward difference method based on the interpolated position x and y.
 * @param input_path input path to resample
 * @param resampled_interval resampling interval point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_v If true, it uses zero_order_hold to resample
 *        longitudinal and lateral velocity. Otherwise, it uses linear interpolation
 * @param resample_input_path_stop_point If true, resample closest stop point in input path
 * @return resampled path
 */
autoware_planning_msgs::msg::Path resamplePath(
  const autoware_planning_msgs::msg::Path & input_path, const double resample_interval,
  const bool use_akima_spline_for_xy = false, const bool use_lerp_for_z = true,
  const bool use_zero_order_hold_for_twist = true,
  const bool resample_input_path_stop_point = true);

/**
 * @brief A resampling function for a trajectory. Note that in a default setting, position xy are
 *        resampled by spline interpolation, position z are resampled by linear interpolation, twist
 *        informaiton(velocity and acceleration) are resampled by zero_order_hold, and heading rate
 *        is resampled by linear interpolation. The rest of the category is resampled by linear
 *        interpolation. Orientation of the resampled path are calculated by a forward difference
 *        method based on the interpolated position x and y.
 * @param input_trajectory input trajectory to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 *        point
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_twist If true, it uses zero_order_hold to resample
 *        longitudinal, lateral velocity and acceleration. Otherwise, it uses linear interpolation
 * @return resampled trajectory
 */
autoware_planning_msgs::msg::Trajectory resampleTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const std::vector<double> & resampled_arclength, const bool use_akima_spline_for_xy = false,
  const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_twist = true);

/**
 * @brief A resampling function for a trajectory. This function resamples closest stop point,
 *        terminal point and points by resample interval. Note that in a default setting, position
 * xy are resampled by spline interpolation, position z are resampled by linear interpolation, twist
 *        informaiton(velocity and acceleration) are resampled by zero_order_hold, and heading rate
 *        is resampled by linear interpolation. The rest of the category is resampled by linear
 *        interpolation. Orientation of the resampled path are calculated by a forward difference
 *        method based on the interpolated position x and y.
 * @param input_trajectory input trajectory to resample
 * @param resampled_interval resampling interval
 * @param use_akima_spline_for_xy If true, it uses linear interpolation to resample position x and
 *        y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If true, it uses linear interpolation to resample position z.
 *        Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_twist If true, it uses zero_order_hold to resample
 *        longitudinal, lateral velocity and acceleration. Otherwise, it uses linear interpolation
 * @param resample_input_trajectory_stop_point If true, resample closest stop point in input
 *        trajectory
 * @return resampled trajectory
 */
autoware_planning_msgs::msg::Trajectory resampleTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double resample_interval,
  const bool use_akima_spline_for_xy = false, const bool use_lerp_for_z = true,
  const bool use_zero_order_hold_for_twist = true,
  const bool resample_input_trajectory_stop_point = true);
}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__RESAMPLE__RESAMPLE_HPP_
