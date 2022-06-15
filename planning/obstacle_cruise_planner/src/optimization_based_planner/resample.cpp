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

#include "obstacle_cruise_planner/optimization_based_planner/resample.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "obstacle_cruise_planner/utils.hpp"

#include <vector>

namespace resampling
{
std::vector<rclcpp::Duration> resampledValidRelativeTimeVector(
  const rclcpp::Time & start_time, const rclcpp::Time & obj_base_time,
  const std::vector<double> & rel_time_vec, const double duration)
{
  const auto prediction_duration = rclcpp::Duration::from_seconds(duration);
  const auto end_time = start_time + prediction_duration;

  // NOTE: rel_time_vec is relative time to start_time.
  //       rel_valid_time_vec is relative to obj_base_time, which is time stamp in predicted object.
  std::vector<rclcpp::Duration> rel_valid_time_vec;
  for (const auto & time : rel_time_vec) {
    // absolute target time
    const auto target_time = start_time + rclcpp::Duration::from_seconds(time);
    if (target_time > end_time) {
      break;
    }

    // relative target time
    const auto rel_target_time = target_time - obj_base_time;
    if (rel_target_time < rclcpp::Duration::from_seconds(0.0)) {
      continue;
    }

    rel_valid_time_vec.push_back(rel_target_time);
  }

  return rel_valid_time_vec;
}

autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & input_path,
  const std::vector<rclcpp::Duration> & rel_time_vec)
{
  autoware_auto_perception_msgs::msg::PredictedPath resampled_path;
  resampled_path.time_step = input_path.time_step;

  for (const auto & rel_time : rel_time_vec) {
    const auto opt_pose = obstacle_cruise_utils::lerpByTimeStamp(input_path, rel_time);
    if (!opt_pose) {
      continue;
    }

    resampled_path.path.push_back(opt_pose.get());
  }

  return resampled_path;
}

inline void convertEulerAngleToMonotonic(std::vector<double> & a)
{
  for (unsigned int i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + tier4_autoware_utils::normalizeRadian(da);
  }
}

autoware_auto_planning_msgs::msg::Trajectory applyLinearInterpolation(
  const std::vector<double> & base_index,
  const autoware_auto_planning_msgs::msg::Trajectory & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose)
{
  std::vector<double> px, py, pz, pyaw, tlx, taz, alx;
  for (const auto & p : base_trajectory.points) {
    px.push_back(p.pose.position.x);
    py.push_back(p.pose.position.y);
    pz.push_back(p.pose.position.z);
    pyaw.push_back(tf2::getYaw(p.pose.orientation));
    tlx.push_back(p.longitudinal_velocity_mps);
    taz.push_back(p.heading_rate_rps);
    alx.push_back(p.acceleration_mps2);
  }

  convertEulerAngleToMonotonic(pyaw);

  std::vector<double> px_p, py_p, pz_p, pyaw_p;
  if (use_spline_for_pose) {
    px_p = interpolation::slerp(base_index, px, out_index);
    py_p = interpolation::slerp(base_index, py, out_index);
    pz_p = interpolation::slerp(base_index, pz, out_index);
    pyaw_p = interpolation::slerp(base_index, pyaw, out_index);
  } else {
    px_p = interpolation::lerp(base_index, px, out_index);
    py_p = interpolation::lerp(base_index, py, out_index);
    pz_p = interpolation::lerp(base_index, pz, out_index);
    pyaw_p = interpolation::lerp(base_index, pyaw, out_index);
  }
  const auto tlx_p = interpolation::lerp(base_index, tlx, out_index);
  const auto taz_p = interpolation::lerp(base_index, taz, out_index);
  const auto alx_p = interpolation::lerp(base_index, alx, out_index);

  autoware_auto_planning_msgs::msg::Trajectory out_trajectory;
  out_trajectory.header = base_trajectory.header;
  autoware_auto_planning_msgs::msg::TrajectoryPoint point;
  for (unsigned int i = 0; i < out_index.size(); ++i) {
    point.pose.position.x = px_p.at(i);
    point.pose.position.y = py_p.at(i);
    point.pose.position.z = pz_p.at(i);
    point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(pyaw_p.at(i));

    point.longitudinal_velocity_mps = tlx_p.at(i);
    point.heading_rate_rps = taz_p.at(i);
    point.acceleration_mps2 = alx_p.at(i);
    out_trajectory.points.push_back(point);
  }
  return out_trajectory;
}
}  // namespace resampling
