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

#include "motion_velocity_smoother/resample.hpp"

#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

#include <algorithm>
#include <vector>

namespace motion_velocity_smoother
{
namespace resampling
{
TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const double v_current,
  const geometry_msgs::msg::Pose & current_pose, const double nearest_dist_threshold,
  const double nearest_yaw_threshold, const ResampleParam & param, const bool use_zoh_for_v)
{
  // Arc length from the initial point to the closest point
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double negative_front_arclength_value = motion_utils::calcSignedArcLength(
    input, current_pose.position, current_seg_idx, input.at(0).pose.position, 0);
  const auto front_arclength_value = std::fabs(negative_front_arclength_value);

  const auto dist_to_closest_stop_point =
    motion_utils::calcDistanceToForwardStopPoint(input, current_pose);

  // Get the resample size from the closest point
  const double trajectory_length = motion_utils::calcArcLength(input);
  const double Nt = param.resample_time / std::max(param.dense_resample_dt, 0.001);
  const double ds_nominal =
    std::max(v_current * param.dense_resample_dt, param.dense_min_interval_distance);
  const double Ns = param.min_trajectory_length / std::max(ds_nominal, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  constexpr double front_ds = 0.1;
  for (double ds = 0.0; ds <= front_arclength_value; ds += front_ds) {
    out_arclength.push_back(ds);
  }
  if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
    out_arclength.back() = front_arclength_value;
  } else {
    out_arclength.push_back(front_arclength_value);
  }

  // Step2. Resample behind trajectory
  double dist_i = 0.0;
  bool is_zero_point_included = false;
  bool is_endpoint_included = false;
  for (size_t i = 1; static_cast<double>(i) <= N; ++i) {
    double ds = ds_nominal;
    if (i > Nt) {
      // if the planning time is not enough to see the desired distance,
      // change the interval distance to see far.
      ds = std::max(param.sparse_min_interval_distance, param.sparse_resample_dt * v_current);
    }

    dist_i += ds;

    // Check if the distance is longer than max_trajectory_length
    if (dist_i > param.max_trajectory_length) {
      break;  // distance is over max.
    }

    // Check if the distance is longer than input arclength
    if (dist_i + front_arclength_value >= trajectory_length) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }

    // Check if the distance is longer than minimum_trajectory_length
    if (i > Nt && dist_i >= param.min_trajectory_length) {
      if (
        std::fabs(out_arclength.back() - (param.min_trajectory_length + front_arclength_value)) <
        1e-3) {
        out_arclength.back() = param.min_trajectory_length + front_arclength_value;
      } else {
        out_arclength.push_back(param.min_trajectory_length + front_arclength_value);
      }
      break;
    }

    // Handle Close Stop Point
    if (
      dist_to_closest_stop_point && dist_i > *dist_to_closest_stop_point &&
      !is_zero_point_included) {
      if (std::fabs(dist_i - *dist_to_closest_stop_point) > 1e-3) {
        // dist_i is much bigger than zero_vel_arclength_value
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (*dist_to_closest_stop_point + front_arclength_value)) <
            1e-3) {
          out_arclength.back() = *dist_to_closest_stop_point + front_arclength_value;
        } else {
          out_arclength.push_back(*dist_to_closest_stop_point + front_arclength_value);
        }
      } else {
        // dist_i is close to the zero_vel_arclength_value
        dist_i = *dist_to_closest_stop_point;
      }

      is_zero_point_included = true;
    }

    out_arclength.push_back(dist_i + front_arclength_value);
  }

  if (input.size() < 2 || out_arclength.size() < 2 || trajectory_length < out_arclength.back()) {
    return input;
  }

  const auto output_traj = motion_utils::resampleTrajectory(
    motion_utils::convertToTrajectory(input), out_arclength, false, true, use_zoh_for_v);
  auto output = motion_utils::convertToTrajectoryPointArray(output_traj);

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (tier4_autoware_utils::calcDistance2d(output.back(), input.back()) < ep_dist) {
      output.back() = input.back();
    } else {
      output.push_back(input.back());
    }
  }

  return output;
}

TrajectoryPoints resampleTrajectory(
  const TrajectoryPoints & input, const geometry_msgs::msg::Pose & current_pose,
  const double nearest_dist_threshold, const double nearest_yaw_threshold,
  const ResampleParam & param, const double nominal_ds, const bool use_zoh_for_v)
{
  // input arclength
  const double trajectory_length = motion_utils::calcArcLength(input);
  const auto dist_to_closest_stop_point =
    motion_utils::calcDistanceToForwardStopPoint(input, current_pose);

  // distance to stop point
  double stop_arclength_value = param.max_trajectory_length;
  if (dist_to_closest_stop_point) {
    stop_arclength_value = std::min(*dist_to_closest_stop_point, stop_arclength_value);
  }
  if (param.min_trajectory_length < stop_arclength_value) {
    stop_arclength_value = param.min_trajectory_length;
  }

  // Do dense resampling before the stop line(3[m] ahead of the stop line)
  const double start_stop_arclength_value = std::max(stop_arclength_value - 3.0, 0.0);

  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  // Arc length from the initial point to the closest point
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    input, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double negative_front_arclength_value = motion_utils::calcSignedArcLength(
    input, current_pose.position, current_seg_idx, input.at(0).pose.position,
    static_cast<size_t>(0));
  const auto front_arclength_value = std::fabs(negative_front_arclength_value);
  for (double s = 0.0; s <= front_arclength_value; s += nominal_ds) {
    out_arclength.push_back(s);
  }
  if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
    out_arclength.back() = front_arclength_value;
  } else {
    out_arclength.push_back(front_arclength_value);
  }

  // Step2. Resample behind trajectory
  double dist_i{0.0};
  bool is_zero_point_included{false};
  bool is_endpoint_included{false};
  while (rclcpp::ok()) {
    double ds = nominal_ds;
    if (start_stop_arclength_value <= dist_i && dist_i <= stop_arclength_value) {
      // Dense sampling before the stop point
      ds = 0.01;
    }
    dist_i += ds;

    // Check if the distance is longer than max_trajectory_length
    if (dist_i > param.max_trajectory_length) {
      break;  // distance is over max.
    }

    // Check if the distance is longer than input arclength
    if (dist_i + front_arclength_value >= trajectory_length) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }

    // Check if the distance is longer than minimum_trajectory_length
    if (dist_i >= param.min_trajectory_length) {
      if (
        std::fabs(out_arclength.back() - (param.min_trajectory_length + front_arclength_value)) <
        1e-3) {
        out_arclength.back() = param.min_trajectory_length + front_arclength_value;
      } else {
        out_arclength.push_back(param.min_trajectory_length + front_arclength_value);
      }
      break;
    }

    // Handle Close Stop Point
    if (dist_i > stop_arclength_value && !is_zero_point_included) {
      if (std::fabs(dist_i - stop_arclength_value) > 1e-3) {
        // dist_i is much bigger than zero_vel_arclength_value
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (stop_arclength_value + front_arclength_value)) < 1e-3) {
          out_arclength.back() = stop_arclength_value + front_arclength_value;
        } else {
          out_arclength.push_back(stop_arclength_value + front_arclength_value);
        }
      } else {
        // dist_i is close to the zero_vel_arclength_value
        dist_i = stop_arclength_value;
      }

      is_zero_point_included = true;
    }

    out_arclength.push_back(dist_i + front_arclength_value);
  }

  if (input.size() < 2 || out_arclength.size() < 2 || trajectory_length < out_arclength.back()) {
    return input;
  }

  const auto output_traj = motion_utils::resampleTrajectory(
    motion_utils::convertToTrajectory(input), out_arclength, false, true, use_zoh_for_v);
  auto output = motion_utils::convertToTrajectoryPointArray(output_traj);

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (tier4_autoware_utils::calcDistance2d(output.back(), input.back()) < ep_dist) {
      output.back() = input.back();
    } else {
      output.push_back(input.back());
    }
  }

  return output;
}

}  // namespace resampling
}  // namespace motion_velocity_smoother
