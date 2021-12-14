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

#include <algorithm>
#include <vector>

namespace motion_velocity_smoother
{
namespace resampling
{
boost::optional<TrajectoryPoints> resampleTrajectory(
  const TrajectoryPoints & input, const double v_current, const size_t closest_id,
  const ResampleParam & param)
{
  // Arc length from the initial point to the closest point
  const double front_arclength_value = trajectory_utils::calcArcLength(input, 0, closest_id);

  // Get the nearest point where velocity is zero
  auto zero_vel_id = tier4_autoware_utils::searchZeroVelocityIndex(input, closest_id, input.size());
  // Arc length from the closest point to the point where velocity is zero
  double zero_vel_arclength_value = param.max_trajectory_length;
  if (zero_vel_id) {
    zero_vel_arclength_value = std::min(
      zero_vel_arclength_value,
      tier4_autoware_utils::calcSignedArcLength(input, closest_id, *zero_vel_id));
  }

  // Get the resample size from the closest point
  const std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);
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
    if (dist_i + front_arclength_value >= in_arclength.back()) {
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
    if (dist_i > zero_vel_arclength_value && !is_zero_point_included) {
      if (std::fabs(dist_i - zero_vel_arclength_value) > 1e-3) {
        // dist_i is much bigger than zero_vel_arclength_value
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (zero_vel_arclength_value + front_arclength_value)) <
            1e-3) {
          out_arclength.back() = zero_vel_arclength_value + front_arclength_value;
        } else {
          out_arclength.push_back(zero_vel_arclength_value + front_arclength_value);
        }
      } else {
        // dist_i is close to the zero_vel_arclength_value
        dist_i = zero_vel_arclength_value;
      }

      is_zero_point_included = true;
    }

    out_arclength.push_back(dist_i + front_arclength_value);
  }

  auto output =
    trajectory_utils::applyLinearInterpolation(in_arclength, input, out_arclength, true);
  if (!output) {
    RCLCPP_WARN(
      rclcpp::get_logger("motion_velocity_smoother").get_child("resample"),
      "fail trajectory interpolation. size : in_arclength = %lu, "
      "input = %lu, out_arclength = %lu",
      in_arclength.size(), input.size(), out_arclength.size());
    return boost::none;
  }

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (tier4_autoware_utils::calcDistance2d(output->back(), input.back()) < ep_dist) {
      output->back() = input.back();
    } else {
      output->push_back(input.back());
    }
  }

  return output;
}

boost::optional<TrajectoryPoints> resampleTrajectory(
  const TrajectoryPoints & input, const size_t closest_id, const ResampleParam & param,
  const double nominal_ds)
{
  // input arclength
  std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);

  // Get the nearest point where velocity is zero
  // to avoid getting closest_id as a stop point, search zero velocity index from closest_id + 1.
  auto stop_id = tier4_autoware_utils::searchZeroVelocityIndex(input, closest_id + 1, input.size());
  // Arc length from the closest point to the point where velocity is zero
  double stop_arclength_value = param.max_trajectory_length;
  if (stop_id) {
    stop_arclength_value = std::min(
      stop_arclength_value, tier4_autoware_utils::calcSignedArcLength(input, closest_id, *stop_id));
  }

  // Do dense resampling before the stop line(3[m] ahead of the stop line)
  if (param.min_trajectory_length < stop_arclength_value) {
    stop_arclength_value = param.min_trajectory_length;
  }
  if (in_arclength.back() < stop_arclength_value) {
    stop_arclength_value = in_arclength.back();
  }
  double start_stop_arclength_value = std::max(stop_arclength_value - 3.0, 0.0);

  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  // Arc length from the initial point to the closest point
  const double front_arclength_value = trajectory_utils::calcArcLength(input, 0, closest_id);
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
  while (true) {
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
    if (dist_i + front_arclength_value >= in_arclength.back()) {
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

  auto output = trajectory_utils::applyLinearInterpolation(in_arclength, input, out_arclength);
  if (!output) {
    RCLCPP_WARN(
      rclcpp::get_logger("motion_velocity_smoother").get_child("resample"),
      "fail trajectory interpolation. size : in_arclength = %lu, "
      "input = %lu, out_arclength = %lu",
      in_arclength.size(), input.size(), out_arclength.size());
    return boost::none;
  }

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (tier4_autoware_utils::calcDistance2d(output->back(), input.back()) < ep_dist) {
      output->back() = input.back();
    } else {
      output->push_back(input.back());
    }
  }

  return output;
}

}  // namespace resampling
}  // namespace motion_velocity_smoother
