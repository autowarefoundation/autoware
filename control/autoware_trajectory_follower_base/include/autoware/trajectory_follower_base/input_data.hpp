// Copyright 2022 The Autoware Foundation
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_BASE__INPUT_DATA_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_BASE__INPUT_DATA_HPP_

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace autoware::motion::control::trajectory_follower
{
struct InputData
{
  autoware_planning_msgs::msg::Trajectory current_trajectory;
  nav_msgs::msg::Odometry current_odometry;
  autoware_vehicle_msgs::msg::SteeringReport current_steering;
  geometry_msgs::msg::AccelWithCovarianceStamped current_accel;
  autoware_adapi_v1_msgs::msg::OperationModeState current_operation_mode;
};
}  // namespace autoware::motion::control::trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_BASE__INPUT_DATA_HPP_
