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

#ifndef TRAJECTORY_FOLLOWER__INPUT_DATA_HPP_
#define TRAJECTORY_FOLLOWER__INPUT_DATA_HPP_

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
struct InputData
{
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr current_trajectory_ptr;
  nav_msgs::msg::Odometry::SharedPtr current_odometry_ptr;
  autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr current_steering_ptr;
  geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr current_accel_ptr;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__INPUT_DATA_HPP_
