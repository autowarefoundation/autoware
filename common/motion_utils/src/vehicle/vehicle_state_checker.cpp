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

#include "motion_utils/vehicle/vehicle_state_checker.hpp"

#include "motion_utils/trajectory/trajectory.hpp"

#include <string>

namespace motion_utils
{
VehicleStopCheckerBase::VehicleStopCheckerBase(rclcpp::Node * node, double buffer_duration)
: clock_(node->get_clock()), logger_(node->get_logger())
{
  buffer_duration_ = buffer_duration;
}

void VehicleStopCheckerBase::addTwist(const TwistStamped & twist)
{
  twist_buffer_.push_front(twist);

  const auto now = clock_->now();
  while (true) {
    // Check oldest data time
    const auto time_diff = now - twist_buffer_.back().header.stamp;

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= buffer_duration_) {
      break;
    }

    // Remove old data
    twist_buffer_.pop_back();
  }
}

bool VehicleStopCheckerBase::isVehicleStopped(const double stop_duration) const
{
  if (twist_buffer_.empty()) {
    return false;
  }

  constexpr double squared_stop_velocity = 1e-3 * 1e-3;
  const auto now = clock_->now();

  const auto time_buffer_back = now - twist_buffer_.back().header.stamp;
  if (time_buffer_back.seconds() < stop_duration) {
    return false;
  }

  // Get velocities within stop_duration
  for (const auto & velocity : twist_buffer_) {
    double x = velocity.twist.linear.x;
    double y = velocity.twist.linear.y;
    double z = velocity.twist.linear.z;
    double v = (x * x) + (y * y) + (z * z);
    if (squared_stop_velocity <= v) {
      return false;
    }

    const auto time_diff = now - velocity.header.stamp;
    if (time_diff.seconds() >= stop_duration) {
      break;
    }
  }

  return true;
}

VehicleStopChecker::VehicleStopChecker(rclcpp::Node * node)
: VehicleStopCheckerBase(node, velocity_buffer_time_sec)
{
  using std::placeholders::_1;

  sub_odom_ = node->create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&VehicleStopChecker::onOdom, this, _1));
}

void VehicleStopChecker::onOdom(const Odometry::SharedPtr msg)
{
  odometry_ptr_ = msg;

  TwistStamped current_velocity;
  current_velocity.header = msg->header;
  current_velocity.twist = msg->twist.twist;
  addTwist(current_velocity);
}

VehicleArrivalChecker::VehicleArrivalChecker(rclcpp::Node * node) : VehicleStopChecker(node)
{
  using std::placeholders::_1;

  sub_trajectory_ = node->create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS(1),
    std::bind(&VehicleArrivalChecker::onTrajectory, this, _1));
}

bool VehicleArrivalChecker::isVehicleStoppedAtStopPoint(const double stop_duration) const
{
  if (!odometry_ptr_ || !trajectory_ptr_) {
    return false;
  }

  if (!isVehicleStopped(stop_duration)) {
    return false;
  }

  const auto & p = odometry_ptr_->pose.pose.position;
  const auto idx = motion_utils::searchZeroVelocityIndex(trajectory_ptr_->points);

  if (!idx) {
    return false;
  }

  return std::abs(motion_utils::calcSignedArcLength(trajectory_ptr_->points, p, idx.get())) <
         th_arrived_distance_m;
}

void VehicleArrivalChecker::onTrajectory(const Trajectory::SharedPtr msg) { trajectory_ptr_ = msg; }
}  // namespace motion_utils
