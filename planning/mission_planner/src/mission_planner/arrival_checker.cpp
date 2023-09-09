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

#include "arrival_checker.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <tf2/utils.h>

namespace mission_planner
{

ArrivalChecker::ArrivalChecker(rclcpp::Node * node) : vehicle_stop_checker_(node)
{
  const double angle_deg = node->declare_parameter<double>("arrival_check_angle_deg");
  angle_ = tier4_autoware_utils::deg2rad(angle_deg);
  distance_ = node->declare_parameter<double>("arrival_check_distance");
  duration_ = node->declare_parameter<double>("arrival_check_duration");
}

void ArrivalChecker::set_goal()
{
  // Ignore the modified goal after the route is cleared.
  goal_with_uuid_ = std::nullopt;
}

void ArrivalChecker::set_goal(const PoseWithUuidStamped & goal)
{
  // Ignore the modified goal for the previous route using uuid.
  goal_with_uuid_ = goal;
}

void ArrivalChecker::modify_goal(const PoseWithUuidStamped & modified_goal)
{
  if (!goal_with_uuid_) {
    return;
  }
  if (goal_with_uuid_.value().uuid.uuid != modified_goal.uuid.uuid) {
    return;
  }
  set_goal(modified_goal);
}

bool ArrivalChecker::is_arrived(const PoseStamped & pose) const
{
  if (!goal_with_uuid_) {
    return false;
  }
  const auto goal = goal_with_uuid_.value();

  // Check frame id
  if (goal.header.frame_id != pose.header.frame_id) {
    return false;
  }

  // Check distance.
  if (distance_ < tier4_autoware_utils::calcDistance2d(pose.pose, goal.pose)) {
    return false;
  }

  // Check angle.
  const double yaw_pose = tf2::getYaw(pose.pose.orientation);
  const double yaw_goal = tf2::getYaw(goal.pose.orientation);
  const double yaw_diff = tier4_autoware_utils::normalizeRadian(yaw_pose - yaw_goal);
  if (angle_ < std::fabs(yaw_diff)) {
    return false;
  }

  // Check vehicle stopped.
  return vehicle_stop_checker_.isVehicleStopped(duration_);
}

}  // namespace mission_planner
