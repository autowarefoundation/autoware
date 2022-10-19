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

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

// TODO(Takagi, Isamu): remove when modified goal is always published
#include <memory>

namespace mission_planner
{

ArrivalChecker::ArrivalChecker(rclcpp::Node * node) : vehicle_stop_checker_(node)
{
  const double angle_deg = node->declare_parameter<double>("arrival_check_angle_deg");
  angle_ = tier4_autoware_utils::deg2rad(angle_deg);
  distance_ = node->declare_parameter<double>("arrival_check_distance");
  duration_ = node->declare_parameter<double>("arrival_check_duration");

  sub_goal_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/modified_goal", 1,
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { goal_pose_ = msg; });
}

void ArrivalChecker::reset_goal()
{
  // Disable checking until the modified goal is received.
  goal_pose_.reset();
}

// TODO(Takagi, Isamu): remove when modified goal is always published
void ArrivalChecker::reset_goal(const geometry_msgs::msg::PoseStamped & goal)
{
  const auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  *pose = goal;
  goal_pose_ = pose;
}

bool ArrivalChecker::is_arrived(const geometry_msgs::msg::PoseStamped & pose) const
{
  // Check if the modified goal is received.
  if (goal_pose_ == nullptr) {
    return false;
  }
  const auto & goal = *goal_pose_;

  // Check frame_id.
  if (goal.header.frame_id != pose.header.frame_id) {
    return false;
  }

  // Check distance.
  if (distance_ < tier4_autoware_utils::calcDistance2d(pose, goal)) {
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
