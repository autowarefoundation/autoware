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

#ifndef MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
#define MISSION_PLANNER__ARRIVAL_CHECKER_HPP_

#include <motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace mission_planner
{

class ArrivalChecker
{
public:
  explicit ArrivalChecker(rclcpp::Node * node);
  void reset_goal();
  void reset_goal(const geometry_msgs::msg::PoseStamped & goal);
  bool is_arrived(const geometry_msgs::msg::PoseStamped & pose) const;

private:
  double distance_;
  double angle_;
  double duration_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  motion_utils::VehicleStopChecker vehicle_stop_checker_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
