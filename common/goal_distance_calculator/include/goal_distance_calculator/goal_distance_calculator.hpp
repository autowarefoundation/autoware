// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_HPP_
#define GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/pose_deviation.hpp>

#include <autoware_auto_planning_msgs/msg/route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <boost/optional.hpp>

namespace goal_distance_calculator
{
using tier4_autoware_utils::PoseDeviation;

struct Param
{
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  autoware_auto_planning_msgs::msg::Route::ConstSharedPtr route;
};

struct Output
{
  PoseDeviation goal_deviation;
};

class GoalDistanceCalculator
{
public:
  static Output update(const Input & input);

  void setParam(const Param & param) { param_ = param; }

private:
  Param param_;
};
}  // namespace goal_distance_calculator

#endif  // GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_HPP_
