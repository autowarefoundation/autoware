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

#ifndef PATH_DISTANCE_CALCULATOR_HPP_
#define PATH_DISTANCE_CALCULATOR_HPP_

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware/universe_utils/ros/self_pose_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/path.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

namespace autoware::path_distance_calculator
{

class PathDistanceCalculator : public rclcpp::Node
{
public:
  explicit PathDistanceCalculator(const rclcpp::NodeOptions & options);

private:
  autoware::universe_utils::InterProcessPollingSubscriber<autoware_planning_msgs::msg::Path>
    sub_path_{this, "~/input/path"};
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr pub_dist_;
  rclcpp::TimerBase::SharedPtr timer_;
  autoware::universe_utils::SelfPoseListener self_pose_listener_;
};

}  // namespace autoware::path_distance_calculator

#endif  // PATH_DISTANCE_CALCULATOR_HPP_
