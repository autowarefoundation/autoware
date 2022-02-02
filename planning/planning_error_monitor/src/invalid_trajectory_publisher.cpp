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

#include "planning_error_monitor/invalid_trajectory_publisher.hpp"

#include <memory>
#include <string>
#include <utility>

namespace planning_diagnostics
{
InvalidTrajectoryPublisherNode::InvalidTrajectoryPublisherNode(
  const rclcpp::NodeOptions & node_options)
: Node("invalid_trajectory_publisher", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  traj_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&InvalidTrajectoryPublisherNode::onCurrentTrajectory, this, _1));

  traj_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);

  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&InvalidTrajectoryPublisherNode::onTimer, this));
}

void InvalidTrajectoryPublisherNode::onTimer()
{
  if (!current_trajectory_) {
    RCLCPP_INFO(this->get_logger(), "waiting trajectory");
    return;
  }
  if (current_trajectory_->points.empty()) {
    RCLCPP_INFO(this->get_logger(), "waiting trajectory");
    return;
  }

  constexpr auto ADDED_VALUE = 1.0e3;

  auto output = *current_trajectory_;
  auto & p = output.points.back().pose.position;
  p.x += ADDED_VALUE;
  p.y += ADDED_VALUE;
  p.z += ADDED_VALUE;

  traj_pub_->publish(output);

  RCLCPP_INFO(this->get_logger(), "invalid trajectory is published.");

  bool EXIT_AFTER_PUBLISH = false;
  if (EXIT_AFTER_PUBLISH) {
    exit(0);
  }
}

void InvalidTrajectoryPublisherNode::onCurrentTrajectory(const Trajectory::ConstSharedPtr msg)
{
  current_trajectory_ = msg;
  traj_sub_.reset();
}

}  // namespace planning_diagnostics

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_diagnostics::InvalidTrajectoryPublisherNode)
