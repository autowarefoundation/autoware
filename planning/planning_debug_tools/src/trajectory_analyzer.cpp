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

#include "planning_debug_tools/trajectory_analyzer.hpp"

namespace planning_debug_tools
{
TrajectoryAnalyzerNode::TrajectoryAnalyzerNode(const rclcpp::NodeOptions & options)
: Node("trajectory_analyzer", options)
{
  using TopicNames = std::vector<std::string>;
  const auto path_topics = declare_parameter<TopicNames>("path_topics");
  const auto path_with_lane_id_topics = declare_parameter<TopicNames>("path_with_lane_id_topics");
  const auto trajectory_topics = declare_parameter<TopicNames>("trajectory_topics");

  for (const auto & s : path_topics) {
    path_analyzers_.push_back(std::make_shared<TrajectoryAnalyzer<Path>>(this, s));
    RCLCPP_INFO(get_logger(), "path_topics: %s", s.c_str());
  }
  for (const auto & s : path_with_lane_id_topics) {
    path_with_lane_id_analyzers_.push_back(
      std::make_shared<TrajectoryAnalyzer<PathWithLaneId>>(this, s));
    RCLCPP_INFO(get_logger(), "path_with_lane_id_topics: %s", s.c_str());
  }

  for (const auto & s : trajectory_topics) {
    trajectory_analyzers_.push_back(std::make_shared<TrajectoryAnalyzer<Trajectory>>(this, s));
    RCLCPP_INFO(get_logger(), "trajectory_topics: %s", s.c_str());
  }

  using std::placeholders::_1;
  sub_ego_kinematics_ = create_subscription<Odometry>(
    "ego_kinematics", 1, std::bind(&TrajectoryAnalyzerNode::onEgoKinematics, this, _1));
}

void TrajectoryAnalyzerNode::onEgoKinematics(const Odometry::ConstSharedPtr msg)
{
  for (auto & a : path_analyzers_) {
    a->setKinematics(msg);
  }
  for (auto & a : path_with_lane_id_analyzers_) {
    a->setKinematics(msg);
  }
  for (auto & a : trajectory_analyzers_) {
    a->setKinematics(msg);
  }
}

}  // namespace planning_debug_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_debug_tools::TrajectoryAnalyzerNode)
