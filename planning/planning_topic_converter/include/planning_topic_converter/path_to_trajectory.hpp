// Copyright 2023 TIER IV, Inc.
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

#ifndef PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_
#define PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_

#include "planning_topic_converter/converter_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <string>

namespace planning_topic_converter
{

using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class PathToTrajectory : public ConverterBase<Path, Trajectory>
{
public:
  explicit PathToTrajectory(const rclcpp::NodeOptions & options);

private:
  void process(const Path::ConstSharedPtr msg) override;
};

}  // namespace planning_topic_converter

#endif  // PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_
