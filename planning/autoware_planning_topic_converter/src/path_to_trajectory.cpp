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

#include "autoware/planning_topic_converter/path_to_trajectory.hpp"

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

namespace autoware::planning_topic_converter
{
namespace
{
TrajectoryPoint convertToTrajectoryPoint(const PathPoint & point)
{
  TrajectoryPoint traj_point;
  traj_point.pose = autoware::universe_utils::getPose(point);
  traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
  traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
  traj_point.heading_rate_rps = point.heading_rate_rps;
  return traj_point;
}

std::vector<TrajectoryPoint> convertToTrajectoryPoints(const std::vector<PathPoint> & points)
{
  std::vector<TrajectoryPoint> traj_points;
  for (const auto & point : points) {
    const auto traj_point = convertToTrajectoryPoint(point);
    traj_points.push_back(traj_point);
  }
  return traj_points;
}
}  // namespace

PathToTrajectory::PathToTrajectory(const rclcpp::NodeOptions & options)
: ConverterBase("path_to_trajectory_converter", options)
{
}

void PathToTrajectory::process(const Path::ConstSharedPtr msg)
{
  const auto trajectory_points = convertToTrajectoryPoints(msg->points);
  const auto output = autoware::motion_utils::convertToTrajectory(trajectory_points, msg->header);
  pub_->publish(output);
}

}  // namespace autoware::planning_topic_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_topic_converter::PathToTrajectory)
