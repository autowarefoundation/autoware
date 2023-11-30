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

#include <behavior_velocity_planner_common/velocity_factor_interface.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

namespace behavior_velocity_planner
{
void VelocityFactorInterface::set(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const Pose & curr_pose, const Pose & stop_pose, const VelocityFactorStatus status,
  const std::string detail)
{
  const auto & curr_point = curr_pose.position;
  const auto & stop_point = stop_pose.position;
  velocity_factor_.behavior = behavior_;
  velocity_factor_.pose = stop_pose;
  velocity_factor_.distance = motion_utils::calcSignedArcLength(points, curr_point, stop_point);
  velocity_factor_.status = status;
  velocity_factor_.detail = detail;
}

}  // namespace behavior_velocity_planner
