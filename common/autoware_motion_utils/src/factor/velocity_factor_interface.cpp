// Copyright 2023-2024 TIER IV, Inc.
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

#include <autoware/motion_utils/factor/velocity_factor_interface.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

namespace autoware::motion_utils
{
template <class PointType>
void VelocityFactorInterface::set(
  const std::vector<PointType> & points, const Pose & curr_pose, const Pose & stop_pose,
  const VelocityFactorStatus status, const std::string & detail)
{
  const auto & curr_point = curr_pose.position;
  const auto & stop_point = stop_pose.position;
  velocity_factor_.behavior = behavior_;
  velocity_factor_.pose = stop_pose;
  velocity_factor_.distance =
    static_cast<float>(autoware::motion_utils::calcSignedArcLength(points, curr_point, stop_point));
  velocity_factor_.status = status;
  velocity_factor_.detail = detail;
}

template void VelocityFactorInterface::set<tier4_planning_msgs::msg::PathPointWithLaneId>(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> &, const Pose &, const Pose &,
  const VelocityFactorStatus, const std::string &);
template void VelocityFactorInterface::set<autoware_planning_msgs::msg::PathPoint>(
  const std::vector<autoware_planning_msgs::msg::PathPoint> &, const Pose &, const Pose &,
  const VelocityFactorStatus, const std::string &);
template void VelocityFactorInterface::set<autoware_planning_msgs::msg::TrajectoryPoint>(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &, const Pose &, const Pose &,
  const VelocityFactorStatus, const std::string &);

}  // namespace autoware::motion_utils
