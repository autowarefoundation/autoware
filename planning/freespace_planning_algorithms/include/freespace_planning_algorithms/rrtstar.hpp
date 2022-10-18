// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_HPP_
#define FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_HPP_

#include "freespace_planning_algorithms/abstract_algorithm.hpp"
#include "freespace_planning_algorithms/rrtstar_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace freespace_planning_algorithms
{
struct RRTStarParam
{
  // algorithm configs
  bool enable_update;  // update solution even after feasible solution found with given time budget
  bool use_informed_sampling;  // use informed sampling (informed rrtstar)
  double max_planning_time;  // if enable_update is true, update is done before time elapsed [msec]
  double neighbour_radius;   // neighbore radius [m]
  double margin;             // [m]
};

class RRTStar : public AbstractPlanningAlgorithm
{
public:
  RRTStar(
    const PlannerCommonParam & planner_common_param, const VehicleShape & original_vehicle_shape,
    const RRTStarParam & rrtstar_param);

  RRTStar(
    const PlannerCommonParam & planner_common_param, const VehicleShape & original_vehicle_shape,
    rclcpp::Node & node)
  : RRTStar(
      planner_common_param, original_vehicle_shape,
      RRTStarParam{
        node.declare_parameter("rrtstar.enable_update", true),
        node.declare_parameter("rrtstar.use_informed_sampling", true),
        node.declare_parameter("rrtstar.max_planning_time", 150.0),
        node.declare_parameter("rrtstar.neighbour_radius", 8.0),
        node.declare_parameter("rrtstar.margin", 0.1)})
  {
  }

  bool makePlan(
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose) override;
  bool hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const override;

private:
  void setRRTPath(const std::vector<rrtstar_core::Pose> & waypints);

  // algorithm specific param
  const RRTStarParam rrtstar_param_;

  const VehicleShape original_vehicle_shape_;
};

}  // namespace freespace_planning_algorithms

#endif  // FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_HPP_
