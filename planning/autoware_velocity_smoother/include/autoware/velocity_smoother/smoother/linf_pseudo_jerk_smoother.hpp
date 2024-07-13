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

#ifndef AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__LINF_PSEUDO_JERK_SMOOTHER_HPP_
#define AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__LINF_PSEUDO_JERK_SMOOTHER_HPP_

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"
#include "autoware/velocity_smoother/smoother/smoother_base.hpp"
#include "osqp_interface/osqp_interface.hpp"

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include "boost/optional.hpp"

#include <memory>
#include <vector>

namespace autoware::velocity_smoother
{
class LinfPseudoJerkSmoother : public SmootherBase
{
public:
  struct Param
  {
    double pseudo_jerk_weight;
    double over_v_weight;
    double over_a_weight;
  };

  explicit LinfPseudoJerkSmoother(
    rclcpp::Node & node, const std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper);

  bool apply(
    const double initial_vel, const double initial_acc, const TrajectoryPoints & input,
    TrajectoryPoints & output, std::vector<TrajectoryPoints> & debug_trajectories) override;

  TrajectoryPoints resampleTrajectory(
    const TrajectoryPoints & input, const double v0, const geometry_msgs::msg::Pose & current_pose,
    const double nearest_dist_threshold, const double nearest_yaw_threshold) const override;

  void setParam(const Param & smoother_param);
  Param getParam() const;

private:
  Param smoother_param_;
  autoware::common::osqp::OSQPInterface qp_solver_;
  rclcpp::Logger logger_{rclcpp::get_logger("smoother").get_child("linf_pseudo_jerk_smoother")};
};
}  // namespace autoware::velocity_smoother

#endif  // AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__LINF_PSEUDO_JERK_SMOOTHER_HPP_
