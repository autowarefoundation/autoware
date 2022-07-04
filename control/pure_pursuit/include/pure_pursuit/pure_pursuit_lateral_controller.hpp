// Copyright 2020 Tier IV, Inc.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PURE_PURSUIT__PURE_PURSUIT_LATERAL_CONTROLLER_HPP_
#define PURE_PURSUIT__PURE_PURSUIT_LATERAL_CONTROLLER_HPP_

#include "pure_pursuit/pure_pursuit.hpp"
#include "pure_pursuit/pure_pursuit_viz.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tier4_autoware_utils/ros/self_pose_listener.hpp"
#include "trajectory_follower/lateral_controller_base.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#include <memory>
#include <vector>

using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerBase;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware_auto_control_msgs::msg::AckermannLateralCommand;

namespace pure_pursuit
{
struct Param
{
  // Global Parameters
  double wheel_base;

  // Algorithm Parameters
  double lookahead_distance_ratio;
  double min_lookahead_distance;
  double reverse_min_lookahead_distance;  // min_lookahead_distance in reverse gear
  double converged_steer_rad_;
};

struct DebugData
{
  geometry_msgs::msg::Point next_target;
};

class PurePursuitLateralController : public LateralControllerBase
{
public:
  explicit PurePursuitLateralController(rclcpp::Node & node);

private:
  rclcpp::Node::SharedPtr node_;
  tier4_autoware_utils::SelfPoseListener self_pose_listener_;

  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry_;
  autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr current_steering_;

  bool isDataReady();

  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onCurrentOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;

  // Debug Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;

  void publishDebugMarker() const;

  /**
   * @brief compute control command for path follow with a constant control period
   */
  boost::optional<LateralOutput> run() override;
  AckermannLateralCommand generateCtrlCmdMsg(const double target_curvature);

  /**
   * @brief set input data
   */
  void setInputData(InputData const & input_data) override;

  // Parameter
  Param param_;

  // Algorithm
  std::unique_ptr<PurePursuit> pure_pursuit_;

  boost::optional<double> calcTargetCurvature();
  boost::optional<autoware_auto_planning_msgs::msg::TrajectoryPoint> calcTargetPoint() const;

  // Debug
  mutable DebugData debug_data_;
};

}  // namespace pure_pursuit

#endif  // PURE_PURSUIT__PURE_PURSUIT_LATERAL_CONTROLLER_HPP_
