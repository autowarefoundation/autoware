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

#ifndef PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_

#include "pure_pursuit/pure_pursuit.hpp"
#include "pure_pursuit/pure_pursuit_viz.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

namespace pure_pursuit
{
struct Param
{
  // Global Parameters
  double wheel_base;

  // Node Parameters
  double ctrl_period;

  // Algorithm Parameters
  double lookahead_distance_ratio;
  double min_lookahead_distance;
  double reverse_min_lookahead_distance;  // min_lookahead_distance in reverse gear
};

struct DebugData
{
  geometry_msgs::msg::Point next_target;
};

class PurePursuitNode : public rclcpp::Node
{
public:
  explicit PurePursuitNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber
  tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_current_odometry_;

  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry_;

  bool isDataReady();

  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onCurrentOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;

  // Publisher
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannLateralCommand>::SharedPtr
    pub_ctrl_cmd_;

  void publishCommand(const double target_curvature);

  // Debug Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;

  void publishDebugMarker() const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

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

#endif  // PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
