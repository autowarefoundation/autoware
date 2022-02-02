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

#include "pure_pursuit/pure_pursuit_node.hpp"

#include "pure_pursuit/pure_pursuit_viz.hpp"
#include "pure_pursuit/util/planning_utils.hpp"
#include "pure_pursuit/util/tf_utils.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

#include <algorithm>
#include <memory>
#include <utility>

namespace
{
double calcLookaheadDistance(
  const double velocity, const double lookahead_distance_ratio, const double min_lookahead_distance)
{
  const double lookahead_distance = lookahead_distance_ratio * std::abs(velocity);
  return std::max(lookahead_distance, min_lookahead_distance);
}

}  // namespace

namespace pure_pursuit
{
PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions & node_options)
: Node("pure_pursuit", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  pure_pursuit_ = std::make_unique<PurePursuit>();

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;

  // Node Parameters
  param_.ctrl_period = this->declare_parameter<double>("control_period", 0.02);

  // Algorithm Parameters
  param_.lookahead_distance_ratio =
    this->declare_parameter<double>("lookahead_distance_ratio", 2.2);
  param_.min_lookahead_distance = this->declare_parameter<double>("min_lookahead_distance", 2.5);
  param_.reverse_min_lookahead_distance =
    this->declare_parameter<double>("reverse_min_lookahead_distance", 7.0);

  // Subscribers
  using std::placeholders::_1;
  sub_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1, std::bind(&PurePursuitNode::onTrajectory, this, _1));
  sub_current_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/current_odometry", 1, std::bind(&PurePursuitNode::onCurrentOdometry, this, _1));

  // Publishers
  pub_ctrl_cmd_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannLateralCommand>(
    "output/control_raw", 1);

  // Debug Publishers
  pub_debug_marker_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 0);

  // Timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(param_.ctrl_period));
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&PurePursuitNode::onTimer, this));
  }

  //  Wait for first current pose
  tf_utils::waitForTransform(tf_buffer_, "map", "base_link");
}

bool PurePursuitNode::isDataReady()
{
  if (!current_odometry_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_odometry...");
    return false;
  }

  if (!trajectory_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for trajectory...");
    return false;
  }

  if (!current_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  return true;
}

void PurePursuitNode::onCurrentOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odometry_ = msg;
}

void PurePursuitNode::onTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  trajectory_ = msg;
}

void PurePursuitNode::onTimer()
{
  current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  const auto target_curvature = calcTargetCurvature();

  if (target_curvature) {
    publishCommand(*target_curvature);
    publishDebugMarker();
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "failed to solve pure_pursuit");
    publishCommand({0.0});
  }
}

void PurePursuitNode::publishCommand(const double target_curvature)
{
  autoware_auto_control_msgs::msg::AckermannLateralCommand cmd;
  cmd.stamp = get_clock()->now();
  cmd.steering_tire_angle =
    planning_utils::convertCurvatureToSteeringAngle(param_.wheel_base, target_curvature);
  pub_ctrl_cmd_->publish(cmd);
}

void PurePursuitNode::publishDebugMarker() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_pose_->pose));

  pub_debug_marker_->publish(marker_array);
}

boost::optional<double> PurePursuitNode::calcTargetCurvature()
{
  // Ignore invalid trajectory
  if (trajectory_->points.size() < 3) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "received path size is < 3, ignored");
    return {};
  }

  // Calculate target point for velocity/acceleration
  const auto target_point = calcTargetPoint();
  if (!target_point) {
    return {};
  }

  const double target_vel = target_point->longitudinal_velocity_mps;

  // Calculate lookahead distance
  const bool is_reverse = (target_vel < 0);
  const double min_lookahead_distance =
    is_reverse ? param_.reverse_min_lookahead_distance : param_.min_lookahead_distance;
  const double lookahead_distance = calcLookaheadDistance(
    current_odometry_->twist.twist.linear.x, param_.lookahead_distance_ratio,
    min_lookahead_distance);

  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(current_pose_->pose);
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(*trajectory_));
  pure_pursuit_->setLookaheadDistance(lookahead_distance);

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return {};
  }

  const auto kappa = pure_pursuit_result.second;

  // Set debug data
  debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();

  return kappa;
}

boost::optional<autoware_auto_planning_msgs::msg::TrajectoryPoint>
PurePursuitNode::calcTargetPoint() const
{
  const auto closest_idx_result = planning_utils::findClosestIdxWithDistAngThr(
    planning_utils::extractPoses(*trajectory_), current_pose_->pose, 3.0, M_PI_4);

  if (!closest_idx_result.first) {
    RCLCPP_ERROR(get_logger(), "cannot find closest waypoint");
    return {};
  }

  return trajectory_->points.at(closest_idx_result.second);
}
}  // namespace pure_pursuit

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pure_pursuit::PurePursuitNode)
