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

#include "trajectory_follower_nodes/controller_node.hpp"

#include "pure_pursuit/pure_pursuit_lateral_controller.hpp"
#include "trajectory_follower/mpc_lateral_controller.hpp"
#include "trajectory_follower/pid_longitudinal_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
Controller::Controller(const rclcpp::NodeOptions & node_options) : Node("controller", node_options)
{
  using std::placeholders::_1;

  const double ctrl_period = declare_parameter<double>("ctrl_period", 0.03);
  timeout_thr_sec_ = declare_parameter<double>("timeout_thr_sec", 0.5);

  const auto lateral_controller_mode =
    getLateralControllerMode(declare_parameter("lateral_controller_mode", "mpc_follower"));
  switch (lateral_controller_mode) {
    case LateralControllerMode::MPC: {
      lateral_controller_ = std::make_shared<trajectory_follower::MpcLateralController>(*this);
      break;
    }
    case LateralControllerMode::PURE_PURSUIT: {
      lateral_controller_ = std::make_shared<pure_pursuit::PurePursuitLateralController>(*this);
      break;
    }
    default:
      throw std::domain_error("[LateralController] invalid algorithm");
  }

  const auto longitudinal_controller_mode =
    getLongitudinalControllerMode(declare_parameter("longitudinal_controller_mode", "pid"));
  switch (longitudinal_controller_mode) {
    case LongitudinalControllerMode::PID: {
      longitudinal_controller_ =
        std::make_shared<trajectory_follower::PidLongitudinalController>(*this);
      break;
    }
    default:
      throw std::domain_error("[LongitudinalController] invalid algorithm");
  }

  sub_ref_path_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/reference_trajectory", rclcpp::QoS{1}, std::bind(&Controller::onTrajectory, this, _1));
  sub_steering_ = create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "~/input/current_steering", rclcpp::QoS{1}, std::bind(&Controller::onSteering, this, _1));
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/current_odometry", rclcpp::QoS{1}, std::bind(&Controller::onOdometry, this, _1));
  sub_accel_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/current_accel", rclcpp::QoS{1}, std::bind(&Controller::onAccel, this, _1));
  control_cmd_pub_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());

  // Timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&Controller::callbackTimerControl, this));
  }
}

Controller::LateralControllerMode Controller::getLateralControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "mpc_follower") return LateralControllerMode::MPC;
  if (controller_mode == "pure_pursuit") return LateralControllerMode::PURE_PURSUIT;

  return LateralControllerMode::INVALID;
}

Controller::LongitudinalControllerMode Controller::getLongitudinalControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "pid") return LongitudinalControllerMode::PID;

  return LongitudinalControllerMode::INVALID;
}

void Controller::onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  input_data_.current_trajectory_ptr = msg;
}

void Controller::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  input_data_.current_odometry_ptr = msg;
}

void Controller::onSteering(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
{
  input_data_.current_steering_ptr = msg;
}

void Controller::onAccel(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  input_data_.current_accel_ptr = msg;
}

bool Controller::isTimeOut()
{
  const auto now = this->now();
  if ((now - lateral_output_->control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Lateral control command too old, control_cmd will not be published.");
    return true;
  }
  if ((now - longitudinal_output_->control_cmd.stamp).seconds() > timeout_thr_sec_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "Longitudinal control command too old, control_cmd will not be published.");
    return true;
  }
  return false;
}

void Controller::callbackTimerControl()
{
  // Since the longitudinal uses the convergence information of the steer
  // with the current trajectory, it is necessary to run the lateral first.
  // TODO(kosuke55): Do not depend on the order of execution.
  lateral_controller_->setInputData(input_data_);  // trajectory, odometry, steering
  const auto lat_out = lateral_controller_->run();
  lateral_output_ = lat_out ? lat_out : lateral_output_;  // use previous value if none.
  if (!lateral_output_) return;

  longitudinal_controller_->sync(lateral_output_->sync_data);
  longitudinal_controller_->setInputData(input_data_);  // trajectory, odometry
  const auto lon_out = longitudinal_controller_->run();
  longitudinal_output_ = lon_out ? lon_out : longitudinal_output_;  // use previous value if none.
  if (!longitudinal_output_) return;

  lateral_controller_->sync(longitudinal_output_->sync_data);

  // TODO(Horibe): Think specification. This comes from the old implementation.
  if (isTimeOut()) return;

  autoware_auto_control_msgs::msg::AckermannControlCommand out;
  out.stamp = this->now();
  out.lateral = lateral_output_->control_cmd;
  out.longitudinal = longitudinal_output_->control_cmd;
  control_cmd_pub_->publish(out);
}

}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_nodes::Controller)
