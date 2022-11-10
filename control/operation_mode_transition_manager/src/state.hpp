// Copyright 2022 Autoware Foundation
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

#ifndef STATE_HPP_
#define STATE_HPP_

#include "data.hpp"
#include "operation_mode_transition_manager/msg/operation_mode_transition_manager_debug.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <utility>

namespace operation_mode_transition_manager
{

class ModeChangeBase
{
public:
  virtual ~ModeChangeBase() = default;
  virtual void update([[maybe_unused]] bool transition) {}
  virtual bool isModeChangeCompleted() = 0;
  virtual bool isModeChangeAvailable() = 0;

  using DebugInfo = operation_mode_transition_manager::msg::OperationModeTransitionManagerDebug;
  virtual DebugInfo getDebugInfo() { return DebugInfo{}; }
};

class StopMode : public ModeChangeBase
{
public:
  bool isModeChangeCompleted() override { return true; }
  bool isModeChangeAvailable() override { return true; }
};

class AutonomousMode : public ModeChangeBase
{
public:
  explicit AutonomousMode(rclcpp::Node * node);
  void update(bool transition) override;
  bool isModeChangeCompleted() override;
  bool isModeChangeAvailable() override;
  DebugInfo getDebugInfo() override { return debug_info_; }

private:
  bool hasDangerAcceleration();
  std::pair<bool, bool> hasDangerLateralAcceleration();

  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using Odometry = nav_msgs::msg::Odometry;
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  EngageAcceptableParam engage_acceptable_param_;
  StableCheckParam stable_check_param_;
  AckermannControlCommand control_cmd_;
  Odometry kinematics_;
  Trajectory trajectory_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  DebugInfo debug_info_;
  std::shared_ptr<rclcpp::Time> stable_start_time_;  // Reset every transition start.
};

// TODO(Takagi, Isamu): Connect with status from local operation node
class LocalMode : public ModeChangeBase
{
public:
  bool isModeChangeCompleted() override { return true; }
  bool isModeChangeAvailable() override { return true; }
};

// TODO(Takagi, Isamu): Connect with status from remote operation node
class RemoteMode : public ModeChangeBase
{
public:
  bool isModeChangeCompleted() override { return true; }
  bool isModeChangeAvailable() override { return true; }
};

}  // namespace operation_mode_transition_manager

#endif  // STATE_HPP_
