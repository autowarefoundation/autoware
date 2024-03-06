// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
// governing permissions and limitations under the License.

#include "mrm_handler/mrm_handler_core.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

MrmHandler::MrmHandler() : Node("mrm_handler")
{
  // Parameter
  param_.update_rate = declare_parameter<int>("update_rate", 10);
  param_.timeout_operation_mode_availability =
    declare_parameter<double>("timeout_operation_mode_availability", 0.5);
  param_.use_emergency_holding = declare_parameter<bool>("use_emergency_holding", false);
  param_.timeout_emergency_recovery = declare_parameter<double>("timeout_emergency_recovery", 5.0);
  param_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped", false);
  param_.use_pull_over = declare_parameter<bool>("use_pull_over", false);
  param_.use_comfortable_stop = declare_parameter<bool>("use_comfortable_stop", false);
  param_.turning_hazard_on.emergency = declare_parameter<bool>("turning_hazard_on.emergency", true);

  using std::placeholders::_1;

  // Subscriber
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&MrmHandler::onOdometry, this, _1));
  // subscribe control mode
  sub_control_mode_ = create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "~/input/control_mode", rclcpp::QoS{1}, std::bind(&MrmHandler::onControlMode, this, _1));
  sub_operation_mode_availability_ =
    create_subscription<tier4_system_msgs::msg::OperationModeAvailability>(
      "~/input/operation_mode_availability", rclcpp::QoS{1},
      std::bind(&MrmHandler::onOperationModeAvailability, this, _1));
  sub_mrm_pull_over_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
    "~/input/mrm/pull_over/status", rclcpp::QoS{1},
    std::bind(&MrmHandler::onMrmPullOverStatus, this, _1));
  sub_mrm_comfortable_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
    "~/input/mrm/comfortable_stop/status", rclcpp::QoS{1},
    std::bind(&MrmHandler::onMrmComfortableStopStatus, this, _1));
  sub_mrm_emergency_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
    "~/input/mrm/emergency_stop/status", rclcpp::QoS{1},
    std::bind(&MrmHandler::onMrmEmergencyStopStatus, this, _1));
  sub_operation_mode_state_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "~/input/api/operation_mode/state", rclcpp::QoS{1},
    std::bind(&MrmHandler::onOperationModeState, this, _1));

  // Publisher
  pub_hazard_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard", rclcpp::QoS{1});
  pub_gear_cmd_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("~/output/gear", rclcpp::QoS{1});
  pub_mrm_state_ =
    create_publisher<autoware_adapi_v1_msgs::msg::MrmState>("~/output/mrm/state", rclcpp::QoS{1});

  // Clients
  client_mrm_pull_over_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_mrm_pull_over_ = create_client<tier4_system_msgs::srv::OperateMrm>(
    "~/output/mrm/pull_over/operate", rmw_qos_profile_services_default,
    client_mrm_pull_over_group_);
  client_mrm_comfortable_stop_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_mrm_comfortable_stop_ = create_client<tier4_system_msgs::srv::OperateMrm>(
    "~/output/mrm/comfortable_stop/operate", rmw_qos_profile_services_default,
    client_mrm_comfortable_stop_group_);
  client_mrm_emergency_stop_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_mrm_emergency_stop_ = create_client<tier4_system_msgs::srv::OperateMrm>(
    "~/output/mrm/emergency_stop/operate", rmw_qos_profile_services_default,
    client_mrm_emergency_stop_group_);

  // Initialize
  odom_ = std::make_shared<const nav_msgs::msg::Odometry>();
  control_mode_ = std::make_shared<const autoware_auto_vehicle_msgs::msg::ControlModeReport>();
  mrm_pull_over_status_ = std::make_shared<const tier4_system_msgs::msg::MrmBehaviorStatus>();
  mrm_comfortable_stop_status_ =
    std::make_shared<const tier4_system_msgs::msg::MrmBehaviorStatus>();
  mrm_emergency_stop_status_ = std::make_shared<const tier4_system_msgs::msg::MrmBehaviorStatus>();
  operation_mode_state_ = std::make_shared<const autoware_adapi_v1_msgs::msg::OperationModeState>();
  mrm_state_.stamp = this->now();
  mrm_state_.state = autoware_adapi_v1_msgs::msg::MrmState::NORMAL;
  mrm_state_.behavior = autoware_adapi_v1_msgs::msg::MrmState::NONE;

  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmHandler::onTimer, this));
}

void MrmHandler::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void MrmHandler::onControlMode(
  const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg)
{
  control_mode_ = msg;
}

void MrmHandler::onOperationModeAvailability(
  const tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr msg)
{
  stamp_operation_mode_availability_ = this->now();

  if (!param_.use_emergency_holding) {
    operation_mode_availability_ = msg;
    return;
  }

  if (!is_emergency_holding_) {
    if (msg->autonomous) {
      stamp_autonomous_become_unavailable_.reset();
    } else if (!msg->autonomous) {
      if (!stamp_autonomous_become_unavailable_.has_value()) {
        stamp_autonomous_become_unavailable_.emplace(this->now());
      } else {
        const auto emergency_duration =
          (this->now() - stamp_autonomous_become_unavailable_.value()).seconds();
        if (emergency_duration > param_.timeout_emergency_recovery) {
          is_emergency_holding_ = true;
        }
      }
    }
  }
  operation_mode_availability_ = msg;
}

void MrmHandler::onMrmPullOverStatus(
  const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg)
{
  mrm_pull_over_status_ = msg;
}

void MrmHandler::onMrmComfortableStopStatus(
  const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg)
{
  mrm_comfortable_stop_status_ = msg;
}

void MrmHandler::onMrmEmergencyStopStatus(
  const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg)
{
  mrm_emergency_stop_status_ = msg;
}

void MrmHandler::onOperationModeState(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = msg;
}

autoware_auto_vehicle_msgs::msg::HazardLightsCommand MrmHandler::createHazardCmdMsg()
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  HazardLightsCommand msg;

  // Check emergency
  const bool is_emergency = isEmergency();

  if (is_emergency_holding_) {
    // turn hazard on during emergency holding
    msg.command = HazardLightsCommand::ENABLE;
  } else if (is_emergency && param_.turning_hazard_on.emergency) {
    // turn hazard on if vehicle is in emergency state and
    // turning hazard on if emergency flag is true
    msg.command = HazardLightsCommand::ENABLE;
  } else {
    msg.command = HazardLightsCommand::NO_COMMAND;
  }
  return msg;
}

void MrmHandler::publishControlCommands()
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;

  // Create timestamp
  const auto stamp = this->now();

  // Publish hazard command
  pub_hazard_cmd_->publish(createHazardCmdMsg());

  // Publish gear
  {
    GearCommand msg;
    msg.stamp = stamp;
    if (param_.use_parking_after_stopped && isStopped()) {
      msg.command = GearCommand::PARK;
    } else {
      msg.command = GearCommand::DRIVE;
    }
    pub_gear_cmd_->publish(msg);
  }
}

void MrmHandler::publishMrmState()
{
  mrm_state_.stamp = this->now();
  pub_mrm_state_->publish(mrm_state_);
}

void MrmHandler::operateMrm()
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  if (mrm_state_.state == MrmState::NORMAL) {
    // Cancel MRM behavior when returning to NORMAL state
    const auto current_mrm_behavior = MrmState::NONE;
    if (current_mrm_behavior != mrm_state_.behavior) {
      cancelMrmBehavior(mrm_state_.behavior);
      mrm_state_.behavior = current_mrm_behavior;
    }
    return;
  }
  if (mrm_state_.state == MrmState::MRM_OPERATING) {
    const auto current_mrm_behavior = getCurrentMrmBehavior();
    if (current_mrm_behavior != mrm_state_.behavior) {
      cancelMrmBehavior(mrm_state_.behavior);
      callMrmBehavior(current_mrm_behavior);
      mrm_state_.behavior = current_mrm_behavior;
    }
    return;
  }
  if (mrm_state_.state == MrmState::MRM_SUCCEEDED) {
    // TODO(mkuri): operate MRC behavior
    // Do nothing
    return;
  }
  if (mrm_state_.state == MrmState::MRM_FAILED) {
    // Do nothing
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM state: %d", mrm_state_.state);
}

void MrmHandler::callMrmBehavior(
  const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior) const
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  auto request = std::make_shared<tier4_system_msgs::srv::OperateMrm::Request>();
  request->operate = true;

  if (mrm_behavior == MrmState::NONE) {
    RCLCPP_WARN(this->get_logger(), "MRM behavior is None. Do nothing.");
    return;
  }
  if (mrm_behavior == MrmState::PULL_OVER) {
    auto result = client_mrm_pull_over_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Pull over is operated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Pull over failed to operate");
    }
    return;
  }
  if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    auto result = client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is operated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop failed to operate");
    }
    return;
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    auto result = client_mrm_emergency_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is operated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Emergency stop failed to operate");
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}

void MrmHandler::cancelMrmBehavior(
  const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior) const
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  auto request = std::make_shared<tier4_system_msgs::srv::OperateMrm::Request>();
  request->operate = false;

  if (mrm_behavior == MrmState::NONE) {
    // Do nothing
    return;
  }
  if (mrm_behavior == MrmState::PULL_OVER) {
    auto result = client_mrm_pull_over_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Pull over is canceled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Pull over failed to cancel");
    }
    return;
  }
  if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    auto result = client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is canceled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop failed to cancel");
    }
    return;
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    auto result = client_mrm_emergency_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is canceled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Emergency stop failed to cancel");
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}

bool MrmHandler::isDataReady()
{
  if (!operation_mode_availability_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for operation_mode_availability msg...");
    return false;
  }

  if (
    param_.use_pull_over &&
    mrm_pull_over_status_->state == tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm pull over to become available...");
    return false;
  }

  if (
    param_.use_comfortable_stop && mrm_comfortable_stop_status_->state ==
                                     tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm comfortable stop to become available...");
    return false;
  }

  if (
    mrm_emergency_stop_status_->state == tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm emergency stop to become available...");
    return false;
  }

  return true;
}

void MrmHandler::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  const bool is_operation_mode_availability_timeout =
    (this->now() - stamp_operation_mode_availability_).seconds() >
    param_.timeout_operation_mode_availability;
  if (is_operation_mode_availability_timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "heartbeat operation_mode_availability is timeout");
    mrm_state_.state = autoware_adapi_v1_msgs::msg::MrmState::MRM_OPERATING;
    publishControlCommands();
    return;
  }

  // Update Emergency State
  updateMrmState();

  // Publish control commands
  publishControlCommands();
  operateMrm();
  publishMrmState();
}

void MrmHandler::transitionTo(const int new_state)
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  const auto state2string = [](const int state) {
    if (state == MrmState::NORMAL) {
      return "NORMAL";
    }
    if (state == MrmState::MRM_OPERATING) {
      return "MRM_OPERATING";
    }
    if (state == MrmState::MRM_SUCCEEDED) {
      return "MRM_SUCCEEDED";
    }
    if (state == MrmState::MRM_FAILED) {
      return "MRM_FAILED";
    }

    const auto msg = "invalid state: " + std::to_string(state);
    throw std::runtime_error(msg);
  };

  RCLCPP_INFO(
    this->get_logger(), "MRM State changed: %s -> %s", state2string(mrm_state_.state),
    state2string(new_state));

  mrm_state_.state = new_state;
}

void MrmHandler::updateMrmState()
{
  using autoware_adapi_v1_msgs::msg::MrmState;
  using autoware_auto_vehicle_msgs::msg::ControlModeReport;

  // Check emergency
  const bool is_emergency = isEmergency();

  // Get mode
  const bool is_auto_mode = control_mode_->mode == ControlModeReport::AUTONOMOUS;

  // State Machine
  if (mrm_state_.state == MrmState::NORMAL) {
    // NORMAL
    if (is_auto_mode && is_emergency) {
      transitionTo(MrmState::MRM_OPERATING);
      return;
    }
  } else {
    // Emergency
    // Send recovery events if "not emergency"
    if (!is_emergency) {
      transitionTo(MrmState::NORMAL);
      return;
    }

    if (mrm_state_.state == MrmState::MRM_OPERATING) {
      // TODO(TetsuKawa): Check MRC is accomplished
      if (mrm_state_.behavior == MrmState::PULL_OVER) {
        if (isStopped() && isArrivedAtGoal()) {
          transitionTo(MrmState::MRM_SUCCEEDED);
          return;
        }
      } else {
        if (isStopped()) {
          transitionTo(MrmState::MRM_SUCCEEDED);
          return;
        }
      }
    } else if (mrm_state_.state == MrmState::MRM_SUCCEEDED) {
      const auto current_mrm_behavior = getCurrentMrmBehavior();
      if (current_mrm_behavior != mrm_state_.behavior) {
        transitionTo(MrmState::MRM_OPERATING);
      }
    } else if (mrm_state_.state == MrmState::MRM_FAILED) {
      // Do nothing(only checking common recovery events)
    } else {
      const auto msg = "invalid state: " + std::to_string(mrm_state_.state);
      throw std::runtime_error(msg);
    }
  }
}

autoware_adapi_v1_msgs::msg::MrmState::_behavior_type MrmHandler::getCurrentMrmBehavior()
{
  using autoware_adapi_v1_msgs::msg::MrmState;
  using tier4_system_msgs::msg::OperationModeAvailability;

  // State machine
  if (mrm_state_.behavior == MrmState::NONE) {
    if (operation_mode_availability_->pull_over) {
      if (param_.use_pull_over) {
        return MrmState::PULL_OVER;
      }
    }
    if (operation_mode_availability_->comfortable_stop) {
      if (param_.use_comfortable_stop) {
        return MrmState::COMFORTABLE_STOP;
      }
    }
    if (!operation_mode_availability_->emergency_stop) {
      RCLCPP_WARN(this->get_logger(), "no mrm operation available: operate emergency_stop");
    }
    return MrmState::EMERGENCY_STOP;
  }
  if (mrm_state_.behavior == MrmState::PULL_OVER) {
    if (operation_mode_availability_->pull_over) {
      if (param_.use_pull_over) {
        return MrmState::PULL_OVER;
      }
    }
    if (operation_mode_availability_->comfortable_stop) {
      if (param_.use_comfortable_stop) {
        return MrmState::COMFORTABLE_STOP;
      }
    }
    if (!operation_mode_availability_->emergency_stop) {
      RCLCPP_WARN(this->get_logger(), "no mrm operation available: operate emergency_stop");
    }
    return MrmState::EMERGENCY_STOP;
  }
  if (mrm_state_.behavior == MrmState::COMFORTABLE_STOP) {
    if (isStopped() && operation_mode_availability_->pull_over) {
      if (param_.use_pull_over) {
        return MrmState::PULL_OVER;
      }
    }
    if (operation_mode_availability_->comfortable_stop) {
      if (param_.use_comfortable_stop) {
        return MrmState::COMFORTABLE_STOP;
      }
    }
    if (!operation_mode_availability_->emergency_stop) {
      RCLCPP_WARN(this->get_logger(), "no mrm operation available: operate emergency_stop");
    }
    return MrmState::EMERGENCY_STOP;
  }
  if (mrm_state_.behavior == MrmState::EMERGENCY_STOP) {
    if (isStopped() && operation_mode_availability_->pull_over) {
      if (param_.use_pull_over) {
        return MrmState::PULL_OVER;
      }
    }
    if (!operation_mode_availability_->emergency_stop) {
      RCLCPP_WARN(this->get_logger(), "no mrm operation available: operate emergency_stop");
    }
    return MrmState::EMERGENCY_STOP;
  }

  return mrm_state_.behavior;
}

bool MrmHandler::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (odom_->twist.twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

bool MrmHandler::isEmergency() const
{
  return !operation_mode_availability_->autonomous || is_emergency_holding_;
}

bool MrmHandler::isArrivedAtGoal()
{
  using autoware_adapi_v1_msgs::msg::OperationModeState;

  return operation_mode_state_->mode == OperationModeState::STOP;
}
