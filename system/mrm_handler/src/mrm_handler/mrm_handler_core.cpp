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

MrmHandler::MrmHandler(const rclcpp::NodeOptions & options) : Node("mrm_handler", options)
{
  // Parameter
  param_.update_rate = declare_parameter<int>("update_rate", 10);
  param_.timeout_operation_mode_availability =
    declare_parameter<double>("timeout_operation_mode_availability", 0.5);
  param_.timeout_call_mrm_behavior = declare_parameter<double>("timeout_call_mrm_behavior", 0.01);
  param_.timeout_cancel_mrm_behavior =
    declare_parameter<double>("timeout_cancel_mrm_behavior", 0.01);
  param_.use_emergency_holding = declare_parameter<bool>("use_emergency_holding", false);
  param_.timeout_emergency_recovery = declare_parameter<double>("timeout_emergency_recovery", 5.0);
  param_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped", false);
  param_.use_pull_over = declare_parameter<bool>("use_pull_over", false);
  param_.use_comfortable_stop = declare_parameter<bool>("use_comfortable_stop", false);
  param_.turning_hazard_on.emergency = declare_parameter<bool>("turning_hazard_on.emergency", true);

  using std::placeholders::_1;

  // Subscribers with callback
  sub_operation_mode_availability_ =
    create_subscription<tier4_system_msgs::msg::OperationModeAvailability>(
      "~/input/operation_mode_availability", rclcpp::QoS{1},
      std::bind(&MrmHandler::onOperationModeAvailability, this, _1));

  // Publisher
  pub_hazard_cmd_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard", rclcpp::QoS{1});
  pub_gear_cmd_ =
    create_publisher<autoware_vehicle_msgs::msg::GearCommand>("~/output/gear", rclcpp::QoS{1});
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
  mrm_state_.stamp = this->now();
  mrm_state_.state = autoware_adapi_v1_msgs::msg::MrmState::NORMAL;
  mrm_state_.behavior = autoware_adapi_v1_msgs::msg::MrmState::NONE;
  is_operation_mode_availability_timeout = false;

  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmHandler::onTimer, this));
}

void MrmHandler::onOperationModeAvailability(
  const tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr msg)
{
  stamp_operation_mode_availability_ = this->now();
  operation_mode_availability_ = msg;
  const bool skip_emergency_holding_check =
    !param_.use_emergency_holding || is_emergency_holding_ || !isOperationModeAutonomous();

  if (skip_emergency_holding_check) {
    return;
  }

  if (msg->autonomous) {
    stamp_autonomous_become_unavailable_.reset();
    return;
  }

  // If no timestamp is available, the ego autonomous mode just became unavailable and the current
  // time is recorded.
  stamp_autonomous_become_unavailable_ = (!stamp_autonomous_become_unavailable_.has_value())
                                           ? this->now()
                                           : stamp_autonomous_become_unavailable_;

  // Check if autonomous mode unavailable time is larger than timeout threshold.
  const auto emergency_duration =
    (this->now() - stamp_autonomous_become_unavailable_.value()).seconds();
  is_emergency_holding_ = (emergency_duration > param_.timeout_emergency_recovery);
}

void MrmHandler::publishHazardCmd()
{
  using autoware_vehicle_msgs::msg::HazardLightsCommand;
  HazardLightsCommand msg;

  msg.stamp = this->now();
  if (is_emergency_holding_) {
    // turn hazard on during emergency holding
    msg.command = HazardLightsCommand::ENABLE;
  } else if (isEmergency() && param_.turning_hazard_on.emergency) {
    // turn hazard on if vehicle is in emergency state and
    // turning hazard on if emergency flag is true
    msg.command = HazardLightsCommand::ENABLE;
  } else {
    msg.command = HazardLightsCommand::NO_COMMAND;
  }

  pub_hazard_cmd_->publish(msg);
}

void MrmHandler::publishGearCmd()
{
  using autoware_vehicle_msgs::msg::GearCommand;
  GearCommand msg;
  msg.stamp = this->now();

  if (isEmergency()) {
    // gear command is created within mrm_handler
    msg.command =
      (param_.use_parking_after_stopped && isStopped()) ? GearCommand::PARK : last_gear_command_;
  } else {
    // use the same gear as the input gear
    auto gear = sub_gear_cmd_.takeData();
    msg.command = (gear == nullptr) ? last_gear_command_ : gear->command;
    last_gear_command_ = msg.command;
  }

  pub_gear_cmd_->publish(msg);
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
    if (current_mrm_behavior == mrm_state_.behavior) {
      return;
    }
    if (requestMrmBehavior(mrm_state_.behavior, RequestType::CANCEL)) {
      mrm_state_.behavior = current_mrm_behavior;
    } else {
      handleFailedRequest();
    }
    return;
  }
  if (mrm_state_.state == MrmState::MRM_OPERATING) {
    const auto current_mrm_behavior = getCurrentMrmBehavior();
    if (current_mrm_behavior == mrm_state_.behavior) {
      return;
    }
    if (!requestMrmBehavior(mrm_state_.behavior, RequestType::CANCEL)) {
      handleFailedRequest();
    } else if (requestMrmBehavior(current_mrm_behavior, RequestType::CALL)) {
      mrm_state_.behavior = current_mrm_behavior;
    } else {
      handleFailedRequest();
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

void MrmHandler::handleFailedRequest()
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  if (requestMrmBehavior(MrmState::EMERGENCY_STOP, CALL)) {
    if (mrm_state_.state != MrmState::MRM_OPERATING) transitionTo(MrmState::MRM_OPERATING);
  } else {
    transitionTo(MrmState::MRM_FAILED);
  }
  mrm_state_.behavior = MrmState::EMERGENCY_STOP;
}

bool MrmHandler::requestMrmBehavior(
  const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior,
  RequestType request_type) const
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  auto request = std::make_shared<tier4_system_msgs::srv::OperateMrm::Request>();
  if (request_type == RequestType::CALL) {
    request->operate = true;
  } else if (request_type == RequestType::CANCEL) {
    request->operate = false;
  } else {
    RCLCPP_ERROR(this->get_logger(), "invalid request type: %d", request_type);
    return false;
  }
  const auto duration = std::chrono::duration<double, std::ratio<1>>(
    request->operate ? param_.timeout_call_mrm_behavior : param_.timeout_cancel_mrm_behavior);
  std::shared_future<std::shared_ptr<tier4_system_msgs::srv::OperateMrm::Response>> future;

  const auto behavior2string = [](const int behavior) {
    if (behavior == MrmState::NONE) {
      return "NONE";
    }
    if (behavior == MrmState::PULL_OVER) {
      return "PULL_OVER";
    }
    if (behavior == MrmState::COMFORTABLE_STOP) {
      return "COMFORTABLE_STOP";
    }
    if (behavior == MrmState::EMERGENCY_STOP) {
      return "EMERGENCY_STOP";
    }
    const auto msg = "invalid behavior: " + std::to_string(behavior);
    throw std::runtime_error(msg);
  };

  switch (mrm_behavior) {
    case MrmState::NONE:
      RCLCPP_WARN(this->get_logger(), "MRM behavior is None. Do nothing.");
      return true;
    case MrmState::PULL_OVER: {
      future = client_mrm_pull_over_->async_send_request(request).future.share();
      break;
    }
    case MrmState::COMFORTABLE_STOP: {
      future = client_mrm_comfortable_stop_->async_send_request(request).future.share();
      break;
    }
    case MrmState::EMERGENCY_STOP: {
      future = client_mrm_emergency_stop_->async_send_request(request).future.share();
      break;
    }
    default:
      RCLCPP_ERROR(this->get_logger(), "invalid behavior: %d", mrm_behavior);
      return false;
  }

  if (future.wait_for(duration) == std::future_status::ready) {
    const auto result = future.get();
    if (result->response.success == true) {
      RCLCPP_WARN(
        this->get_logger(), request->operate ? "%s is operated." : "%s is canceled.",
        behavior2string(mrm_behavior));
      return true;
    } else {
      RCLCPP_ERROR(
        this->get_logger(), request->operate ? "%s failed to operate." : "%s failed to cancel.",
        behavior2string(mrm_behavior));
      return false;
    }
  } else {
    RCLCPP_ERROR(
      this->get_logger(), request->operate ? "%s call timed out." : "%s cancel timed out.",
      behavior2string(mrm_behavior));
    return false;
  }
}

bool MrmHandler::isDataReady()
{
  if (!operation_mode_availability_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for operation_mode_availability msg...");
    return false;
  }

  if (param_.use_pull_over && !isPullOverStatusAvailable()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm pull over to become available...");
    return false;
  }

  if (param_.use_comfortable_stop && !isComfortableStopStatusAvailable()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm comfortable stop to become available...");
    return false;
  }

  if (!isEmergencyStopStatusAvailable()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm emergency stop to become available...");
    return false;
  }

  return true;
}

void MrmHandler::checkOperationModeAvailabilityTimeout()
{
  if (
    (this->now() - stamp_operation_mode_availability_).seconds() >
    param_.timeout_operation_mode_availability) {
    is_operation_mode_availability_timeout = true;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "operation_mode_availability is timeout");
  } else {
    is_operation_mode_availability_timeout = false;
  }
}

void MrmHandler::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  // Check whether operation_mode_availability is timeout
  checkOperationModeAvailabilityTimeout();

  // Update Emergency State
  updateMrmState();

  // Operate MRM
  operateMrm();

  // Publish
  publishMrmState();
  publishHazardCmd();
  publishGearCmd();
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
  using autoware_vehicle_msgs::msg::ControlModeReport;

  // Check emergency
  const bool is_emergency = isEmergency();

  // Send recovery events if is not an emergency
  if (!is_emergency) {
    if (mrm_state_.state != MrmState::NORMAL) transitionTo(MrmState::NORMAL);
    return;
  }

  // Get mode
  const bool is_control_mode_autonomous = isControlModeAutonomous();
  const bool is_operation_mode_autonomous = isOperationModeAutonomous();

  // State Machine
  switch (mrm_state_.state) {
    case MrmState::NORMAL:
      if (is_control_mode_autonomous && is_operation_mode_autonomous) {
        transitionTo(MrmState::MRM_OPERATING);
      }
      return;

    case MrmState::MRM_OPERATING:
      if (!isStopped()) return;
      if (mrm_state_.behavior != MrmState::PULL_OVER) {
        transitionTo(MrmState::MRM_SUCCEEDED);
        return;
      }
      if (isArrivedAtGoal()) {
        transitionTo(MrmState::MRM_SUCCEEDED);
      }
      return;

    case MrmState::MRM_SUCCEEDED:
      if (mrm_state_.behavior != getCurrentMrmBehavior()) {
        transitionTo(MrmState::MRM_OPERATING);
      }
      return;
    case MrmState::MRM_FAILED:
      // Do nothing(only checking common recovery events)
      return;

    default: {
      const auto msg = "invalid state: " + std::to_string(mrm_state_.state);
      throw std::runtime_error(msg);
    }
      return;
  }
}

autoware_adapi_v1_msgs::msg::MrmState::_behavior_type MrmHandler::getCurrentMrmBehavior()
{
  using autoware_adapi_v1_msgs::msg::MrmState;
  using tier4_system_msgs::msg::OperationModeAvailability;

  // State machine
  if (mrm_state_.behavior == MrmState::NONE) {
    if (is_operation_mode_availability_timeout) {
      return MrmState::EMERGENCY_STOP;
    }
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
    if (is_operation_mode_availability_timeout) {
      return MrmState::EMERGENCY_STOP;
    }
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
    if (is_operation_mode_availability_timeout) {
      return MrmState::EMERGENCY_STOP;
    }
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
    if (is_operation_mode_availability_timeout) {
      return MrmState::EMERGENCY_STOP;
    }
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
  auto odom = sub_odom_.takeData();
  if (odom == nullptr) return false;
  constexpr auto th_stopped_velocity = 0.001;
  return (std::abs(odom->twist.twist.linear.x) < th_stopped_velocity);
}

bool MrmHandler::isEmergency() const
{
  return !operation_mode_availability_->autonomous || is_emergency_holding_ ||
         is_operation_mode_availability_timeout;
}

bool MrmHandler::isControlModeAutonomous()
{
  using autoware_vehicle_msgs::msg::ControlModeReport;
  auto mode = sub_control_mode_.takeData();
  if (mode == nullptr) return false;
  return mode->mode == ControlModeReport::AUTONOMOUS;
}

bool MrmHandler::isOperationModeAutonomous()
{
  using autoware_adapi_v1_msgs::msg::OperationModeState;
  auto state = sub_operation_mode_state_.takeData();
  if (state == nullptr) return false;
  return state->mode == OperationModeState::AUTONOMOUS;
}

bool MrmHandler::isPullOverStatusAvailable()
{
  auto status = sub_mrm_pull_over_status_.takeData();
  if (status == nullptr) return false;
  return status->state != tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE;
}

bool MrmHandler::isComfortableStopStatusAvailable()
{
  auto status = sub_mrm_comfortable_stop_status_.takeData();
  if (status == nullptr) return false;
  return status->state != tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE;
}

bool MrmHandler::isEmergencyStopStatusAvailable()
{
  auto status = sub_mrm_emergency_stop_status_.takeData();
  if (status == nullptr) return false;
  return status->state != tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE;
}

bool MrmHandler::isArrivedAtGoal()
{
  using autoware_adapi_v1_msgs::msg::OperationModeState;
  auto state = sub_operation_mode_state_.takeData();
  if (state == nullptr) return false;
  return state->mode == OperationModeState::STOP;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MrmHandler)
