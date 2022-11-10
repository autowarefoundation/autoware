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

#include "emergency_handler/emergency_handler_core.hpp"

#include <memory>
#include <string>
#include <utility>

EmergencyHandler::EmergencyHandler() : Node("emergency_handler")
{
  // Parameter
  param_.update_rate = declare_parameter<int>("update_rate", 10);
  param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);
  param_.timeout_takeover_request = declare_parameter<double>("timeout_takeover_request", 10.0);
  param_.use_takeover_request = declare_parameter<bool>("use_takeover_request", false);
  param_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped", false);
  param_.use_comfortable_stop = declare_parameter<bool>("use_comfortable_stop", false);
  param_.turning_hazard_on.emergency = declare_parameter<bool>("turning_hazard_on.emergency", true);

  using std::placeholders::_1;

  // Subscriber
  sub_hazard_status_stamped_ =
    create_subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>(
      "~/input/hazard_status", rclcpp::QoS{1},
      std::bind(&EmergencyHandler::onHazardStatusStamped, this, _1));
  sub_prev_control_command_ =
    create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "~/input/prev_control_command", rclcpp::QoS{1},
      std::bind(&EmergencyHandler::onPrevControlCommand, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&EmergencyHandler::onOdometry, this, _1));
  // subscribe control mode
  sub_control_mode_ = create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "~/input/control_mode", rclcpp::QoS{1}, std::bind(&EmergencyHandler::onControlMode, this, _1));
  sub_mrm_comfortable_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
    "~/input/mrm/comfortable_stop/status", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onMrmComfortableStopStatus, this, _1));
  sub_mrm_emergency_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
    "~/input/mrm/emergency_stop/status", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onMrmEmergencyStopStatus, this, _1));

  // Publisher
  pub_control_command_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "~/output/control_command", rclcpp::QoS{1});
  pub_hazard_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard", rclcpp::QoS{1});
  pub_gear_cmd_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("~/output/gear", rclcpp::QoS{1});
  pub_mrm_state_ =
    create_publisher<autoware_adapi_v1_msgs::msg::MrmState>("~/output/mrm/state", rclcpp::QoS{1});

  // Clients
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
  prev_control_command_ = autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr(
    new autoware_auto_control_msgs::msg::AckermannControlCommand);
  mrm_comfortable_stop_status_ =
    std::make_shared<const tier4_system_msgs::msg::MrmBehaviorStatus>();
  mrm_emergency_stop_status_ = std::make_shared<const tier4_system_msgs::msg::MrmBehaviorStatus>();
  mrm_state_.stamp = this->now();
  mrm_state_.state = autoware_adapi_v1_msgs::msg::MrmState::NORMAL;
  mrm_state_.behavior = autoware_adapi_v1_msgs::msg::MrmState::NONE;

  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&EmergencyHandler::onTimer, this));
}

void EmergencyHandler::onHazardStatusStamped(
  const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  hazard_status_stamped_ = msg;
  stamp_hazard_status_ = this->now();
}

void EmergencyHandler::onPrevControlCommand(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  auto control_command = new autoware_auto_control_msgs::msg::AckermannControlCommand(*msg);
  control_command->stamp = msg->stamp;
  prev_control_command_ =
    autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr(control_command);
}

void EmergencyHandler::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void EmergencyHandler::onControlMode(
  const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg)
{
  control_mode_ = msg;
}

void EmergencyHandler::onMrmComfortableStopStatus(
  const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg)
{
  mrm_comfortable_stop_status_ = msg;
}

void EmergencyHandler::onMrmEmergencyStopStatus(
  const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg)
{
  mrm_emergency_stop_status_ = msg;
}

autoware_auto_vehicle_msgs::msg::HazardLightsCommand EmergencyHandler::createHazardCmdMsg()
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  HazardLightsCommand msg;

  // Check emergency
  const bool is_emergency = isEmergency(hazard_status_stamped_->status);

  if (hazard_status_stamped_->status.emergency_holding) {
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

void EmergencyHandler::publishControlCommands()
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

void EmergencyHandler::publishMrmState()
{
  mrm_state_.stamp = this->now();
  pub_mrm_state_->publish(mrm_state_);
}

void EmergencyHandler::operateMrm()
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

void EmergencyHandler::callMrmBehavior(
  const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior) const
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  auto request = std::make_shared<tier4_system_msgs::srv::OperateMrm::Request>();
  request->operate = true;

  if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    auto result = client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is operated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop is failed to operate");
    }
    return;
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    auto result = client_mrm_emergency_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is operated");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Emergency stop is failed to operate");
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}

void EmergencyHandler::cancelMrmBehavior(
  const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior) const
{
  using autoware_adapi_v1_msgs::msg::MrmState;

  auto request = std::make_shared<tier4_system_msgs::srv::OperateMrm::Request>();
  request->operate = false;

  if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    auto result = client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is canceled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop is failed to cancel");
    }
    return;
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    auto result = client_mrm_emergency_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is canceled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Emergency stop is failed to cancel");
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}

bool EmergencyHandler::isDataReady()
{
  if (!hazard_status_stamped_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for hazard_status_stamped msg...");
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

void EmergencyHandler::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  const bool is_hazard_status_timeout =
    (this->now() - stamp_hazard_status_).seconds() > param_.timeout_hazard_status;
  if (is_hazard_status_timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "heartbeat_hazard_status is timeout");
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

void EmergencyHandler::transitionTo(const int new_state)
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

void EmergencyHandler::updateMrmState()
{
  using autoware_adapi_v1_msgs::msg::MrmState;
  using autoware_auto_vehicle_msgs::msg::ControlModeReport;

  // Check emergency
  const bool is_emergency = isEmergency(hazard_status_stamped_->status);

  // Get mode
  const bool is_auto_mode = control_mode_->mode == ControlModeReport::AUTONOMOUS;
  const bool is_takeover_done = control_mode_->mode == ControlModeReport::MANUAL;

  // State Machine
  if (mrm_state_.state == MrmState::NORMAL) {
    // NORMAL
    if (is_auto_mode && is_emergency) {
      if (param_.use_takeover_request) {
        takeover_requested_time_ = this->get_clock()->now();
        is_takeover_request_ = true;
        return;
      } else {
        transitionTo(MrmState::MRM_OPERATING);
        return;
      }
    }
  } else {
    // Emergency
    // Send recovery events if "not emergency" or "takeover done"
    if (!is_emergency) {
      transitionTo(MrmState::NORMAL);
      return;
    }
    // TODO(Kenji Miyake): Check if human can safely override, for example using DSM
    if (is_takeover_done) {
      transitionTo(MrmState::NORMAL);
      return;
    }

    if (is_takeover_request_) {
      const auto time_from_takeover_request = this->get_clock()->now() - takeover_requested_time_;
      if (time_from_takeover_request.seconds() > param_.timeout_takeover_request) {
        transitionTo(MrmState::MRM_OPERATING);
        return;
      }
    } else if (mrm_state_.state == MrmState::MRM_OPERATING) {
      // TODO(Kenji Miyake): Check MRC is accomplished
      if (isStopped()) {
        transitionTo(MrmState::MRM_SUCCEEDED);
        return;
      }
    } else if (mrm_state_.state == MrmState::MRM_SUCCEEDED) {
      // Do nothing(only checking common recovery events)
    } else if (mrm_state_.state == MrmState::MRM_FAILED) {
      // Do nothing(only checking common recovery events)
    } else {
      const auto msg = "invalid state: " + std::to_string(mrm_state_.state);
      throw std::runtime_error(msg);
    }
  }
}

autoware_adapi_v1_msgs::msg::MrmState::_behavior_type EmergencyHandler::getCurrentMrmBehavior()
{
  using autoware_adapi_v1_msgs::msg::MrmState;
  using autoware_auto_system_msgs::msg::HazardStatus;

  // Get hazard level
  const auto level = hazard_status_stamped_->status.level;

  // State machine
  if (mrm_state_.behavior == MrmState::NONE) {
    if (level == HazardStatus::LATENT_FAULT) {
      if (param_.use_comfortable_stop) {
        return MrmState::COMFORTABLE_STOP;
      }
      return MrmState::EMERGENCY_STOP;
    }
    if (level == HazardStatus::SINGLE_POINT_FAULT) {
      return MrmState::EMERGENCY_STOP;
    }
  }
  if (mrm_state_.behavior == MrmState::COMFORTABLE_STOP) {
    if (level == HazardStatus::SINGLE_POINT_FAULT) {
      return MrmState::EMERGENCY_STOP;
    }
  }

  return mrm_state_.behavior;
}

bool EmergencyHandler::isEmergency(
  const autoware_auto_system_msgs::msg::HazardStatus & hazard_status)
{
  return hazard_status.emergency || hazard_status.emergency_holding;
}

bool EmergencyHandler::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (odom_->twist.twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}
