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

EmergencyHandler::EmergencyHandler(const rclcpp::NodeOptions & options)
: Node("emergency_handler", options)
{
  // Parameter
  param_.update_rate = declare_parameter<int>("update_rate");
  param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status");
  param_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped");
  param_.use_comfortable_stop = declare_parameter<bool>("use_comfortable_stop");
  param_.turning_hazard_on.emergency = declare_parameter<bool>("turning_hazard_on.emergency");

  using std::placeholders::_1;

  // Subscribers with callback
  sub_hazard_status_stamped_ = create_subscription<autoware_system_msgs::msg::HazardStatusStamped>(
    "~/input/hazard_status", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onHazardStatusStamped, this, _1));
  sub_prev_control_command_ = create_subscription<autoware_control_msgs::msg::Control>(
    "~/input/prev_control_command", rclcpp::QoS{1},
    std::bind(&EmergencyHandler::onPrevControlCommand, this, _1));

  // Publisher
  pub_control_command_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_command", rclcpp::QoS{1});
  pub_hazard_cmd_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard", rclcpp::QoS{1});
  pub_gear_cmd_ =
    create_publisher<autoware_vehicle_msgs::msg::GearCommand>("~/output/gear", rclcpp::QoS{1});
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
  mrm_state_.stamp = this->now();
  mrm_state_.state = autoware_adapi_v1_msgs::msg::MrmState::NORMAL;
  mrm_state_.behavior = autoware_adapi_v1_msgs::msg::MrmState::NONE;
  is_hazard_status_timeout_ = false;

  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&EmergencyHandler::onTimer, this));
}

void EmergencyHandler::onHazardStatusStamped(
  const autoware_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  hazard_status_stamped_ = msg;
  stamp_hazard_status_ = this->now();
}

void EmergencyHandler::onPrevControlCommand(
  const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
{
  auto control_command = new autoware_control_msgs::msg::Control(*msg);
  control_command->stamp = msg->stamp;
  prev_control_command_ = autoware_control_msgs::msg::Control::ConstSharedPtr(control_command);
}

autoware_vehicle_msgs::msg::HazardLightsCommand EmergencyHandler::createHazardCmdMsg()
{
  using autoware_vehicle_msgs::msg::HazardLightsCommand;
  HazardLightsCommand msg;

  // Check emergency
  const bool is_emergency = isEmergency();

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
  using autoware_vehicle_msgs::msg::GearCommand;

  // Create timestamp
  const auto stamp = this->now();

  // Publish hazard command
  pub_hazard_cmd_->publish(createHazardCmdMsg());

  // Publish gear
  {
    GearCommand msg;
    msg.stamp = stamp;
    const auto command = [&]() {
      // If stopped and use_parking is not true, send the last gear command
      if (isStopped())
        return (param_.use_parking_after_stopped) ? GearCommand::PARK : last_gear_command_;
      return (isDrivingBackwards()) ? GearCommand::REVERSE : GearCommand::DRIVE;
    }();

    msg.command = command;
    last_gear_command_ = msg.command;
    pub_gear_cmd_->publish(msg);
    return;
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

  if (mrm_behavior == MrmState::NONE) {
    RCLCPP_WARN(this->get_logger(), "MRM behavior is None. Do nothing.");
    return;
  }
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

  if (mrm_behavior == MrmState::NONE) {
    // Do nothing
    return;
  }
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

void EmergencyHandler::checkHazardStatusTimeout()
{
  if ((this->now() - stamp_hazard_status_).seconds() > param_.timeout_hazard_status) {
    is_hazard_status_timeout_ = true;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "heartbeat_hazard_status is timeout");
  } else {
    is_hazard_status_timeout_ = false;
  }
}

void EmergencyHandler::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  // Check whether heartbeat hazard_status is timeout
  checkHazardStatusTimeout();

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

  RCLCPP_DEBUG(
    this->get_logger(), "MRM State changed: %s -> %s", state2string(mrm_state_.state),
    state2string(new_state));

  mrm_state_.state = new_state;
}

void EmergencyHandler::updateMrmState()
{
  using autoware_adapi_v1_msgs::msg::MrmState;
  using autoware_vehicle_msgs::msg::ControlModeReport;

  // Check emergency
  const bool is_emergency = isEmergency();

  // Get mode
  const bool is_auto_mode = isAutonomous();

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
  using autoware_system_msgs::msg::HazardStatus;

  // Get hazard level
  auto level = hazard_status_stamped_->status.level;
  if (is_hazard_status_timeout_) {
    level = HazardStatus::SINGLE_POINT_FAULT;
  }

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

bool EmergencyHandler::isAutonomous()
{
  using autoware_vehicle_msgs::msg::ControlModeReport;
  auto mode = sub_control_mode_.takeData();
  if (mode == nullptr) return false;
  return mode->mode == ControlModeReport::AUTONOMOUS;
}

bool EmergencyHandler::isEmergency()
{
  return hazard_status_stamped_->status.emergency ||
         hazard_status_stamped_->status.emergency_holding || is_hazard_status_timeout_;
}

bool EmergencyHandler::isStopped()
{
  auto odom = sub_odom_.takeData();
  if (odom == nullptr) return false;
  constexpr auto th_stopped_velocity = 0.001;
  return (std::abs(odom->twist.twist.linear.x) < th_stopped_velocity);
}

bool EmergencyHandler::isComfortableStopStatusAvailable()
{
  auto status = sub_mrm_comfortable_stop_status_.takeData();
  if (status == nullptr) return false;
  return status->state != tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE;
}

bool EmergencyHandler::isEmergencyStopStatusAvailable()
{
  auto status = sub_mrm_emergency_stop_status_.takeData();
  if (status == nullptr) return false;
  return status->state != tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE;
}

bool EmergencyHandler::isDrivingBackwards()
{
  auto odom = sub_odom_.takeData();
  if (odom == nullptr) return false;
  constexpr auto th_moving_backwards = -0.001;
  return odom->twist.twist.linear.x < th_moving_backwards;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(EmergencyHandler)
