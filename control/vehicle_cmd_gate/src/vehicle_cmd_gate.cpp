// Copyright 2015-2019 Autoware Foundation
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

#include "vehicle_cmd_gate.hpp"

#include <rclcpp/logging.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace vehicle_cmd_gate
{

namespace
{
const char * getGateModeName(const GateMode::_data_type & gate_mode)
{
  if (gate_mode == GateMode::AUTO) {
    return "AUTO";
  }
  if (gate_mode == GateMode::EXTERNAL) {
    return "EXTERNAL";
  }
  return "NOT_SUPPORTED";
}

}  // namespace

VehicleCmdGate::VehicleCmdGate(const rclcpp::NodeOptions & node_options)
: Node("vehicle_cmd_gate", node_options), is_engaged_(false), updater_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  // Publisher
  vehicle_cmd_emergency_pub_ =
    this->create_publisher<VehicleEmergencyStamped>("output/vehicle_cmd_emergency", durable_qos);
  control_cmd_pub_ =
    this->create_publisher<AckermannControlCommand>("output/control_cmd", durable_qos);
  gear_cmd_pub_ = this->create_publisher<GearCommand>("output/gear_cmd", durable_qos);
  turn_indicator_cmd_pub_ =
    this->create_publisher<TurnIndicatorsCommand>("output/turn_indicators_cmd", durable_qos);
  hazard_light_cmd_pub_ =
    this->create_publisher<HazardLightsCommand>("output/hazard_lights_cmd", durable_qos);

  gate_mode_pub_ = this->create_publisher<GateMode>("output/gate_mode", durable_qos);
  engage_pub_ = this->create_publisher<EngageMsg>("output/engage", durable_qos);
  pub_external_emergency_ =
    this->create_publisher<Emergency>("output/external_emergency", durable_qos);
  operation_mode_pub_ =
    this->create_publisher<OperationModeState>("output/operation_mode", durable_qos);

  // Subscriber
  external_emergency_stop_heartbeat_sub_ = this->create_subscription<Heartbeat>(
    "input/external_emergency_stop_heartbeat", 1,
    std::bind(&VehicleCmdGate::onExternalEmergencyStopHeartbeat, this, _1));
  gate_mode_sub_ = this->create_subscription<GateMode>(
    "input/gate_mode", 1, std::bind(&VehicleCmdGate::onGateMode, this, _1));
  engage_sub_ = this->create_subscription<EngageMsg>(
    "input/engage", 1, std::bind(&VehicleCmdGate::onEngage, this, _1));
  steer_sub_ = this->create_subscription<SteeringReport>(
    "input/steering", 1, std::bind(&VehicleCmdGate::onSteering, this, _1));
  operation_mode_sub_ = this->create_subscription<OperationModeState>(
    "input/operation_mode", rclcpp::QoS(1).transient_local(),
    [this](const OperationModeState::SharedPtr msg) { current_operation_mode_ = *msg; });
  mrm_state_sub_ = this->create_subscription<MrmState>(
    "input/mrm_state", 1, std::bind(&VehicleCmdGate::onMrmState, this, _1));

  // Subscriber for auto
  auto_control_cmd_sub_ = this->create_subscription<AckermannControlCommand>(
    "input/auto/control_cmd", 1, std::bind(&VehicleCmdGate::onAutoCtrlCmd, this, _1));

  auto_turn_indicator_cmd_sub_ = this->create_subscription<TurnIndicatorsCommand>(
    "input/auto/turn_indicators_cmd", 1,
    std::bind(&VehicleCmdGate::onAutoTurnIndicatorsCmd, this, _1));

  auto_hazard_light_cmd_sub_ = this->create_subscription<HazardLightsCommand>(
    "input/auto/hazard_lights_cmd", 1, std::bind(&VehicleCmdGate::onAutoHazardLightsCmd, this, _1));

  auto_gear_cmd_sub_ = this->create_subscription<GearCommand>(
    "input/auto/gear_cmd", 1, std::bind(&VehicleCmdGate::onAutoShiftCmd, this, _1));

  // Subscriber for external
  remote_control_cmd_sub_ = this->create_subscription<AckermannControlCommand>(
    "input/external/control_cmd", 1, std::bind(&VehicleCmdGate::onRemoteCtrlCmd, this, _1));

  remote_turn_indicator_cmd_sub_ = this->create_subscription<TurnIndicatorsCommand>(
    "input/external/turn_indicators_cmd", 1,
    std::bind(&VehicleCmdGate::onRemoteTurnIndicatorsCmd, this, _1));

  remote_hazard_light_cmd_sub_ = this->create_subscription<HazardLightsCommand>(
    "input/external/hazard_lights_cmd", 1,
    std::bind(&VehicleCmdGate::onRemoteHazardLightsCmd, this, _1));

  remote_gear_cmd_sub_ = this->create_subscription<GearCommand>(
    "input/external/gear_cmd", 1, std::bind(&VehicleCmdGate::onRemoteShiftCmd, this, _1));

  // Subscriber for emergency
  emergency_control_cmd_sub_ = this->create_subscription<AckermannControlCommand>(
    "input/emergency/control_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyCtrlCmd, this, _1));

  emergency_hazard_light_cmd_sub_ = this->create_subscription<HazardLightsCommand>(
    "input/emergency/hazard_lights_cmd", 1,
    std::bind(&VehicleCmdGate::onEmergencyHazardLightsCmd, this, _1));

  emergency_gear_cmd_sub_ = this->create_subscription<GearCommand>(
    "input/emergency/gear_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyShiftCmd, this, _1));

  // Parameter
  update_period_ = 1.0 / declare_parameter<double>("update_rate");
  use_emergency_handling_ = declare_parameter<bool>("use_emergency_handling");
  check_external_emergency_heartbeat_ =
    declare_parameter<bool>("check_external_emergency_heartbeat");
  system_emergency_heartbeat_timeout_ =
    declare_parameter<double>("system_emergency_heartbeat_timeout");
  external_emergency_stop_heartbeat_timeout_ =
    declare_parameter<double>("external_emergency_stop_heartbeat_timeout");
  stop_hold_acceleration_ = declare_parameter<double>("stop_hold_acceleration");
  emergency_acceleration_ = declare_parameter<double>("emergency_acceleration");

  // Vehicle Parameter
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  {
    VehicleCmdFilterParam p;
    p.wheel_base = vehicle_info.wheel_base_m;
    p.vel_lim = declare_parameter<double>("nominal.vel_lim");
    p.lon_acc_lim = declare_parameter<double>("nominal.lon_acc_lim");
    p.lon_jerk_lim = declare_parameter<double>("nominal.lon_jerk_lim");
    p.lat_acc_lim = declare_parameter<double>("nominal.lat_acc_lim");
    p.lat_jerk_lim = declare_parameter<double>("nominal.lat_jerk_lim");
    p.actual_steer_diff_lim = declare_parameter<double>("nominal.actual_steer_diff_lim");
    filter_.setParam(p);
  }

  {
    VehicleCmdFilterParam p;
    p.wheel_base = vehicle_info.wheel_base_m;
    p.vel_lim = declare_parameter<double>("on_transition.vel_lim");
    p.lon_acc_lim = declare_parameter<double>("on_transition.lon_acc_lim");
    p.lon_jerk_lim = declare_parameter<double>("on_transition.lon_jerk_lim");
    p.lat_acc_lim = declare_parameter<double>("on_transition.lat_acc_lim");
    p.lat_jerk_lim = declare_parameter<double>("on_transition.lat_jerk_lim");
    p.actual_steer_diff_lim = declare_parameter<double>("on_transition.actual_steer_diff_lim");
    filter_on_transition_.setParam(p);
  }

  // Set default value
  current_gate_mode_.data = GateMode::AUTO;
  current_operation_mode_.mode = OperationModeState::STOP;

  // Service
  srv_engage_ = create_service<tier4_external_api_msgs::srv::Engage>(
    "~/service/engage", std::bind(&VehicleCmdGate::onEngageService, this, _1, _2));
  srv_external_emergency_ = create_service<tier4_external_api_msgs::srv::SetEmergency>(
    "~/service/external_emergency",
    std::bind(&VehicleCmdGate::onExternalEmergencyStopService, this, _1, _2, _3));
  srv_external_emergency_stop_ = this->create_service<std_srvs::srv::Trigger>(
    "~/service/external_emergency_stop",
    std::bind(&VehicleCmdGate::onSetExternalEmergencyStopService, this, _1, _2, _3));
  srv_clear_external_emergency_stop_ = this->create_service<std_srvs::srv::Trigger>(
    "~/service/clear_external_emergency_stop",
    std::bind(&VehicleCmdGate::onClearExternalEmergencyStopService, this, _1, _2, _3));

  // Diagnostics Updater
  updater_.setHardwareID("vehicle_cmd_gate");
  updater_.add("heartbeat", [](auto & stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
  });
  updater_.add("emergency_stop_operation", this, &VehicleCmdGate::checkExternalEmergencyStop);

  // Pause interface
  pause_ = std::make_unique<PauseInterface>(this);

  // Timer
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_period_));
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&VehicleCmdGate::onTimer, this));
  timer_pub_status_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&VehicleCmdGate::publishStatus, this));
}

bool VehicleCmdGate::isHeartbeatTimeout(
  const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout)
{
  if (timeout == 0.0) {
    return false;
  }

  if (!heartbeat_received_time) {
    return true;
  }

  const auto time_from_heartbeat = this->now() - *heartbeat_received_time;

  return time_from_heartbeat.seconds() > timeout;
}

bool VehicleCmdGate::isDataReady()
{
  // emergency state must be received before running
  if (use_emergency_handling_) {
    if (!emergency_state_heartbeat_received_time_) {
      RCLCPP_WARN(get_logger(), "emergency_state_heartbeat_received_time_ is false");
      return false;
    }
  }

  if (check_external_emergency_heartbeat_) {
    if (!external_emergency_stop_heartbeat_received_time_) {
      RCLCPP_WARN(get_logger(), "external_emergency_stop_heartbeat_received_time_ is false");
      return false;
    }
  }

  return true;
}

// for auto
void VehicleCmdGate::onAutoCtrlCmd(AckermannControlCommand::ConstSharedPtr msg)
{
  auto_commands_.control = *msg;

  if (current_gate_mode_.data == GateMode::AUTO) {
    publishControlCommands(auto_commands_);
  }
}

void VehicleCmdGate::onAutoTurnIndicatorsCmd(TurnIndicatorsCommand::ConstSharedPtr msg)
{
  auto_commands_.turn_indicator = *msg;
}

void VehicleCmdGate::onAutoHazardLightsCmd(HazardLightsCommand::ConstSharedPtr msg)
{
  auto_commands_.hazard_light = *msg;
}

void VehicleCmdGate::onAutoShiftCmd(GearCommand::ConstSharedPtr msg) { auto_commands_.gear = *msg; }

// for remote
void VehicleCmdGate::onRemoteCtrlCmd(AckermannControlCommand::ConstSharedPtr msg)
{
  remote_commands_.control = *msg;

  if (current_gate_mode_.data == GateMode::EXTERNAL) {
    publishControlCommands(remote_commands_);
  }
}

void VehicleCmdGate::onRemoteTurnIndicatorsCmd(TurnIndicatorsCommand::ConstSharedPtr msg)
{
  remote_commands_.turn_indicator = *msg;
}

void VehicleCmdGate::onRemoteHazardLightsCmd(HazardLightsCommand::ConstSharedPtr msg)
{
  remote_commands_.hazard_light = *msg;
}

void VehicleCmdGate::onRemoteShiftCmd(GearCommand::ConstSharedPtr msg)
{
  remote_commands_.gear = *msg;
}

// for emergency
void VehicleCmdGate::onEmergencyCtrlCmd(AckermannControlCommand::ConstSharedPtr msg)
{
  emergency_commands_.control = *msg;

  if (use_emergency_handling_ && is_system_emergency_) {
    publishControlCommands(emergency_commands_);
  }
}
void VehicleCmdGate::onEmergencyHazardLightsCmd(HazardLightsCommand::ConstSharedPtr msg)
{
  emergency_commands_.hazard_light = *msg;
}
void VehicleCmdGate::onEmergencyShiftCmd(GearCommand::ConstSharedPtr msg)
{
  emergency_commands_.gear = *msg;
}

void VehicleCmdGate::onTimer()
{
  updater_.force_update();

  if (!isDataReady()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting topics...");
    return;
  }

  // Check system emergency heartbeat
  if (use_emergency_handling_) {
    is_emergency_state_heartbeat_timeout_ = isHeartbeatTimeout(
      emergency_state_heartbeat_received_time_, system_emergency_heartbeat_timeout_);

    if (is_emergency_state_heartbeat_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000 /*ms*/, "system_emergency heartbeat is timeout.");
      publishEmergencyStopControlCommands();
      return;
    }
  }

  // Check external emergency stop heartbeat
  if (check_external_emergency_heartbeat_) {
    is_external_emergency_stop_heartbeat_timeout_ = isHeartbeatTimeout(
      external_emergency_stop_heartbeat_received_time_, external_emergency_stop_heartbeat_timeout_);

    if (is_external_emergency_stop_heartbeat_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000 /*ms*/, "external_emergency_stop heartbeat is timeout.");
      is_external_emergency_stop_ = true;
    }
  }

  // Check external emergency stop
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000 /*ms*/,
        "Please call `clear_external_emergency_stop` service to clear state.");
    }

    publishEmergencyStopControlCommands();
    return;
  }

  // Select commands
  TurnIndicatorsCommand turn_indicator;
  HazardLightsCommand hazard_light;
  GearCommand gear;
  if (use_emergency_handling_ && is_system_emergency_) {
    turn_indicator = emergency_commands_.turn_indicator;
    hazard_light = emergency_commands_.hazard_light;
    gear = emergency_commands_.gear;
  } else {
    if (current_gate_mode_.data == GateMode::AUTO) {
      turn_indicator = auto_commands_.turn_indicator;
      hazard_light = auto_commands_.hazard_light;
      gear = auto_commands_.gear;

      // Don't send turn signal when autoware is not engaged
      if (!is_engaged_) {
        turn_indicator.command = TurnIndicatorsCommand::NO_COMMAND;
        hazard_light.command = HazardLightsCommand::NO_COMMAND;
      }
    } else if (current_gate_mode_.data == GateMode::EXTERNAL) {
      turn_indicator = remote_commands_.turn_indicator;
      hazard_light = remote_commands_.hazard_light;
      gear = remote_commands_.gear;
    } else {
      throw std::runtime_error("invalid mode");
    }
  }

  // Publish topics
  turn_indicator_cmd_pub_->publish(turn_indicator);
  hazard_light_cmd_pub_->publish(hazard_light);
  gear_cmd_pub_->publish(gear);
}

void VehicleCmdGate::publishControlCommands(const Commands & commands)
{
  // Check system emergency
  if (use_emergency_handling_ && is_emergency_state_heartbeat_timeout_) {
    return;
  }

  // Check external emergency stop
  if (is_external_emergency_stop_) {
    return;
  }

  // Check initialization is done
  if (!isDataReady()) {
    return;
  }

  Commands filtered_commands;

  // Set default commands
  {
    filtered_commands.control = commands.control;
    filtered_commands.gear = commands.gear;  // tmp
  }

  // Check emergency
  if (use_emergency_handling_ && is_system_emergency_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "Emergency!");
    filtered_commands.control = emergency_commands_.control;
    filtered_commands.gear = emergency_commands_.gear;  // tmp
  }

  // Check engage
  if (!is_engaged_) {
    filtered_commands.control = createStopControlCmd();
  }

  // Check pause
  pause_->update(filtered_commands.control);
  if (pause_->is_paused()) {
    filtered_commands.control.longitudinal.speed = 0.0;
    filtered_commands.control.longitudinal.acceleration = stop_hold_acceleration_;
  }

  // Apply limit filtering
  filtered_commands.control = filterControlCommand(filtered_commands.control);

  // tmp: Publish vehicle emergency status
  VehicleEmergencyStamped vehicle_cmd_emergency;
  vehicle_cmd_emergency.emergency = (use_emergency_handling_ && is_system_emergency_);
  vehicle_cmd_emergency.stamp = filtered_commands.control.stamp;

  // Publish commands
  vehicle_cmd_emergency_pub_->publish(vehicle_cmd_emergency);
  control_cmd_pub_->publish(filtered_commands.control);

  // Save ControlCmd to steering angle when disengaged
  prev_control_cmd_ = filtered_commands.control;
}

void VehicleCmdGate::publishEmergencyStopControlCommands()
{
  const auto stamp = this->now();

  // ControlCommand
  AckermannControlCommand control_cmd;
  control_cmd.stamp = stamp;
  control_cmd = createEmergencyStopControlCmd();

  // Update control command
  pause_->update(control_cmd);

  // gear
  GearCommand gear;
  gear.stamp = stamp;
  // default value is 0

  // TurnSignal
  TurnIndicatorsCommand turn_indicator;
  turn_indicator.stamp = stamp;
  turn_indicator.command = TurnIndicatorsCommand::NO_COMMAND;

  // Hazard
  HazardLightsCommand hazard_light;
  hazard_light.stamp = stamp;
  hazard_light.command = HazardLightsCommand::ENABLE;

  // VehicleCommand emergency;
  VehicleEmergencyStamped vehicle_cmd_emergency;
  vehicle_cmd_emergency.stamp = stamp;
  vehicle_cmd_emergency.emergency = true;

  // Publish topics
  vehicle_cmd_emergency_pub_->publish(vehicle_cmd_emergency);
  control_cmd_pub_->publish(control_cmd);
  turn_indicator_cmd_pub_->publish(turn_indicator);
  hazard_light_cmd_pub_->publish(hazard_light);
  gear_cmd_pub_->publish(gear);
}

void VehicleCmdGate::publishStatus()
{
  const auto stamp = this->now();

  // Engage
  EngageMsg autoware_engage;
  autoware_engage.stamp = stamp;
  autoware_engage.engage = is_engaged_;

  // External emergency
  Emergency external_emergency;
  external_emergency.stamp = stamp;
  external_emergency.emergency = is_external_emergency_stop_;

  gate_mode_pub_->publish(current_gate_mode_);
  engage_pub_->publish(autoware_engage);
  pub_external_emergency_->publish(external_emergency);
  operation_mode_pub_->publish(current_operation_mode_);
  pause_->publish();
}

AckermannControlCommand VehicleCmdGate::filterControlCommand(const AckermannControlCommand & in)
{
  AckermannControlCommand out = in;
  const double dt = getDt();
  const auto mode = current_operation_mode_;

  // Apply transition_filter when transiting from MANUAL to AUTO.
  if (mode.is_in_transition) {
    filter_on_transition_.filterAll(dt, current_steer_, out);
  } else {
    filter_.filterAll(dt, current_steer_, out);
  }

  // set prev value for both to keep consistency over switching
  // TODO(Horibe): prev value should be actual steer, vel, acc when Manual mode to keep
  // consistency when switching from Manual to Auto.
  filter_.setPrevCmd(out);
  filter_on_transition_.setPrevCmd(out);

  return out;
}

AckermannControlCommand VehicleCmdGate::createStopControlCmd() const
{
  AckermannControlCommand cmd;
  const auto t = this->now();
  cmd.stamp = t;
  cmd.lateral.stamp = t;
  cmd.longitudinal.stamp = t;
  cmd.lateral.steering_tire_angle = current_steer_;
  cmd.lateral.steering_tire_rotation_rate = 0.0;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = stop_hold_acceleration_;

  return cmd;
}

AckermannControlCommand VehicleCmdGate::createEmergencyStopControlCmd() const
{
  AckermannControlCommand cmd;
  const auto t = this->now();
  cmd.stamp = t;
  cmd.lateral.stamp = t;
  cmd.longitudinal.stamp = t;
  cmd.lateral.steering_tire_angle = prev_control_cmd_.lateral.steering_tire_angle;
  cmd.lateral.steering_tire_rotation_rate = prev_control_cmd_.lateral.steering_tire_rotation_rate;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = emergency_acceleration_;

  return cmd;
}

void VehicleCmdGate::onExternalEmergencyStopHeartbeat(
  [[maybe_unused]] Heartbeat::ConstSharedPtr msg)
{
  external_emergency_stop_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
}

void VehicleCmdGate::onGateMode(GateMode::ConstSharedPtr msg)
{
  const auto prev_gate_mode = current_gate_mode_;
  current_gate_mode_ = *msg;

  if (current_gate_mode_.data != prev_gate_mode.data) {
    RCLCPP_INFO(
      get_logger(), "GateMode changed: %s -> %s", getGateModeName(prev_gate_mode.data),
      getGateModeName(current_gate_mode_.data));
  }
}

void VehicleCmdGate::onEngage(EngageMsg::ConstSharedPtr msg) { is_engaged_ = msg->engage; }

void VehicleCmdGate::onEngageService(
  const tier4_external_api_msgs::srv::Engage::Request::SharedPtr request,
  const tier4_external_api_msgs::srv::Engage::Response::SharedPtr response)
{
  is_engaged_ = request->engage;
  response->status = tier4_api_utils::response_success();
}

void VehicleCmdGate::onSteering(SteeringReport::ConstSharedPtr msg)
{
  current_steer_ = msg->steering_tire_angle;
}

void VehicleCmdGate::onMrmState(MrmState::ConstSharedPtr msg)
{
  is_system_emergency_ =
    (msg->state == MrmState::MRM_OPERATING || msg->state == MrmState::MRM_SUCCEEDED ||
     msg->state == MrmState::MRM_FAILED) &&
    (msg->behavior == MrmState::EMERGENCY_STOP);
  emergency_state_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
}

double VehicleCmdGate::getDt()
{
  if (!prev_time_) {
    prev_time_ = std::make_shared<rclcpp::Time>(this->now());
    return 0.0;
  }

  const auto current_time = this->now();
  const auto dt = (current_time - *prev_time_).seconds();
  *prev_time_ = current_time;

  return dt;
}

void VehicleCmdGate::onExternalEmergencyStopService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<tier4_external_api_msgs::srv::SetEmergency::Request> request,
  const std::shared_ptr<tier4_external_api_msgs::srv::SetEmergency::Response> response)
{
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
  if (request->emergency) {
    onSetExternalEmergencyStopService(request_header, req, res);
  } else {
    onClearExternalEmergencyStopService(request_header, req, res);
  }

  if (res->success) {
    response->status = tier4_api_utils::response_success(res->message);
  } else {
    response->status = tier4_api_utils::response_error(res->message);
  }
}

bool VehicleCmdGate::onSetExternalEmergencyStopService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  is_external_emergency_stop_ = true;
  res->success = true;
  res->message = "external_emergency_stop requested was accepted.";

  return true;
}

bool VehicleCmdGate::onClearExternalEmergencyStopService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  if (is_external_emergency_stop_) {
    if (!is_external_emergency_stop_heartbeat_timeout_) {
      is_external_emergency_stop_ = false;
      res->success = true;
      res->message = "external_emergency_stop state was cleared.";
    } else {
      res->success = false;
      res->message = "Couldn't clear external_emergency_stop state because heartbeat is timeout.";
    }
  } else {
    res->success = false;
    res->message = "Not in external_emergency_stop state.";
  }

  return true;
}

void VehicleCmdGate::checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  DiagnosticStatus status;
  if (is_external_emergency_stop_heartbeat_timeout_) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "external_emergency_stop heartbeat is timeout.";
  } else if (is_external_emergency_stop_) {
    status.level = DiagnosticStatus::ERROR;
    status.message =
      "external_emergency_stop is required. Please call `clear_external_emergency_stop` service to "
      "clear state.";
  } else {
    status.level = DiagnosticStatus::OK;
  }

  stat.summary(status.level, status.message);
}

}  // namespace vehicle_cmd_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_cmd_gate::VehicleCmdGate)
