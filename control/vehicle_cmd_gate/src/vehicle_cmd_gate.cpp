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

#include "marker_helper.hpp"

#include <rclcpp/logging.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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

  // Stop Checker
  vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);

  // Publisher
  vehicle_cmd_emergency_pub_ =
    create_publisher<VehicleEmergencyStamped>("output/vehicle_cmd_emergency", durable_qos);
  control_cmd_pub_ = create_publisher<AckermannControlCommand>("output/control_cmd", durable_qos);
  gear_cmd_pub_ = create_publisher<GearCommand>("output/gear_cmd", durable_qos);
  turn_indicator_cmd_pub_ =
    create_publisher<TurnIndicatorsCommand>("output/turn_indicators_cmd", durable_qos);
  hazard_light_cmd_pub_ =
    create_publisher<HazardLightsCommand>("output/hazard_lights_cmd", durable_qos);

  gate_mode_pub_ = create_publisher<GateMode>("output/gate_mode", durable_qos);
  engage_pub_ = create_publisher<EngageMsg>("output/engage", durable_qos);
  pub_external_emergency_ = create_publisher<Emergency>("output/external_emergency", durable_qos);
  operation_mode_pub_ = create_publisher<OperationModeState>("output/operation_mode", durable_qos);

  is_filter_activated_pub_ =
    create_publisher<IsFilterActivated>("~/is_filter_activated", durable_qos);
  filter_activated_marker_pub_ =
    create_publisher<MarkerArray>("~/is_filter_activated/marker", durable_qos);
  filter_activated_marker_raw_pub_ =
    create_publisher<MarkerArray>("~/is_filter_activated/marker_raw", durable_qos);
  filter_activated_flag_pub_ =
    create_publisher<BoolStamped>("~/is_filter_activated/flag", durable_qos);

  // Subscriber
  external_emergency_stop_heartbeat_sub_ = create_subscription<Heartbeat>(
    "input/external_emergency_stop_heartbeat", 1,
    std::bind(&VehicleCmdGate::onExternalEmergencyStopHeartbeat, this, _1));
  gate_mode_sub_ = create_subscription<GateMode>(
    "input/gate_mode", 1, std::bind(&VehicleCmdGate::onGateMode, this, _1));
  engage_sub_ = create_subscription<EngageMsg>(
    "input/engage", 1, std::bind(&VehicleCmdGate::onEngage, this, _1));
  kinematics_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    [this](Odometry::SharedPtr msg) { current_kinematics_ = *msg; });
  acc_sub_ = create_subscription<AccelWithCovarianceStamped>(
    "input/acceleration", 1, [this](AccelWithCovarianceStamped::SharedPtr msg) {
      current_acceleration_ = msg->accel.accel.linear.x;
    });
  steer_sub_ = create_subscription<SteeringReport>(
    "input/steering", 1,
    [this](SteeringReport::SharedPtr msg) { current_steer_ = msg->steering_tire_angle; });
  operation_mode_sub_ = create_subscription<OperationModeState>(
    "input/operation_mode", rclcpp::QoS(1).transient_local(),
    [this](const OperationModeState::SharedPtr msg) { current_operation_mode_ = *msg; });
  mrm_state_sub_ = create_subscription<MrmState>(
    "input/mrm_state", 1, std::bind(&VehicleCmdGate::onMrmState, this, _1));

  // Subscriber for auto
  auto_control_cmd_sub_ = create_subscription<AckermannControlCommand>(
    "input/auto/control_cmd", 1, std::bind(&VehicleCmdGate::onAutoCtrlCmd, this, _1));

  auto_turn_indicator_cmd_sub_ = create_subscription<TurnIndicatorsCommand>(
    "input/auto/turn_indicators_cmd", 1,
    [this](TurnIndicatorsCommand::ConstSharedPtr msg) { auto_commands_.turn_indicator = *msg; });

  auto_hazard_light_cmd_sub_ = create_subscription<HazardLightsCommand>(
    "input/auto/hazard_lights_cmd", 1,
    [this](HazardLightsCommand::ConstSharedPtr msg) { auto_commands_.hazard_light = *msg; });

  auto_gear_cmd_sub_ = create_subscription<GearCommand>(
    "input/auto/gear_cmd", 1,
    [this](GearCommand::ConstSharedPtr msg) { auto_commands_.gear = *msg; });

  // Subscriber for external
  remote_control_cmd_sub_ = create_subscription<AckermannControlCommand>(
    "input/external/control_cmd", 1, std::bind(&VehicleCmdGate::onRemoteCtrlCmd, this, _1));

  remote_turn_indicator_cmd_sub_ = create_subscription<TurnIndicatorsCommand>(
    "input/external/turn_indicators_cmd", 1,
    [this](TurnIndicatorsCommand::ConstSharedPtr msg) { remote_commands_.turn_indicator = *msg; });

  remote_hazard_light_cmd_sub_ = create_subscription<HazardLightsCommand>(
    "input/external/hazard_lights_cmd", 1,
    [this](HazardLightsCommand::ConstSharedPtr msg) { remote_commands_.hazard_light = *msg; });

  remote_gear_cmd_sub_ = create_subscription<GearCommand>(
    "input/external/gear_cmd", 1,
    [this](GearCommand::ConstSharedPtr msg) { remote_commands_.gear = *msg; });

  // Subscriber for emergency
  emergency_control_cmd_sub_ = create_subscription<AckermannControlCommand>(
    "input/emergency/control_cmd", 1, std::bind(&VehicleCmdGate::onEmergencyCtrlCmd, this, _1));

  emergency_hazard_light_cmd_sub_ = create_subscription<HazardLightsCommand>(
    "input/emergency/hazard_lights_cmd", 1,
    [this](HazardLightsCommand::ConstSharedPtr msg) { emergency_commands_.hazard_light = *msg; });

  emergency_gear_cmd_sub_ = create_subscription<GearCommand>(
    "input/emergency/gear_cmd", 1,
    [this](GearCommand::ConstSharedPtr msg) { emergency_commands_.gear = *msg; });

  // Parameter
  use_emergency_handling_ = declare_parameter<bool>("use_emergency_handling");
  check_external_emergency_heartbeat_ =
    declare_parameter<bool>("check_external_emergency_heartbeat");
  system_emergency_heartbeat_timeout_ =
    declare_parameter<double>("system_emergency_heartbeat_timeout");
  external_emergency_stop_heartbeat_timeout_ =
    declare_parameter<double>("external_emergency_stop_heartbeat_timeout");
  stop_hold_acceleration_ = declare_parameter<double>("stop_hold_acceleration");
  emergency_acceleration_ = declare_parameter<double>("emergency_acceleration");
  moderate_stop_service_acceleration_ =
    declare_parameter<double>("moderate_stop_service_acceleration");
  stop_check_duration_ = declare_parameter<double>("stop_check_duration");
  enable_cmd_limit_filter_ = declare_parameter<bool>("enable_cmd_limit_filter");
  filter_activated_count_threshold_ = declare_parameter<int>("filter_activated_count_threshold");
  filter_activated_velocity_threshold_ =
    declare_parameter<double>("filter_activated_velocity_threshold");

  // Vehicle Parameter
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  {
    VehicleCmdFilterParam p;
    p.wheel_base = vehicle_info.wheel_base_m;
    p.vel_lim = declare_parameter<double>("nominal.vel_lim");
    p.reference_speed_points =
      declare_parameter<std::vector<double>>("nominal.reference_speed_points");
    p.steer_lim = declare_parameter<std::vector<double>>("nominal.steer_lim");
    p.steer_rate_lim = declare_parameter<std::vector<double>>("nominal.steer_rate_lim");
    p.lon_acc_lim = declare_parameter<std::vector<double>>("nominal.lon_acc_lim");
    p.lon_jerk_lim = declare_parameter<std::vector<double>>("nominal.lon_jerk_lim");
    p.lat_acc_lim = declare_parameter<std::vector<double>>("nominal.lat_acc_lim");
    p.lat_jerk_lim = declare_parameter<std::vector<double>>("nominal.lat_jerk_lim");
    p.actual_steer_diff_lim =
      declare_parameter<std::vector<double>>("nominal.actual_steer_diff_lim");
    filter_.setParam(p);
  }

  {
    VehicleCmdFilterParam p;
    p.wheel_base = vehicle_info.wheel_base_m;
    p.vel_lim = declare_parameter<double>("on_transition.vel_lim");
    p.reference_speed_points =
      declare_parameter<std::vector<double>>("on_transition.reference_speed_points");
    p.steer_lim = declare_parameter<std::vector<double>>("on_transition.steer_lim");
    p.steer_rate_lim = declare_parameter<std::vector<double>>("on_transition.steer_rate_lim");
    p.lon_acc_lim = declare_parameter<std::vector<double>>("on_transition.lon_acc_lim");
    p.lon_jerk_lim = declare_parameter<std::vector<double>>("on_transition.lon_jerk_lim");
    p.lat_acc_lim = declare_parameter<std::vector<double>>("on_transition.lat_acc_lim");
    p.lat_jerk_lim = declare_parameter<std::vector<double>>("on_transition.lat_jerk_lim");
    p.actual_steer_diff_lim =
      declare_parameter<std::vector<double>>("on_transition.actual_steer_diff_lim");
    filter_on_transition_.setParam(p);
  }

  // Set default value
  current_gate_mode_.data = GateMode::AUTO;
  current_operation_mode_.mode = OperationModeState::STOP;

  // Service
  srv_engage_ = create_service<EngageSrv>(
    "~/service/engage", std::bind(&VehicleCmdGate::onEngageService, this, _1, _2));
  srv_external_emergency_ = create_service<SetEmergency>(
    "~/service/external_emergency",
    std::bind(&VehicleCmdGate::onExternalEmergencyStopService, this, _1, _2, _3));
  srv_external_emergency_stop_ = create_service<Trigger>(
    "~/service/external_emergency_stop",
    std::bind(&VehicleCmdGate::onSetExternalEmergencyStopService, this, _1, _2, _3));
  srv_clear_external_emergency_stop_ = create_service<Trigger>(
    "~/service/clear_external_emergency_stop",
    std::bind(&VehicleCmdGate::onClearExternalEmergencyStopService, this, _1, _2, _3));

  // Diagnostics Updater
  updater_.setHardwareID("vehicle_cmd_gate");
  updater_.add("heartbeat", [](auto & stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
  });
  updater_.add("emergency_stop_operation", this, &VehicleCmdGate::checkExternalEmergencyStop);

  // Pause interface
  adapi_pause_ = std::make_unique<AdapiPauseInterface>(this);
  moderate_stop_interface_ = std::make_unique<ModerateStopInterface>(this);

  // Timer
  const auto update_period = 1.0 / declare_parameter<double>("update_rate");
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_period));
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&VehicleCmdGate::onTimer, this));
  timer_pub_status_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&VehicleCmdGate::publishStatus, this));

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
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

// for remote
void VehicleCmdGate::onRemoteCtrlCmd(AckermannControlCommand::ConstSharedPtr msg)
{
  remote_commands_.control = *msg;

  if (current_gate_mode_.data == GateMode::EXTERNAL) {
    publishControlCommands(remote_commands_);
  }
}

// for emergency
void VehicleCmdGate::onEmergencyCtrlCmd(AckermannControlCommand::ConstSharedPtr msg)
{
  emergency_commands_.control = *msg;

  if (use_emergency_handling_ && is_system_emergency_) {
    publishControlCommands(emergency_commands_);
  }
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

  if (moderate_stop_interface_->is_stop_requested()) {  // if stop requested, stop the vehicle
    filtered_commands.control.longitudinal.speed = 0.0;
    filtered_commands.control.longitudinal.acceleration = moderate_stop_service_acceleration_;
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

  // Check pause. Place this check after all other checks as it needs the final output.
  adapi_pause_->update(filtered_commands.control);
  if (adapi_pause_->is_paused()) {
    filtered_commands.control = createStopControlCmd();
  }

  // Check if command filtering option is enable
  if (enable_cmd_limit_filter_) {
    // Apply limit filtering
    filtered_commands.control = filterControlCommand(filtered_commands.control);
  }
  // tmp: Publish vehicle emergency status
  VehicleEmergencyStamped vehicle_cmd_emergency;
  vehicle_cmd_emergency.emergency = (use_emergency_handling_ && is_system_emergency_);
  vehicle_cmd_emergency.stamp = filtered_commands.control.stamp;

  // Publish commands
  vehicle_cmd_emergency_pub_->publish(vehicle_cmd_emergency);
  control_cmd_pub_->publish(filtered_commands.control);
  adapi_pause_->publish();
  moderate_stop_interface_->publish();

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
  adapi_pause_->update(control_cmd);

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
  adapi_pause_->publish();
  moderate_stop_interface_->publish();
}

AckermannControlCommand VehicleCmdGate::filterControlCommand(const AckermannControlCommand & in)
{
  AckermannControlCommand out = in;
  const double dt = getDt();
  const auto mode = current_operation_mode_;
  const auto current_status_cmd = getActualStatusAsCommand();
  const auto ego_is_stopped = vehicle_stop_checker_->isVehicleStopped(stop_check_duration_);
  const auto input_cmd_is_stopping = in.longitudinal.acceleration < 0.0;

  filter_.setCurrentSpeed(current_kinematics_.twist.twist.linear.x);
  filter_on_transition_.setCurrentSpeed(current_kinematics_.twist.twist.linear.x);

  IsFilterActivated is_filter_activated;

  // Apply transition_filter when transiting from MANUAL to AUTO.
  if (mode.is_in_transition) {
    filter_on_transition_.filterAll(dt, current_steer_, out, is_filter_activated);
  } else {
    // When ego is stopped and the input command is not stopping,
    // use the higher of actual vehicle longitudinal state
    // and previous longitudinal command for the filtering
    // this is to prevent the jerk limits being applied and causing
    // a delay when restarting the vehicle.

    if (ego_is_stopped && !input_cmd_is_stopping) {
      auto prev_cmd = filter_.getPrevCmd();
      prev_cmd.longitudinal.acceleration =
        std::max(prev_cmd.longitudinal.acceleration, current_status_cmd.longitudinal.acceleration);
      // consider reverse driving
      prev_cmd.longitudinal.speed =
        std::fabs(prev_cmd.longitudinal.speed) > std::fabs(current_status_cmd.longitudinal.speed)
          ? prev_cmd.longitudinal.speed
          : current_status_cmd.longitudinal.speed;
      filter_.setPrevCmd(prev_cmd);
    }
    filter_.filterAll(dt, current_steer_, out, is_filter_activated);
  }

  // set prev value for both to keep consistency over switching:
  // Actual steer, vel, acc should be considered in manual mode to prevent sudden motion when
  // switching from manual to autonomous
  auto prev_values = mode.is_autoware_control_enabled ? out : current_status_cmd;

  if (ego_is_stopped) {
    prev_values.longitudinal = out.longitudinal;
  }

  // TODO(Horibe): To prevent sudden acceleration/deceleration when switching from manual to
  // autonomous, the filter should be applied for actual speed and acceleration during manual
  // driving. However, this means that the output command from Gate will always be close to the
  // driving state during manual driving. Here, let autoware publish the stop command when the ego
  // is stopped to intend the autoware is trying to keep stopping.

  filter_.setPrevCmd(prev_values);
  filter_on_transition_.setPrevCmd(prev_values);

  is_filter_activated.stamp = now();
  is_filter_activated_pub_->publish(is_filter_activated);
  publishMarkers(is_filter_activated);

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

void VehicleCmdGate::onEngage(EngageMsg::ConstSharedPtr msg)
{
  is_engaged_ = msg->engage;
}

void VehicleCmdGate::onEngageService(
  const EngageSrv::Request::SharedPtr request, const EngageSrv::Response::SharedPtr response)
{
  is_engaged_ = request->engage;
  response->status = tier4_api_utils::response_success();
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

AckermannControlCommand VehicleCmdGate::getActualStatusAsCommand()
{
  AckermannControlCommand status;
  status.stamp = status.lateral.stamp = status.longitudinal.stamp = this->now();
  status.lateral.steering_tire_angle = current_steer_;
  status.lateral.steering_tire_rotation_rate = 0.0;
  status.longitudinal.speed = current_kinematics_.twist.twist.linear.x;
  status.longitudinal.acceleration = current_acceleration_;
  return status;
}

void VehicleCmdGate::onExternalEmergencyStopService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const SetEmergency::Request::SharedPtr request, const SetEmergency::Response::SharedPtr response)
{
  auto req = std::make_shared<Trigger::Request>();
  auto res = std::make_shared<Trigger::Response>();
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
  [[maybe_unused]] const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
{
  is_external_emergency_stop_ = true;
  res->success = true;
  res->message = "external_emergency_stop requested was accepted.";

  return true;
}

bool VehicleCmdGate::onClearExternalEmergencyStopService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> req_header,
  [[maybe_unused]] const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res)
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

MarkerArray VehicleCmdGate::createMarkerArray(const IsFilterActivated & filter_activated)
{
  MarkerArray msg;

  if (!filter_activated.is_activated) {
    return msg;
  }

  /* add string marker */
  bool first_msg = true;
  std::string reason = "filter activated on";

  if (filter_activated.is_activated_on_acceleration) {
    reason += " lon_acc";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_jerk) {
    reason += first_msg ? " jerk" : ", jerk";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_speed) {
    reason += first_msg ? " speed" : ", speed";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_steering) {
    reason += first_msg ? " steer" : ", steer";
    first_msg = false;
  }
  if (filter_activated.is_activated_on_steering_rate) {
    reason += first_msg ? " steer_rate" : ", steer_rate";
    first_msg = false;
  }

  msg.markers.emplace_back(createStringMarker(
    "base_link", "msg", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    createMarkerPosition(0.0, 0.0, 1.0), createMarkerScale(0.0, 0.0, 1.0),
    createMarkerColor(1.0, 0.0, 0.0, 1.0), reason));

  /* add sphere marker */
  msg.markers.emplace_back(createMarker(
    "base_link", "sphere", 0, visualization_msgs::msg::Marker::SPHERE,
    createMarkerPosition(0.0, 0.0, 0.0), createMarkerScale(3.0, 3.0, 3.0),
    createMarkerColor(1.0, 0.0, 0.0, 0.3)));

  return msg;
}

void VehicleCmdGate::publishMarkers(const IsFilterActivated & filter_activated)
{
  BoolStamped filter_activated_flag;
  if (filter_activated.is_activated) {
    filter_activated_count_++;
  } else {
    filter_activated_count_ = 0;
  }
  if (
    filter_activated_count_ >= filter_activated_count_threshold_ &&
    std::fabs(current_kinematics_.twist.twist.linear.x) >= filter_activated_velocity_threshold_ &&
    current_operation_mode_.mode == OperationModeState::AUTONOMOUS) {
    filter_activated_marker_pub_->publish(createMarkerArray(filter_activated));
    filter_activated_flag.data = true;
  } else {
    filter_activated_flag.data = false;
  }

  filter_activated_flag.stamp = now();
  filter_activated_flag_pub_->publish(filter_activated_flag);
  filter_activated_marker_raw_pub_->publish(createMarkerArray(filter_activated));
}
}  // namespace vehicle_cmd_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_cmd_gate::VehicleCmdGate)
