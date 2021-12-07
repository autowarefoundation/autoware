// Copyright 2017-2019 Autoware Foundation
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

#include <pacmod_interface/pacmod_interface.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

PacmodInterface::PacmodInterface()
: Node("pacmod_interface"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);

  /* parameters for vehicle specifications */
  tire_radius_ = vehicle_info_.wheel_radius_m;
  wheel_base_ = vehicle_info_.wheel_base_m;

  steering_offset_ = declare_parameter("steering_offset", 0.0);
  enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

  /* parameters for emergency stop */
  emergency_brake_ = declare_parameter("emergency_brake", 0.7);

  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);
  accel_pedal_offset_ = declare_parameter("accel_pedal_offset", 0.0);
  brake_pedal_offset_ = declare_parameter("brake_pedal_offset", 0.0);

  /* parameters for limitter */
  max_throttle_ = declare_parameter("max_throttle", 0.2);
  max_brake_ = declare_parameter("max_brake", 0.8);
  max_steering_wheel_ = declare_parameter("max_steering_wheel", 2.7 * M_PI);
  max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 6.6);
  min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5);
  steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0);
  steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0);
  low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh

  /* parameters for turn signal recovery */
  hazard_thresh_time_ = declare_parameter("hazard_thresh_time", 0.20);  // s
  /* initialize */
  prev_steer_cmd_.header.stamp = this->now();
  prev_steer_cmd_.command = 0.0;

  /* subscribers */
  using std::placeholders::_1;

  // From autoware
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&PacmodInterface::callbackControlCmd, this, _1));
  gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&PacmodInterface::callbackGearCmd, this, _1));
  turn_indicators_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
      std::bind(&PacmodInterface::callbackTurnIndicatorsCommand, this, _1));
  hazard_lights_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
      std::bind(&PacmodInterface::callbackHazardLightsCommand, this, _1));
  engage_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "/vehicle/engage", rclcpp::QoS{1}, std::bind(&PacmodInterface::callbackEngage, this, _1));
  actuation_cmd_sub_ = create_subscription<ActuationCommandStamped>(
    "/control/command/actuation_cmd", 1,
    std::bind(&PacmodInterface::callbackActuationCmd, this, _1));
  emergency_sub_ = create_subscription<autoware_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&PacmodInterface::callbackEmergencyCmd, this, _1));

  // From pacmod

  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
      this, "/pacmod/steering_rpt");
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>(
      this, "/pacmod/wheel_speed_rpt");
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/accel_rpt");
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/brake_rpt");
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/shift_rpt");
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/turn_rpt");
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>>(
    this, "/pacmod/global_rpt");

  pacmod_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>>(
      PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
      *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_);

  pacmod_feedbacks_sync_->registerCallback(std::bind(
    &PacmodInterface::callbackPacmodRpt, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7));

  /* publisher */
  // To pacmod
  accel_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/accel_cmd", rclcpp::QoS{1});
  brake_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/brake_cmd", rclcpp::QoS{1});
  steer_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SteeringCmd>("/pacmod/steering_cmd", rclcpp::QoS{1});
  shift_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/shift_cmd", rclcpp::QoS{1});
  turn_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/turn_cmd", rclcpp::QoS{1});
  raw_steer_cmd_pub_ = create_publisher<pacmod3_msgs::msg::SteeringCmd>(
    "/pacmod/raw_steer_cmd", rclcpp::QoS{1});  // only for debug

  // To Autoware
  control_mode_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  turn_indicators_status_pub_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  hazard_lights_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  actuation_status_pub_ =
    create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1);
  steering_wheel_status_pub_ =
    create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);

  // Timer
  auto timer_callback = std::bind(&PacmodInterface::publishCommands, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / loop_rate_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void PacmodInterface::callbackActuationCmd(const ActuationCommandStamped::ConstSharedPtr msg)
{
  actuation_command_received_time_ = this->now();
  actuation_cmd_ptr_ = msg;
}

void PacmodInterface::callbackEmergencyCmd(
  const autoware_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

void PacmodInterface::callbackControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

void PacmodInterface::callbackGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

void PacmodInterface::callbackTurnIndicatorsCommand(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

void PacmodInterface::callbackHazardLightsCommand(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
}

void PacmodInterface::callbackEngage(
  const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  engage_cmd_ = msg->engage;
  is_clear_override_needed_ = true;
}

void PacmodInterface::callbackPacmodRpt(
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
  const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt)
{
  is_pacmod_rpt_received_ = true;
  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  gear_cmd_rpt_ptr_ = shift_rpt;
  global_rpt_ptr_ = global_rpt;
  turn_rpt_ptr_ = turn_rpt;

  is_pacmod_enabled_ =
    steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled;
  RCLCPP_DEBUG(
    get_logger(),
    "enabled: is_pacmod_enabled_ %d, steer %d, accel %d, brake %d, shift %d, "
    "global %d",
    is_pacmod_enabled_, steer_wheel_rpt_ptr_->enabled, accel_rpt_ptr_->enabled,
    brake_rpt_ptr_->enabled, gear_cmd_rpt_ptr_->enabled, global_rpt_ptr_->enabled);

  const double current_velocity = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *gear_cmd_rpt_ptr_);  // current vehicle speed > 0 [m/s]
  const double current_steer_wheel =
    steer_wheel_rpt_ptr_->output;  // current vehicle steering wheel angle [rad]
  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio - steering_offset_;

  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish steering wheel status */
  {
    SteeringWheelStatusStamped steering_wheel_status_msg;
    steering_wheel_status_msg.stamp = header.stamp;
    steering_wheel_status_msg.data = current_steer_wheel;
    steering_wheel_status_pub_->publish(steering_wheel_status_msg);
  }

  /* publish vehicle status control_mode */
  {
    autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = header.stamp;

    if (global_rpt->enabled && is_pacmod_enabled_) {
      control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    } else {
      control_mode_msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
    }

    control_mode_pub_->publish(control_mode_msg);
  }

  /* publish vehicle status twist */
  {
    autoware_auto_vehicle_msgs::msg::VelocityReport twist;
    twist.header = header;
    twist.longitudinal_velocity = current_velocity;                                 // [m/s]
    twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    vehicle_twist_pub_->publish(twist);
  }

  /* publish current shift */
  {
    autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
    gear_report_msg.stamp = header.stamp;
    const auto opt_gear_report = toAutowareShiftReport(*gear_cmd_rpt_ptr_);
    if (opt_gear_report) {
      gear_report_msg.report = *opt_gear_report;
      gear_status_pub_->publish(gear_report_msg);
    }
  }

  /* publish current status */
  {
    autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = header.stamp;
    steer_msg.steering_tire_angle = current_steer;
    steering_status_pub_->publish(steer_msg);
  }

  /* publish control status */
  {
    ActuationStatusStamped actuation_status;
    actuation_status.header = header;
    actuation_status.status.accel_status = accel_rpt_ptr_->output;
    actuation_status.status.brake_status = brake_rpt_ptr_->output;
    actuation_status.status.steer_status = current_steer;
    actuation_status_pub_->publish(actuation_status);
  }

  /* publish current turn signal */
  {
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
    turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(*turn_rpt);
    turn_indicators_status_pub_->publish(turn_msg);

    autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(*turn_rpt);
    hazard_lights_status_pub_->publish(hazard_msg);
  }
}

void PacmodInterface::publishCommands()
{
  /* guard */
  if (!actuation_cmd_ptr_ || !control_cmd_ptr_ || !is_pacmod_rpt_received_ || !gear_cmd_ptr_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "vehicle_cmd = %d, pacmod3_msgs = %d", actuation_cmd_ptr_ != nullptr,
      is_pacmod_rpt_received_);
    return;
  }

  const rclcpp::Time current_time = get_clock()->now();

  double desired_throttle = actuation_cmd_ptr_->actuation.accel_cmd + accel_pedal_offset_;
  double desired_brake = actuation_cmd_ptr_->actuation.brake_cmd + brake_pedal_offset_;
  if (actuation_cmd_ptr_->actuation.brake_cmd <= std::numeric_limits<double>::epsilon()) {
    desired_brake = 0.0;
  }

  /* check emergency and timeout */
  const double control_cmd_delta_time_ms =
    (current_time - control_command_received_time_).seconds() * 1000.0;
  const double actuation_cmd_delta_time_ms =
    (current_time - actuation_command_received_time_).seconds() * 1000.0;
  bool timeouted = false;
  const int t_out = command_timeout_ms_;
  if (t_out >= 0 && (control_cmd_delta_time_ms > t_out || actuation_cmd_delta_time_ms > t_out)) {
    timeouted = true;
  }
  if (is_emergency_ || timeouted) {
    RCLCPP_ERROR(
      get_logger(), "Emergency Stopping, emergency = %d, timeouted = %d", is_emergency_, timeouted);
    desired_throttle = 0.0;
    desired_brake = emergency_brake_;
  }

  const double current_velocity =
    calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *gear_cmd_rpt_ptr_);
  const double current_steer_wheel = steer_wheel_rpt_ptr_->output;

  /* calculate desired steering wheel */
  double adaptive_gear_ratio = calculateVariableGearRatio(current_velocity, current_steer_wheel);
  double desired_steer_wheel =
    (control_cmd_ptr_->lateral.steering_tire_angle + steering_offset_) * adaptive_gear_ratio;
  desired_steer_wheel =
    std::min(std::max(desired_steer_wheel, -max_steering_wheel_), max_steering_wheel_);

  /* check clear flag */
  bool clear_override = false;
  if (is_pacmod_enabled_ == true) {
    is_clear_override_needed_ = false;
  } else if (is_clear_override_needed_ == true) {
    clear_override = true;
  }

  /* make engage cmd false when a driver overrides vehicle control */
  if (!prev_override_ && global_rpt_ptr_->override_active) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pacmod is overridden, enable flag is back to false");
    engage_cmd_ = false;
  }
  prev_override_ = global_rpt_ptr_->override_active;

  /* make engage cmd false when vehicle report is timed out, e.g. E-stop is depressed */
  const bool report_timed_out = ((current_time - global_rpt_ptr_->header.stamp).seconds() > 1.0);
  if (report_timed_out) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pacmod report is timed out, enable flag is back to false");
    engage_cmd_ = false;
  }

  /* make engage cmd false when vehicle fault is active */
  if (global_rpt_ptr_->pacmod_sys_fault_active) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pacmod fault is active, enable flag is back to false");
    engage_cmd_ = false;
  }
  RCLCPP_DEBUG(
    get_logger(),
    "is_pacmod_enabled_ = %d, is_clear_override_needed_ = %d, clear_override = "
    "%d",
    is_pacmod_enabled_, is_clear_override_needed_, clear_override);

  /* check shift change */
  const double brake_for_shift_trans = 0.7;
  uint16_t desired_shift = gear_cmd_rpt_ptr_->output;
  if (std::fabs(current_velocity) < 0.1) {  // velocity is low -> the shift can be changed
    if (toPacmodShiftCmd(*gear_cmd_ptr_) != gear_cmd_rpt_ptr_->output) {  // need shift
                                                                          // change.
      desired_throttle = 0.0;
      desired_brake = brake_for_shift_trans;  // set brake to change the shift
      desired_shift = toPacmodShiftCmd(*gear_cmd_ptr_);
      RCLCPP_DEBUG(
        get_logger(), "Doing shift change. current = %d, desired = %d. set brake_cmd to %f",
        gear_cmd_rpt_ptr_->output, toPacmodShiftCmd(*gear_cmd_ptr_), desired_brake);
    }
  }

  /* publish accel cmd */
  {
    pacmod3_msgs::msg::SystemCmdFloat accel_cmd;
    accel_cmd.header.frame_id = base_frame_id_;
    accel_cmd.header.stamp = current_time;
    accel_cmd.enable = engage_cmd_;
    accel_cmd.ignore_overrides = false;
    accel_cmd.clear_override = clear_override;
    accel_cmd.command = std::max(0.0, std::min(desired_throttle, max_throttle_));
    accel_cmd_pub_->publish(accel_cmd);
  }

  /* publish brake cmd */
  {
    pacmod3_msgs::msg::SystemCmdFloat brake_cmd;
    brake_cmd.header.frame_id = base_frame_id_;
    brake_cmd.header.stamp = current_time;
    brake_cmd.enable = engage_cmd_;
    brake_cmd.ignore_overrides = false;
    brake_cmd.clear_override = clear_override;
    brake_cmd.command = std::max(0.0, std::min(desired_brake, max_brake_));
    brake_cmd_pub_->publish(brake_cmd);
  }

  /* publish steering cmd */
  {
    pacmod3_msgs::msg::SteeringCmd steer_cmd;
    steer_cmd.header.frame_id = base_frame_id_;
    steer_cmd.header.stamp = current_time;
    steer_cmd.enable = engage_cmd_;
    steer_cmd.ignore_overrides = false;
    steer_cmd.clear_override = clear_override;
    steer_cmd.rotation_rate = calcSteerWheelRateCmd(adaptive_gear_ratio);
    steer_cmd.command = steerWheelRateLimiter(
      desired_steer_wheel, prev_steer_cmd_.command, current_time, prev_steer_cmd_.header.stamp,
      steer_cmd.rotation_rate, current_steer_wheel, engage_cmd_);
    steer_cmd_pub_->publish(steer_cmd);
    prev_steer_cmd_ = steer_cmd;
  }

  /* publish raw steering cmd for debug */
  {
    pacmod3_msgs::msg::SteeringCmd raw_steer_cmd;
    raw_steer_cmd.header.frame_id = base_frame_id_;
    raw_steer_cmd.header.stamp = current_time;
    raw_steer_cmd.enable = engage_cmd_;
    raw_steer_cmd.ignore_overrides = false;
    raw_steer_cmd.clear_override = clear_override;
    raw_steer_cmd.command = desired_steer_wheel;
    raw_steer_cmd.rotation_rate =
      control_cmd_ptr_->lateral.steering_tire_rotation_rate * adaptive_gear_ratio;
    raw_steer_cmd_pub_->publish(raw_steer_cmd);
  }

  /* publish shift cmd */
  {
    pacmod3_msgs::msg::SystemCmdInt shift_cmd;
    shift_cmd.header.frame_id = base_frame_id_;
    shift_cmd.header.stamp = current_time;
    shift_cmd.enable = engage_cmd_;
    shift_cmd.ignore_overrides = false;
    shift_cmd.clear_override = clear_override;
    shift_cmd.command = desired_shift;
    shift_cmd_pub_->publish(shift_cmd);
  }

  if (turn_indicators_cmd_ptr_ && hazard_lights_cmd_ptr_) {
    /* publish shift cmd */
    pacmod3_msgs::msg::SystemCmdInt turn_cmd;
    turn_cmd.header.frame_id = base_frame_id_;
    turn_cmd.header.stamp = current_time;
    turn_cmd.enable = engage_cmd_;
    turn_cmd.ignore_overrides = false;
    turn_cmd.clear_override = clear_override;
    turn_cmd.command =
      toPacmodTurnCmdWithHazardRecover(*turn_indicators_cmd_ptr_, *hazard_lights_cmd_ptr_);
    turn_cmd_pub_->publish(turn_cmd);
  }
}

double PacmodInterface::calcSteerWheelRateCmd(const double gear_ratio)
{
  const auto current_vel =
    std::fabs(calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *gear_cmd_rpt_ptr_));

  // send low steer rate at low speed
  if (current_vel < std::numeric_limits<double>::epsilon()) {
    return steering_wheel_rate_stopped_;
  } else if (current_vel < low_vel_thresh_) {
    return steering_wheel_rate_low_vel_;
  }

  if (!enable_steering_rate_control_) {
    return max_steering_wheel_rate_;
  }

  constexpr double margin = 1.5;
  const double rate = margin * control_cmd_ptr_->lateral.steering_tire_rotation_rate * gear_ratio;
  return std::min(std::max(std::fabs(rate), min_steering_wheel_rate_), max_steering_wheel_rate_);
}

double PacmodInterface::calculateVehicleVelocity(
  const pacmod3_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptInt & shift_rpt)
{
  const double sign = (shift_rpt.output == pacmod3_msgs::msg::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
  const double vel =
    (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) * 0.5 *
    tire_radius_;
  return sign * vel;
}

double PacmodInterface::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

uint16_t PacmodInterface::toPacmodShiftCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand & gear_cmd)
{
  using pacmod3_msgs::msg::SystemCmdInt;

  if (gear_cmd.command == autoware_auto_vehicle_msgs::msg::GearCommand::PARK) {
    return SystemCmdInt::SHIFT_PARK;
  }
  if (gear_cmd.command == autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE) {
    return SystemCmdInt::SHIFT_REVERSE;
  }
  if (gear_cmd.command == autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE) {
    return SystemCmdInt::SHIFT_FORWARD;
  }
  if (gear_cmd.command == autoware_auto_vehicle_msgs::msg::GearCommand::LOW) {
    return SystemCmdInt::SHIFT_LOW;
  }

  return SystemCmdInt::SHIFT_NONE;
}

std::optional<int32_t> PacmodInterface::toAutowareShiftReport(
  const pacmod3_msgs::msg::SystemRptInt & shift)
{
  using autoware_auto_vehicle_msgs::msg::GearReport;
  using pacmod3_msgs::msg::SystemRptInt;

  if (shift.output == SystemRptInt::SHIFT_PARK) {
    return GearReport::PARK;
  }
  if (shift.output == SystemRptInt::SHIFT_REVERSE) {
    return GearReport::REVERSE;
  }
  if (shift.output == SystemRptInt::SHIFT_FORWARD) {
    return GearReport::DRIVE;
  }
  if (shift.output == SystemRptInt::SHIFT_LOW) {
    return GearReport::LOW;
  }
  return {};
}

uint16_t PacmodInterface::toPacmodTurnCmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand & hazard)
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
  using pacmod3_msgs::msg::SystemCmdInt;

  // NOTE: hazard lights command has a highest priority here.
  if (hazard.command == HazardLightsCommand::ENABLE) {
    return SystemCmdInt::TURN_HAZARDS;
  }
  if (turn.command == TurnIndicatorsCommand::ENABLE_LEFT) {
    return SystemCmdInt::TURN_LEFT;
  }
  if (turn.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    return SystemCmdInt::TURN_RIGHT;
  }
  return SystemCmdInt::TURN_NONE;
}

uint16_t PacmodInterface::toPacmodTurnCmdWithHazardRecover(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand & turn,
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand & hazard)
{
  using pacmod3_msgs::msg::SystemRptInt;

  if (!engage_cmd_ || turn_rpt_ptr_->command == turn_rpt_ptr_->output) {
    last_shift_inout_matched_time_ = this->now();
    return toPacmodTurnCmd(turn, hazard);
  }

  if ((this->now() - last_shift_inout_matched_time_).seconds() < hazard_thresh_time_) {
    return toPacmodTurnCmd(turn, hazard);
  }

  // hazard recover mode
  if (hazard_recover_count_ > hazard_recover_cmd_num_) {
    last_shift_inout_matched_time_ = this->now();
    hazard_recover_count_ = 0;
  }
  hazard_recover_count_++;

  if (
    turn_rpt_ptr_->command != SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output == SystemRptInt::TURN_HAZARDS) {
    // publish hazard commands for turning off the hazard lights
    return SystemRptInt::TURN_HAZARDS;
  } else if (  // NOLINT
    turn_rpt_ptr_->command == SystemRptInt::TURN_HAZARDS &&
    turn_rpt_ptr_->output != SystemRptInt::TURN_HAZARDS) {
    // publish none commands for turning on the hazard lights
    return SystemRptInt::TURN_NONE;
  } else {
    // something wrong
    RCLCPP_ERROR_STREAM(
      get_logger(), "turn signal command and output do not match. "
                      << "COMMAND: " << turn_rpt_ptr_->command
                      << "; OUTPUT: " << turn_rpt_ptr_->output);
    return toPacmodTurnCmd(turn, hazard);
  }
}

int32_t PacmodInterface::toAutowareTurnIndicatorsReport(
  const pacmod3_msgs::msg::SystemRptInt & turn)
{
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  using pacmod3_msgs::msg::SystemRptInt;

  if (turn.output == SystemRptInt::TURN_RIGHT) {
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (turn.output == SystemRptInt::TURN_LEFT) {
    return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (turn.output == SystemRptInt::TURN_NONE) {
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}

int32_t PacmodInterface::toAutowareHazardLightsReport(
  const pacmod3_msgs::msg::SystemRptInt & hazard)
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
  using pacmod3_msgs::msg::SystemRptInt;

  if (hazard.output == SystemRptInt::TURN_HAZARDS) {
    return HazardLightsReport::ENABLE;
  }

  return HazardLightsReport::DISABLE;
}

double PacmodInterface::steerWheelRateLimiter(
  const double current_steer_cmd, const double prev_steer_cmd,
  const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
  const double steer_rate, const double current_steer_output, const bool engage)
{
  if (!engage) {
    // return current steer as steer command ( do not apply steer rate filter )
    return current_steer_output;
  }

  const double dsteer = current_steer_cmd - prev_steer_cmd;
  const double dt = std::max(0.0, (current_steer_time - prev_steer_time).seconds());
  const double max_dsteer = std::fabs(steer_rate) * dt;
  const double limited_steer_cmd =
    prev_steer_cmd + std::min(std::max(-max_dsteer, dsteer), max_dsteer);
  return limited_steer_cmd;
}
