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

#include <algorithm>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#define FMT_HEADER_ONLY
#include "system_error_monitor/diagnostics_filter.hpp"
#include "system_error_monitor/system_error_monitor_core.hpp"

#include <fmt/format.h>

namespace
{
enum class DebugLevel { DEBUG, INFO, WARN, ERROR, FATAL };

template <DebugLevel debug_level>
void logThrottledNamed(
  const std::string & logger_name, const rclcpp::Clock::SharedPtr clock, const double duration_ms,
  const std::string & message)
{
  static std::unordered_map<std::string, rclcpp::Time> last_output_time;
  if (last_output_time.count(logger_name) != 0) {
    const auto time_from_last_output = clock->now() - last_output_time.at(logger_name);
    if (time_from_last_output.seconds() * 1000.0 < duration_ms) {
      return;
    }
  }

  last_output_time[logger_name] = clock->now();
  if constexpr (debug_level == DebugLevel::DEBUG) {
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name), message.c_str());
  } else if constexpr (debug_level == DebugLevel::INFO) {
    RCLCPP_INFO(rclcpp::get_logger(logger_name), message.c_str());
  } else if constexpr (debug_level == DebugLevel::WARN) {
    RCLCPP_WARN(rclcpp::get_logger(logger_name), message.c_str());
  } else if constexpr (debug_level == DebugLevel::ERROR) {
    RCLCPP_ERROR(rclcpp::get_logger(logger_name), message.c_str());
  } else if constexpr (debug_level == DebugLevel::FATAL) {
    RCLCPP_FATAL(rclcpp::get_logger(logger_name), message.c_str());
  }
}

std::vector<std::string> split(const std::string & str, const char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

int str2level(const std::string & level_str)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using std::regex_constants::icase;

  if (std::regex_match(level_str, std::regex("warn", icase))) {
    return DiagnosticStatus::WARN;
  }
  if (std::regex_match(level_str, std::regex("error", icase))) {
    return DiagnosticStatus::ERROR;
  }
  if (std::regex_match(level_str, std::regex("stale", icase))) {
    return DiagnosticStatus::STALE;
  }

  throw std::runtime_error(fmt::format("invalid level: {}", level_str));
}

bool isOverLevel(const int & diag_level, const std::string & failure_level_str)
{
  if (failure_level_str == "none") {
    return false;
  }

  return diag_level >= str2level(failure_level_str);
}

std::vector<diagnostic_msgs::msg::DiagnosticStatus> & getTargetDiagnosticsRef(
  const int hazard_level, autoware_auto_system_msgs::msg::HazardStatus * hazard_status)
{
  using autoware_auto_system_msgs::msg::HazardStatus;

  if (hazard_level == HazardStatus::NO_FAULT) {
    return hazard_status->diag_no_fault;
  }
  if (hazard_level == HazardStatus::SAFE_FAULT) {
    return hazard_status->diag_safe_fault;
  }
  if (hazard_level == HazardStatus::LATENT_FAULT) {
    return hazard_status->diag_latent_fault;
  }
  if (hazard_level == HazardStatus::SINGLE_POINT_FAULT) {
    return hazard_status->diag_single_point_fault;
  }

  throw std::runtime_error(fmt::format("invalid hazard level: {}", hazard_level));
}

diagnostic_msgs::msg::DiagnosticArray convertHazardStatusToDiagnosticArray(
  rclcpp::Clock::SharedPtr clock,
  const autoware_auto_system_msgs::msg::HazardStatus & hazard_status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = clock->now();

  const auto decorateDiag = [](const auto & hazard_diag, const std::string & label) {
    auto diag = hazard_diag;

    diag.message = label + diag.message;

    return diag;
  };

  for (const auto & hazard_diag : hazard_status.diag_no_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[No Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_safe_fault) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Safe Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_latent_fault) {
    const std::string logger_name = "system_error_monitor " + hazard_diag.name;
    logThrottledNamed<DebugLevel::WARN>(
      logger_name, clock, 5000, "[Latent Fault]: " + hazard_diag.message);

    diag_array.status.push_back(decorateDiag(hazard_diag, "[Latent Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diag_single_point_fault) {
    const std::string logger_name = "system_error_monitor " + hazard_diag.name;
    logThrottledNamed<DebugLevel::ERROR>(
      logger_name, clock, 5000, "[Single Point Fault]: " + hazard_diag.message);

    diag_array.status.push_back(decorateDiag(hazard_diag, "[Single Point Fault]"));
  }

  return diag_array;
}

std::set<std::string> getErrorModules(
  const autoware_auto_system_msgs::msg::HazardStatus & hazard_status,
  const int emergency_hazard_level)
{
  std::set<std::string> error_modules;
  using autoware_auto_system_msgs::msg::HazardStatus;
  if (emergency_hazard_level <= HazardStatus::SINGLE_POINT_FAULT) {
    for (const auto & diag_spf : hazard_status.diag_single_point_fault) {
      error_modules.insert(diag_spf.name);
    }
  }
  if (emergency_hazard_level <= HazardStatus::LATENT_FAULT) {
    for (const auto & diag_lf : hazard_status.diag_latent_fault) {
      error_modules.insert(diag_lf.name);
    }
  }
  if (emergency_hazard_level <= HazardStatus::SAFE_FAULT) {
    for (const auto & diag_sf : hazard_status.diag_safe_fault) {
      error_modules.insert(diag_sf.name);
    }
  }

  return error_modules;
}

autoware_auto_system_msgs::msg::HazardStatus createTimeoutHazardStatus()
{
  autoware_auto_system_msgs::msg::HazardStatus hazard_status;
  hazard_status.level = autoware_auto_system_msgs::msg::HazardStatus::SINGLE_POINT_FAULT;
  hazard_status.emergency = true;
  hazard_status.emergency_holding = false;
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = "system_error_monitor/input_data_timeout";
  diag.hardware_id = "system_error_monitor";
  diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  hazard_status.diag_single_point_fault.push_back(diag);
  return hazard_status;
}

int isInNoFaultCondition(
  const autoware_auto_system_msgs::msg::AutowareState & autoware_state,
  const tier4_control_msgs::msg::GateMode & current_gate_mode)
{
  using autoware_auto_system_msgs::msg::AutowareState;
  using tier4_control_msgs::msg::GateMode;

  const auto is_in_autonomous_ignore_state =
    (autoware_state.state == AutowareState::INITIALIZING) ||
    (autoware_state.state == AutowareState::WAITING_FOR_ROUTE) ||
    (autoware_state.state == AutowareState::PLANNING) ||
    (autoware_state.state == AutowareState::FINALIZING);

  if (current_gate_mode.data == GateMode::AUTO && is_in_autonomous_ignore_state) {
    return true;
  }

  const auto is_in_external_ignore_state = (autoware_state.state == AutowareState::INITIALIZING) ||
                                           (autoware_state.state == AutowareState::FINALIZING);

  if (current_gate_mode.data == GateMode::EXTERNAL && is_in_external_ignore_state) {
    return true;
  }

  return false;
}
}  // namespace

AutowareErrorMonitor::AutowareErrorMonitor()
: Node(
    "system_error_monitor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  // Parameter
  get_parameter_or<int>("update_rate", params_.update_rate, 10);
  get_parameter_or<bool>("ignore_missing_diagnostics", params_.ignore_missing_diagnostics, false);
  get_parameter_or<bool>("add_leaf_diagnostics", params_.add_leaf_diagnostics, true);
  get_parameter_or<double>("data_ready_timeout", params_.data_ready_timeout, 30.0);
  get_parameter_or<double>("data_heartbeat_timeout", params_.data_heartbeat_timeout, 1.0);
  get_parameter_or<double>("diag_timeout_sec", params_.diag_timeout_sec, 1.0);
  get_parameter_or<double>("hazard_recovery_timeout", params_.hazard_recovery_timeout, 5.0);
  get_parameter_or<int>(
    "emergency_hazard_level", params_.emergency_hazard_level,
    autoware_auto_system_msgs::msg::HazardStatus::LATENT_FAULT);
  get_parameter_or<bool>("use_emergency_hold", params_.use_emergency_hold, false);
  get_parameter_or<bool>(
    "use_emergency_hold_in_manual_driving", params_.use_emergency_hold_in_manual_driving, false);

  loadRequiredModules(KeyName::autonomous_driving);
  loadRequiredModules(KeyName::external_control);

  using std::placeholders::_1;
  using std::placeholders::_2;
  // Subscriber
  sub_diag_array_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "input/diag_array", rclcpp::QoS{1}, std::bind(&AutowareErrorMonitor::onDiagArray, this, _1));
  sub_current_gate_mode_ = create_subscription<tier4_control_msgs::msg::GateMode>(
    "~/input/current_gate_mode", rclcpp::QoS{1},
    std::bind(&AutowareErrorMonitor::onCurrentGateMode, this, _1));
  sub_autoware_state_ = create_subscription<autoware_auto_system_msgs::msg::AutowareState>(
    "~/input/autoware_state", rclcpp::QoS{1},
    std::bind(&AutowareErrorMonitor::onAutowareState, this, _1));
  sub_control_mode_ = create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "~/input/control_mode", rclcpp::QoS{1},
    std::bind(&AutowareErrorMonitor::onControlMode, this, _1));

  // Publisher
  pub_hazard_status_ = create_publisher<autoware_auto_system_msgs::msg::HazardStatusStamped>(
    "~/output/hazard_status", rclcpp::QoS{1});
  pub_diagnostics_err_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "~/output/diagnostics_err", rclcpp::QoS{1});

  // Service
  srv_clear_emergency_ = this->create_service<std_srvs::srv::Trigger>(
    "service/clear_emergency",
    std::bind(&AutowareErrorMonitor::onClearEmergencyService, this, _1, _2));

  // Initialize
  autoware_auto_vehicle_msgs::msg::ControlModeReport vehicle_state_report;
  vehicle_state_report.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
  control_mode_ = std::make_shared<const autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    vehicle_state_report);

  // Timer
  initialized_time_ = this->now();
  const auto period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&AutowareErrorMonitor::onTimer, this));
}

void AutowareErrorMonitor::loadRequiredModules(const std::string & key)
{
  const auto param_key = std::string("required_modules.") + key;

  const uint64_t depth = 3;
  const auto param_names = this->list_parameters({param_key}, depth).names;

  if (param_names.empty()) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  // Load module names from parameter key
  std::set<std::string> module_names;
  RequiredModules required_modules;

  for (const auto & param_name : param_names) {
    // Example of param_name: required_modules.key.module
    //                     or required_modules.key.module.parameter
    const auto split_names = split(param_name, '.');
    const auto & param_required_modules = split_names.at(0);
    const auto & param_key = split_names.at(1);
    const auto & param_module = split_names.at(2);
    const auto module_name_with_prefix =
      fmt::format("{0}.{1}.{2}", param_required_modules, param_key, param_module);

    // Skip duplicate parameter
    if (module_names.count(module_name_with_prefix) != 0) {
      continue;
    }
    module_names.insert(module_name_with_prefix);

    // Load diag level
    const auto sf_key = module_name_with_prefix + std::string(".sf_at");
    std::string sf_at;
    this->get_parameter_or(sf_key, sf_at, std::string("none"));

    const auto lf_key = module_name_with_prefix + std::string(".lf_at");
    std::string lf_at;
    this->get_parameter_or(lf_key, lf_at, std::string("warn"));

    const auto spf_key = module_name_with_prefix + std::string(".spf_at");
    std::string spf_at;
    this->get_parameter_or(spf_key, spf_at, std::string("error"));

    // auto_recovery
    const auto auto_recovery_key = module_name_with_prefix + std::string(".auto_recovery");
    std::string auto_recovery_approval_str;
    this->get_parameter_or(auto_recovery_key, auto_recovery_approval_str, std::string("true"));

    // Convert auto_recovery_approval_str to bool
    bool auto_recovery_approval{};
    std::istringstream(auto_recovery_approval_str) >> std::boolalpha >> auto_recovery_approval;

    required_modules.push_back({param_module, sf_at, lf_at, spf_at, auto_recovery_approval});
  }

  required_modules_map_.insert(std::make_pair(key, required_modules));
}

void AutowareErrorMonitor::onDiagArray(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  diag_array_ = msg;

  const auto & header = msg->header;

  for (const auto & diag : msg->status) {
    if (diag_buffer_map_.count(diag.name) == 0) {
      diag_buffer_map_.insert(std::make_pair(diag.name, DiagBuffer{}));
    }

    auto & diag_buffer = diag_buffer_map_.at(diag.name);
    diag_buffer.push_back(DiagStamped{header, diag});

    while (diag_buffer.size() > diag_buffer_size_) {
      diag_buffer.pop_front();
    }
  }

  // for Heartbeat
  diag_array_stamp_ = this->now();
}

void AutowareErrorMonitor::onCurrentGateMode(
  const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  current_gate_mode_ = msg;

  // for Heartbeat
  current_gate_mode_stamp_ = this->now();
}

void AutowareErrorMonitor::onAutowareState(
  const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  autoware_state_ = msg;

  // for Heartbeat
  autoware_state_stamp_ = this->now();
}

void AutowareErrorMonitor::onControlMode(
  const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg)
{
  control_mode_ = msg;

  // for Heartbeat
  control_mode_stamp_ = this->now();
}

bool AutowareErrorMonitor::isDataReady()
{
  if (!diag_array_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for diag_array msg...");
    return false;
  }

  if (!current_gate_mode_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_gate_mode msg...");
    return false;
  }

  if (!autoware_state_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for autoware_state msg...");
    return false;
  }

  if (!control_mode_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for vehicle_state_report msg...");
    return false;
  }
  return true;
}

bool AutowareErrorMonitor::isDataHeartbeatTimeout()
{
  auto isTimeout = [this](const rclcpp::Time & last_stamp, const double threshold) {
    const auto time_diff = this->now() - last_stamp;
    return time_diff.seconds() > threshold;
  };

  if (isTimeout(diag_array_stamp_, params_.data_heartbeat_timeout)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "diag_array msg is timeout...");
    return true;
  }

  if (isTimeout(current_gate_mode_stamp_, params_.data_heartbeat_timeout)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "current_gate_mode msg is timeout...");
    return true;
  }

  if (isTimeout(autoware_state_stamp_, params_.data_heartbeat_timeout)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "autoware_state msg is timeout...");
    return true;
  }

  if (isTimeout(control_mode_stamp_, params_.data_heartbeat_timeout)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "vehicle_state_report msg is timeout...");
    return true;
  }

  return false;
}

void AutowareErrorMonitor::onTimer()
{
  if (!isDataReady()) {
    if ((this->now() - initialized_time_).seconds() > params_.data_ready_timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "input data is timeout");
      publishHazardStatus(createTimeoutHazardStatus());
    }
    return;
  }

  if (isDataHeartbeatTimeout()) {
    publishHazardStatus(createTimeoutHazardStatus());
    return;
  }

  current_mode_ = current_gate_mode_->data == tier4_control_msgs::msg::GateMode::AUTO
                    ? KeyName::autonomous_driving
                    : KeyName::external_control;

  updateHazardStatus();
  publishHazardStatus(hazard_status_);
}

boost::optional<DiagStamped> AutowareErrorMonitor::getLatestDiag(
  const std::string & diag_name) const
{
  if (diag_buffer_map_.count(diag_name) == 0) {
    return {};
  }

  const auto & diag_buffer = diag_buffer_map_.at(diag_name);

  if (diag_buffer.empty()) {
    return {};
  }

  return diag_buffer.back();
}

uint8_t AutowareErrorMonitor::getHazardLevel(
  const DiagConfig & required_module, const int diag_level) const
{
  using autoware_auto_system_msgs::msg::HazardStatus;

  if (isOverLevel(diag_level, required_module.spf_at)) {
    return HazardStatus::SINGLE_POINT_FAULT;
  }
  if (isOverLevel(diag_level, required_module.lf_at)) {
    return HazardStatus::LATENT_FAULT;
  }
  if (isOverLevel(diag_level, required_module.sf_at)) {
    return HazardStatus::SAFE_FAULT;
  }

  return HazardStatus::NO_FAULT;
}

void AutowareErrorMonitor::appendHazardDiag(
  const DiagConfig & required_module, const diagnostic_msgs::msg::DiagnosticStatus & hazard_diag,
  autoware_auto_system_msgs::msg::HazardStatus * hazard_status) const
{
  const auto hazard_level = getHazardLevel(required_module, hazard_diag.level);

  auto & target_diagnostics_ref = getTargetDiagnosticsRef(hazard_level, hazard_status);
  target_diagnostics_ref.push_back(hazard_diag);

  if (params_.add_leaf_diagnostics) {
    for (const auto & diag :
         diagnostics_filter::extractLeafChildrenDiagnostics(hazard_diag, diag_array_->status)) {
      target_diagnostics_ref.push_back(diag);
    }
  }

  hazard_status->level = std::max(hazard_status->level, hazard_level);
}

autoware_auto_system_msgs::msg::HazardStatus AutowareErrorMonitor::judgeHazardStatus() const
{
  using autoware_auto_system_msgs::msg::HazardStatus;
  using diagnostic_msgs::msg::DiagnosticStatus;

  autoware_auto_system_msgs::msg::HazardStatus hazard_status;
  for (const auto & required_module : required_modules_map_.at(current_mode_)) {
    const auto & diag_name = required_module.name;
    const auto latest_diag = getLatestDiag(diag_name);

    // no diag found
    if (!latest_diag) {
      if (!params_.ignore_missing_diagnostics) {
        DiagnosticStatus missing_diag;

        missing_diag.name = diag_name;
        missing_diag.hardware_id = "system_error_monitor";
        missing_diag.level = DiagnosticStatus::STALE;
        missing_diag.message = "no diag found";

        appendHazardDiag(required_module, missing_diag, &hazard_status);
      }

      continue;
    }

    // diag level high
    {
      appendHazardDiag(required_module, latest_diag->status, &hazard_status);
    }

    // diag timeout
    {
      const auto time_diff = this->now() - latest_diag->header.stamp;
      if (time_diff.seconds() > params_.diag_timeout_sec) {
        DiagnosticStatus timeout_diag = latest_diag->status;
        timeout_diag.level = DiagnosticStatus::STALE;
        timeout_diag.message = "timeout";

        appendHazardDiag(required_module, timeout_diag, &hazard_status);
      }
    }
  }

  // Ignore error when vehicle is not ready to start
  if (isInNoFaultCondition(*autoware_state_, *current_gate_mode_)) {
    hazard_status.level = autoware_auto_system_msgs::msg::HazardStatus::NO_FAULT;
  }

  return hazard_status;
}

void AutowareErrorMonitor::updateHazardStatus()
{
  const bool prev_emergency_status = hazard_status_.emergency;

  // Create hazard status based on diagnostics
  if (!hazard_status_.emergency_holding) {
    const auto current_hazard_status = judgeHazardStatus();
    hazard_status_.level = current_hazard_status.level;
    hazard_status_.diag_no_fault = current_hazard_status.diag_no_fault;
    hazard_status_.diag_safe_fault = current_hazard_status.diag_safe_fault;
    hazard_status_.diag_latent_fault = current_hazard_status.diag_latent_fault;
    hazard_status_.diag_single_point_fault = current_hazard_status.diag_single_point_fault;
  }

  // Update emergency status
  {
    hazard_status_.emergency = hazard_status_.level >= params_.emergency_hazard_level;
    if (hazard_status_.emergency != prev_emergency_status) {
      emergency_state_switch_time_ = this->now();
    }
  }

  // Update emergency_holding condition
  if (params_.use_emergency_hold) {
    hazard_status_.emergency_holding = isEmergencyHoldingRequired();
  }
}

bool AutowareErrorMonitor::canAutoRecovery() const
{
  const auto error_modules = getErrorModules(hazard_status_, params_.emergency_hazard_level);
  for (const auto & required_module : required_modules_map_.at(current_mode_)) {
    if (required_module.auto_recovery) {
      continue;
    }
    if (error_modules.count(required_module.name) != 0) {
      return false;
    }
  }
  return true;
}

bool AutowareErrorMonitor::isEmergencyHoldingRequired() const
{
  // Does not change holding status until emergency_holding is cleared by service call
  if (hazard_status_.emergency_holding) {
    return true;
  }

  if (!hazard_status_.emergency) {
    return false;
  }

  // Don't hold status if emergency duration within recovery timeout
  const auto emergency_duration = (this->now() - emergency_state_switch_time_).seconds();
  const auto within_recovery_timeout = emergency_duration < params_.hazard_recovery_timeout;
  if (within_recovery_timeout && canAutoRecovery()) {
    return false;
  }

  // Don't hold status during manual driving
  const bool is_manual_driving =
    (control_mode_->mode == autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL);
  const auto no_hold_condition =
    (!params_.use_emergency_hold_in_manual_driving && is_manual_driving);
  if (no_hold_condition) {
    return false;
  }

  return true;
}

void AutowareErrorMonitor::publishHazardStatus(
  const autoware_auto_system_msgs::msg::HazardStatus & hazard_status)
{
  autoware_auto_system_msgs::msg::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.stamp = this->now();
  hazard_status_stamped.status = hazard_status;
  pub_hazard_status_->publish(hazard_status_stamped);
  pub_diagnostics_err_->publish(
    convertHazardStatusToDiagnosticArray(this->get_clock(), hazard_status_stamped.status));
}

bool AutowareErrorMonitor::onClearEmergencyService(
  [[maybe_unused]] std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  hazard_status_.emergency_holding = false;
  updateHazardStatus();
  response->success = true;
  response->message = "Emergency Holding state was cleared.";

  return true;
}
