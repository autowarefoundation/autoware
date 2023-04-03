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

#include "dummy_diag_publisher/dummy_diag_publisher_core.hpp"

#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>

#define FMT_HEADER_ONLY
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/create_timer.hpp>
#include <tier4_autoware_utils/ros/update_param.hpp>

#include <fmt/format.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
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
}  // namespace

rcl_interfaces::msg::SetParametersResult DummyDiagPublisher::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (const auto & param : parameters) {
    const auto param_name = param.get_name();
    const auto split_names = split(param_name, '.');
    const auto & diag_name = split_names.at(0);
    auto it = std::find_if(
      std::begin(required_diags_), std::end(required_diags_),
      [&diag_name](DummyDiagConfig config) { return config.name == diag_name; });
    if (it == std::end(required_diags_)) {  // diag name not found
      result.successful = false;
      result.reason = "no matching diag name";
    } else {  // diag name found
      const auto status_prefix_str = diag_name + std::string(".status");
      const auto is_active_prefix_str = diag_name + std::string(".is_active");
      auto status_str = convertStatusToStr(it->status);
      auto prev_status_str = status_str;
      auto is_active = true;
      try {
        tier4_autoware_utils::updateParam(parameters, status_prefix_str, status_str);
        tier4_autoware_utils::updateParam(parameters, is_active_prefix_str, is_active);
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        result.successful = false;
        result.reason = e.what();
        return result;
      }
      const auto status = convertStrToStatus(status_str);
      if (!status) {
        result.successful = false;
        result.reason = "invalid status";
        return result;
      }
      result = updateDiag(diag_name, *it, is_active, *status);
    }  // end diag name found
  }
  return result;
}

// update diag with new param
rcl_interfaces::msg::SetParametersResult DummyDiagPublisher::updateDiag(
  const std::string diag_name, DummyDiagConfig & config, bool is_active, const Status status)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  if (is_active == config.is_active && config.status == status) {  // diag config not changed
    result.successful = true;
    result.reason = "config not changed";
  } else if (is_active == true && config.is_active == false) {  // newly activated
    config.is_active = true;
    if (config.status == status) {  // status not changed
      addDiagByStatus(diag_name, config.status);
    } else {  // status changed
      config.status = status;
      addDiagByStatus(diag_name, status);
    }
  } else {  // deactivated or status changed
    if (!updater_.removeByName(diag_name)) {
      result.successful = false;
      result.reason = "updater removal failed";
      return result;
    }
    if (is_active == false) {  // deactivated
      config.is_active = false;
    } else {  // status changed
      config.status = status;
      addDiagByStatus(diag_name, status);
    }
  }
  return result;
}

std::optional<DummyDiagPublisher::Status> DummyDiagPublisher::convertStrToStatus(
  std::string & status_str)
{
  static std::unordered_map<std::string, Status> const table = {
    {"OK", Status::OK}, {"Warn", Status::WARN}, {"Error", Status::ERROR}, {"Stale", Status::STALE}};

  auto it = table.find(status_str);
  Status status;
  if (it != table.end()) {
    status = it->second;
    return status;
  }
  return {};
}
std::string DummyDiagPublisher::convertStatusToStr(const Status & status)
{
  if (status == Status::OK) {
    return std::string("OK");
  } else if (status == Status::WARN) {
    return std::string("Warn");
  } else if (status == Status::ERROR) {
    return std::string("Error");
  } else {
    return std::string("Stale");
  }
}

void DummyDiagPublisher::loadRequiredDiags()
{
  const auto param_key = std::string("required_diags");
  const uint64_t depth = 3;
  const auto param_names = this->list_parameters({param_key}, depth).names;

  if (param_names.empty()) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  std::set<std::string> diag_names;

  for (const auto & param_name : param_names) {
    const auto split_names = split(param_name, '.');
    const auto & param_required_diags = split_names.at(0);
    const auto & param_diag = split_names.at(1);

    const auto diag_name_with_prefix = fmt::format("{0}.{1}", param_required_diags, param_diag);

    if (diag_names.count(diag_name_with_prefix) != 0) {
      continue;
    }

    diag_names.insert(diag_name_with_prefix);

    const auto is_active_key = diag_name_with_prefix + std::string(".is_active");
    std::string is_active_str;
    this->get_parameter_or(is_active_key, is_active_str, std::string("true"));
    const auto status_key = diag_name_with_prefix + std::string(".status");
    std::string status_str;
    this->get_parameter_or(status_key, status_str, std::string("OK"));

    bool is_active{};
    std::istringstream(is_active_str) >> std::boolalpha >> is_active;
    const auto status = convertStrToStatus(status_str);
    if (!status) {
      throw std::runtime_error(fmt::format("invalid status found: {}", status_str));
    }
    required_diags_.push_back({param_diag, is_active, *status});
  }
}

void DummyDiagPublisher::produceOKDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = diag_config_.msg_ok;

  stat.summary(status.level, status.message);
}
void DummyDiagPublisher::produceWarnDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = diag_config_.msg_warn;

  stat.summary(status.level, status.message);
}
void DummyDiagPublisher::produceErrorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = diag_config_.msg_error;

  stat.summary(status.level, status.message);
}
void DummyDiagPublisher::produceStaleDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  status.message = diag_config_.msg_stale;

  stat.summary(status.level, status.message);
}
void DummyDiagPublisher::addDiagByStatus(const std::string & diag_name, const Status status)
{
  if (status == Status::OK) {
    updater_.add(diag_name, this, &DummyDiagPublisher::produceOKDiagnostics);
  } else if (status == Status::WARN) {
    updater_.add(diag_name, this, &DummyDiagPublisher::produceWarnDiagnostics);
  } else if (status == Status::ERROR) {
    updater_.add(diag_name, this, &DummyDiagPublisher::produceErrorDiagnostics);
  } else if (status == Status::STALE) {
    updater_.add(diag_name, this, &DummyDiagPublisher::produceStaleDiagnostics);
  } else {
    throw std::runtime_error("invalid status");
  }
}

void DummyDiagPublisher::onTimer()
{
  updater_.force_update();
}

DummyDiagPublisher::DummyDiagPublisher()
: Node(
    "dummy_diag_publisher", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)),
  updater_(this)

{
  // Parameter
  update_rate_ = this->get_parameter_or("update_rate", 10.0);

  // Set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DummyDiagPublisher::paramCallback, this, std::placeholders::_1));

  // Diagnostic Updater
  loadRequiredDiags();

  const std::string hardware_id = "dummy_diag";
  updater_.setHardwareID(hardware_id);
  diag_config_ = DiagConfig{hardware_id, "OK", "Warn", "Error", "Stale"};
  for (const auto & config : required_diags_) {
    if (config.is_active) {
      addDiagByStatus(config.name, config.status);
    }
  }

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&DummyDiagPublisher::onTimer, this));
}
