// Copyright 2022 TIER IV, Inc.
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

#include "service_log_checker.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

ServiceLogChecker::ServiceLogChecker() : Node("service_log_checker"), diagnostics_(this)
{
  sub_ = create_subscription<ServiceLog>(
    "/service_log", 50, std::bind(&ServiceLogChecker::on_service_log, this, std::placeholders::_1));

  diagnostics_.setHardwareID(get_name());
  diagnostics_.add("response_status", this, &ServiceLogChecker::update_diagnostics);
}

void ServiceLogChecker::on_service_log(const ServiceLog::ConstSharedPtr msg)
{
  try {
    // Ignore service request.
    if (msg->type == ServiceLog::CLIENT_REQUEST || msg->type == ServiceLog::SERVER_REQUEST) {
      return;
    }

    // Ignore service errors.
    if (msg->type == ServiceLog::ERROR_UNREADY) {
      return set_error(*msg, "not ready");
    }
    if (msg->type == ServiceLog::ERROR_TIMEOUT) {
      return set_error(*msg, "timeout");
    }

    // Ignore version API because it doesn't have response status.
    if (msg->name == "/api/interface/version") {
      return;
    }

    // Parse response data.
    const auto status = YAML::Load(msg->yaml)["status"];
    if (!status) {
      return set_error(*msg, "no response status");
    }

    // Check response status.
    const auto success = status["success"].as<bool>();
    if (!success) {
      const auto message = status["message"].as<std::string>();
      const auto code = status["code"].as<uint16_t>();
      return set_error(*msg, fmt::format("status code {} '{}'", code, message));
    }
  } catch (const YAML::Exception & error) {
    return set_error(*msg, fmt::format("invalid data: '{}'", error.what()));
  }

  set_success(*msg);
}

void ServiceLogChecker::set_success(const ServiceLog & msg)
{
  errors_.erase(fmt::format("{} ({})", msg.name, msg.node));
}

void ServiceLogChecker::set_error(const ServiceLog & msg, const std::string & log)
{
  errors_[fmt::format("{} ({})", msg.name, msg.node)] = log;
  RCLCPP_ERROR_STREAM(get_logger(), fmt::format("{}: {} ({})", msg.name, log, msg.node));
}

void ServiceLogChecker::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  for (const auto & error : errors_) {
    stat.add(error.first, error.second);
  }

  if (errors_.empty()) {
    stat.summary(DiagnosticStatus::OK, "OK");
  } else {
    stat.summary(DiagnosticStatus::ERROR, "ERROR");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ServiceLogChecker>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
