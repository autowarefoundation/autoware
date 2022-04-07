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

#include "dummy_diag_publisher/dummy_diag_publisher_node.hpp"

#include <rclcpp/create_timer.hpp>
#include <tier4_autoware_utils/ros/update_param.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

rcl_interfaces::msg::SetParametersResult DummyDiagPublisherNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  DummyDiagPublisherConfig config = config_;
  try {
    int status = static_cast<int>(config.status);
    tier4_autoware_utils::updateParam(parameters, "status", status);
    config.status = Status(status);
    tier4_autoware_utils::updateParam(parameters, "is_active", config.is_active);
    config_ = config;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void DummyDiagPublisherNode::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  if (config_.status == Status::OK) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = diag_config_.msg_ok;
  } else if (config_.status == Status::WARN) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = diag_config_.msg_warn;
  } else if (config_.status == Status::ERROR) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = diag_config_.msg_error;
  } else if (config_.status == Status::STALE) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = diag_config_.msg_stale;
  } else {
    throw std::runtime_error("invalid status");
  }

  stat.summary(status.level, status.message);
}

void DummyDiagPublisherNode::onTimer()
{
  if (config_.is_active) {
    updater_.force_update();
  }
}

DummyDiagPublisherNode::DummyDiagPublisherNode(const rclcpp::NodeOptions & node_options)
: Node("dummy_diag_publisher", node_options)
{
  // Parameter
  update_rate_ = declare_parameter("update_rate", 10.0);
  const std::string diag_name = this->declare_parameter<std::string>("diag_name");
  const std::string hardware_id = "dummy_diag_" + diag_name;
  diag_config_ = DiagConfig{
    diag_name, hardware_id, "OK", "Warn", "Error", "Stale",
  };

  // Set parameter callback
  config_.status = static_cast<Status>(this->declare_parameter("status", 0));
  config_.is_active = this->declare_parameter("is_active", true);
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DummyDiagPublisherNode::paramCallback, this, std::placeholders::_1));

  // Diagnostic Updater
  updater_.setHardwareID(diag_config_.hardware_id);
  updater_.add(diag_config_.name, this, &DummyDiagPublisherNode::produceDiagnostics);

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&DummyDiagPublisherNode::onTimer, this));
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DummyDiagPublisherNode)
