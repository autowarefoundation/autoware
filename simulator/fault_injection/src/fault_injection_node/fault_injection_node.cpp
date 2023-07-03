// Copyright 2021 Tier IV, Inc.
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

#include "fault_injection/fault_injection_node.hpp"

#include <tier4_autoware_utils/ros/update_param.hpp>

#include <memory>
#include <string>
#include <unordered_map>
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

namespace fault_injection
{
#ifdef ROS_DISTRO_GALACTIC
using rosidl_generator_traits::to_yaml;
#endif

FaultInjectionNode::FaultInjectionNode(rclcpp::NodeOptions node_options)
: Node("fault_injection", node_options.automatically_declare_parameters_from_overrides(true)),
  updater_(this)
{
  updater_.setHardwareID("fault_injection");

  using std::placeholders::_1;

  // Parameter Server
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&FaultInjectionNode::onSetParam, this, _1));

  // Subscriber
  sub_simulation_events_ = this->create_subscription<SimulationEvents>(
    "~/input/simulation_events", rclcpp::QoS{rclcpp::KeepLast(10)},
    std::bind(&FaultInjectionNode::onSimulationEvents, this, _1));

  // Load all config
  for (const auto & diag : readEventDiagList()) {
    diagnostic_storage_.registerEvent(diag);
    updater_.add(
      diag.diag_name, std::bind(&FaultInjectionNode::updateEventDiag, this, _1, diag.sim_name));
  }
}

void FaultInjectionNode::onSimulationEvents(const SimulationEvents::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received data: %s", to_yaml(*msg).c_str());
  for (const auto & event : msg->fault_injection_events) {
    if (diagnostic_storage_.isEventRegistered(event.name)) {
      diagnostic_storage_.updateLevel(event.name, event.level);
    }
  }
}

void FaultInjectionNode::updateEventDiag(
  diagnostic_updater::DiagnosticStatusWrapper & wrap, const std::string & event_name)
{
  const auto diag = diagnostic_storage_.getDiag(event_name);
  wrap.name = diag.name;
  wrap.level = diag.level;
  wrap.message = diag.message;
  wrap.hardware_id = diag.hardware_id;
}

rcl_interfaces::msg::SetParametersResult FaultInjectionNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  RCLCPP_DEBUG(this->get_logger(), "call onSetParam");

  try {
    double value;
    if (tier4_autoware_utils::updateParam(params, "diagnostic_updater.period", value)) {
      updater_.setPeriod(value);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

std::vector<DiagConfig> FaultInjectionNode::readEventDiagList()
{
  // Expected parameter name is "event_diag_list.param_name".
  // In this case, depth is 2.
  const auto param_name_list = list_parameters({"event_diag_list"}, 2);

  std::vector<DiagConfig> diag_configs;
  // NOTE: param_name_list.prefixes returns {"event_diag_list"}
  //       and param_name_list.names returns {"event_diag_list.param_name"}
  for (const auto & param_name : param_name_list.names) {
    // Trim parameter prefix
    const auto sim_name = split(param_name, '.').back();
    const auto diag_name = get_parameter(param_name).as_string();
    diag_configs.emplace_back(DiagConfig{sim_name, diag_name});
    RCLCPP_DEBUG(get_logger(), "Parameter: %s, value: %s", sim_name.c_str(), diag_name.c_str());
  }

  return diag_configs;
}
}  // namespace fault_injection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fault_injection::FaultInjectionNode)
