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

#include "main.hpp"

#include <vector>

namespace component_state_monitor
{

// clang-format off

const std::vector<StateType> types = {
  StateType::kLaunch,
  StateType::kAutonomous
};

const std::vector<Module> modules = {
  Module::kSensing,
  Module::kPerception,
  Module::kMap,
  Module::kLocalization,
  Module::kPlanning,
  Module::kControl,
  Module::kVehicle,
  Module::kSystem
};

const std::unordered_map<StateType, std::string> type_names = {
  {StateType::kLaunch, "launch"},
  {StateType::kAutonomous, "autonomous"}
};

const std::unordered_map<Module, std::string> module_names = {
  {Module::kSensing, "sensing"},
  {Module::kPerception, "perception"},
  {Module::kMap, "map"},
  {Module::kLocalization, "localization"},
  {Module::kPlanning, "planning"},
  {Module::kControl, "control"},
  {Module::kVehicle, "vehicle"},
  {Module::kSystem, "system"}
};

// clang-format on

StateMonitor::StateMonitor(const rclcpp::NodeOptions & options) : Node("state", options)
{
  for (const auto & type : types) {
    for (const auto & module : modules) {
      const auto name = type_names.at(type) + "." + module_names.at(module);
      topics_[type][module] = declare_parameter(name, std::vector<std::string>());
    }
  }

  for (const auto & type : types) {
    for (const auto & module : modules) {
      for (const auto & topic : topics_[type][module]) {
        levels_[topic] = DiagnosticStatus::STALE;
      }
    }
  }

  const auto qos = rclcpp::QoS(1).transient_local();
  for (const auto & type : types) {
    for (const auto & module : modules) {
      const auto name = "~/" + type_names.at(type) + "/" + module_names.at(module);
      pubs_[type][module] = create_publisher<ModeChangeAvailable>(name, qos);
    }
  }

  sub_diag_ = create_subscription<DiagnosticArray>(
    "/diagnostics", 100, std::bind(&StateMonitor::on_diag, this, std::placeholders::_1));

  const auto rate = rclcpp::Rate(10.0);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this] { on_timer(); });
}

void StateMonitor::update_state(const StateType & type, const Module & module, bool state)
{
  if (states_[type].count(module) == 0 || states_[type][module] != state) {
    ModeChangeAvailable msg;
    msg.stamp = now();
    msg.available = state;
    pubs_[type][module]->publish(msg);
  }
  states_[type][module] = state;
}

void StateMonitor::on_timer()
{
  const auto is_ok = [this](const std::string & name) {
    return levels_[name] == DiagnosticStatus::OK;
  };

  const auto state = [is_ok](const std::vector<std::string> & names) {
    return std::all_of(names.begin(), names.end(), is_ok);
  };

  for (const auto & module : modules) {
    const auto launch_state = state(topics_[StateType::kLaunch][module]);
    const auto auto_state = state(topics_[StateType::kAutonomous][module]);
    update_state(StateType::kLaunch, module, launch_state);
    update_state(StateType::kAutonomous, module, launch_state & auto_state);
  }
}

void StateMonitor::on_diag(const DiagnosticArray::ConstSharedPtr msg)
{
  for (const auto & status : msg->status) {
    if (status.hardware_id == "topic_state_monitor") {
      if (levels_.count(status.name)) {
        levels_[status.name] = status.level;
      }
    }
  }
}

}  // namespace component_state_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_state_monitor::StateMonitor)
