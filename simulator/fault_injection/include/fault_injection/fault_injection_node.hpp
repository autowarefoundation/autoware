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

#ifndef FAULT_INJECTION__FAULT_INJECTION_NODE_HPP_
#define FAULT_INJECTION__FAULT_INJECTION_NODE_HPP_

#include "fault_injection/diagnostic_storage.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_simulation_msgs/msg/simulation_events.hpp>

#include <string>
#include <vector>

namespace fault_injection
{
using tier4_simulation_msgs::msg::SimulationEvents;

class FaultInjectionNode : public rclcpp::Node
{
public:
  explicit FaultInjectionNode(rclcpp::NodeOptions node_options);

private:
  // Subscribers
  void onSimulationEvents(const SimulationEvents::ConstSharedPtr msg);
  rclcpp::Subscription<SimulationEvents>::SharedPtr sub_simulation_events_;

  // Parameter Server
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  void updateEventDiag(
    diagnostic_updater::DiagnosticStatusWrapper & wrap, const std::string & event_name);

  void addDiag(
    const diagnostic_msgs::msg::DiagnosticStatus & status, diagnostic_updater::Updater & updater);

  std::vector<DiagConfig> readEventDiagList();

  diagnostic_updater::Updater updater_;

  DiagnosticStorage diagnostic_storage_;
};

}  // namespace fault_injection

#endif  // FAULT_INJECTION__FAULT_INJECTION_NODE_HPP_
