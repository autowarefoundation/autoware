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

#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace component_state_monitor
{

// clang-format off

enum class StateType {
  kLaunch,
  kAutonomous
};

enum class Module {
  kSensing,
  kPerception,
  kMap,
  kLocalization,
  kPlanning,
  kControl,
  kVehicle,
  kSystem
};

// clang-format on

class StateMonitor : public rclcpp::Node
{
public:
  explicit StateMonitor(const rclcpp::NodeOptions & options);

private:
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using DiagnosticLevel = DiagnosticArray::_status_type::value_type::_level_type;
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;

  template <class T>
  using TypeModuleMap = std::unordered_map<StateType, std::unordered_map<Module, T>>;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub_diag_;
  TypeModuleMap<rclcpp::Publisher<ModeChangeAvailable>::SharedPtr> pubs_;
  TypeModuleMap<bool> states_;
  TypeModuleMap<std::vector<std::string>> topics_;
  std::unordered_map<std::string, DiagnosticLevel> levels_;

  void update_state(const StateType & type, const Module & module, bool state);
  void on_timer();
  void on_diag(const DiagnosticArray::ConstSharedPtr msg);
};

}  // namespace component_state_monitor

#endif  // MAIN_HPP_
