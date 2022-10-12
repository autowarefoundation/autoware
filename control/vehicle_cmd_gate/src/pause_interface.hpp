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

#ifndef PAUSE_INTERFACE_HPP_
#define PAUSE_INTERFACE_HPP_

#include <component_interface_specs/control.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

namespace vehicle_cmd_gate
{

class PauseInterface
{
private:
  static constexpr double eps = 1e-3;
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using SetPause = control_interface::SetPause;
  using IsPaused = control_interface::IsPaused;
  using IsStartRequested = control_interface::IsStartRequested;

public:
  explicit PauseInterface(rclcpp::Node * node);
  bool is_paused();
  void publish();
  void update(const AckermannControlCommand & control);

private:
  bool is_paused_;
  bool is_start_requested_;
  std::optional<bool> prev_is_paused_;
  std::optional<bool> prev_is_start_requested_;

  rclcpp::Node * node_;
  component_interface_utils::Service<SetPause>::SharedPtr srv_set_pause_;
  component_interface_utils::Publisher<IsPaused>::SharedPtr pub_is_paused_;
  component_interface_utils::Publisher<IsStartRequested>::SharedPtr pub_is_start_requested_;

  void on_pause(
    const SetPause::Service::Request::SharedPtr req,
    const SetPause::Service::Response::SharedPtr res);
};

}  // namespace vehicle_cmd_gate

#endif  // PAUSE_INTERFACE_HPP_
