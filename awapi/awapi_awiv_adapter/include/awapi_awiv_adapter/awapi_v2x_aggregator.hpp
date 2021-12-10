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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_V2X_AGGREGATOR_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_V2X_AGGREGATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <map>
#include <string>
#include <vector>

namespace autoware_api
{
using Command = tier4_v2x_msgs::msg::InfrastructureCommand;
using CommandArray = tier4_v2x_msgs::msg::InfrastructureCommandArray;
using State = tier4_v2x_msgs::msg::VirtualTrafficLightState;
using StateArray = tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;

class AutowareIvV2XAggregator
{
public:
  explicit AutowareIvV2XAggregator(rclcpp::Node & node);

  CommandArray::ConstSharedPtr updateV2XCommand(const CommandArray::ConstSharedPtr & msg);

  StateArray::ConstSharedPtr updateV2XState(const StateArray::ConstSharedPtr & msg);

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  const double max_delay_sec_{5.0};
  const double max_clock_error_sec_{300.0};
  std::map<std::string, Command> command_map_;
  std::map<std::string, State> state_map_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_V2X_AGGREGATOR_HPP_
