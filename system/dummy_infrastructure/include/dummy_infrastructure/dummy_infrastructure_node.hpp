// Copyright 2021 Tier IV
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

#ifndef DUMMY_INFRASTRUCTURE__DUMMY_INFRASTRUCTURE_NODE_HPP_
#define DUMMY_INFRASTRUCTURE__DUMMY_INFRASTRUCTURE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <tier4_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace dummy_infrastructure
{
using tier4_v2x_msgs::msg::InfrastructureCommand;
using tier4_v2x_msgs::msg::InfrastructureCommandArray;
using tier4_v2x_msgs::msg::VirtualTrafficLightState;
using tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;

class DummyInfrastructureNode : public rclcpp::Node
{
public:
  explicit DummyInfrastructureNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
    bool use_first_command{};
    bool use_command_state{};
    std::string instrument_id{};
    bool approval{};
    bool is_finalized{};
  };

private:
  // Subscriber
  rclcpp::Subscription<InfrastructureCommandArray>::SharedPtr sub_command_array_{};

  // Callback
  void onCommandArray(const InfrastructureCommandArray::ConstSharedPtr msg);

  // Data Buffer
  InfrastructureCommandArray::ConstSharedPtr command_array_{};

  // Publisher
  rclcpp::Publisher<VirtualTrafficLightStateArray>::SharedPtr pub_state_array_{};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};

  bool isDataReady();
  void onTimer();

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};
};

}  // namespace dummy_infrastructure

#endif  // DUMMY_INFRASTRUCTURE__DUMMY_INFRASTRUCTURE_NODE_HPP_
