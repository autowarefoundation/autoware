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

#ifndef RTC_REPLAYER__RTC_REPLAYER_NODE_HPP_
#define RTC_REPLAYER__RTC_REPLAYER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/srv/cooperate_commands.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rtc_replayer
{
using std::placeholders::_1;
using std::placeholders::_2;
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::CooperateCommands;
using unique_identifier_msgs::msg::UUID;
class RTCReplayerNode : public rclcpp::Node
{
public:
  explicit RTCReplayerNode(const rclcpp::NodeOptions & node_options);

private:
  void onCooperateStatus(const CooperateStatusArray::ConstSharedPtr msg);

  rclcpp::Subscription<CooperateStatusArray>::SharedPtr sub_statuses_;
  rclcpp::Client<CooperateCommands>::SharedPtr client_rtc_commands_;
  std::map<std::string, uint8_t> prev_cmd_status_;
};

}  // namespace rtc_replayer

#endif  // RTC_REPLAYER__RTC_REPLAYER_NODE_HPP_
