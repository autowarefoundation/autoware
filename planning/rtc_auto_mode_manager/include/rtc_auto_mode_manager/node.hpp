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

#ifndef RTC_AUTO_MODE_MANAGER__NODE_HPP_
#define RTC_AUTO_MODE_MANAGER__NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rtc_auto_mode_manager/rtc_auto_mode_manager_interface.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rtc_auto_mode_manager
{
class RTCAutoModeManagerNode : public rclcpp::Node
{
public:
  explicit RTCAutoModeManagerNode(const rclcpp::NodeOptions & node_options);

private:
  std::vector<std::shared_ptr<RTCAutoModeManagerInterface>> managers_;
};

}  // namespace rtc_auto_mode_manager

#endif  // RTC_AUTO_MODE_MANAGER__NODE_HPP_
