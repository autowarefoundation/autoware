// Copyright 2024 The Autoware Contributors
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

#ifndef HEARTBEAT_HPP_
#define HEARTBEAT_HPP_

#include <autoware_ad_api_specs/system.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class HeartbeatNode : public rclcpp::Node
{
public:
  explicit HeartbeatNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  Pub<autoware_ad_api::system::Heartbeat> pub_;
  uint16_t sequence_ = 0;
};

}  // namespace default_ad_api

#endif  // HEARTBEAT_HPP_
