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

#include "heartbeat.hpp"

#include <utility>

namespace default_ad_api
{

HeartbeatNode::HeartbeatNode(const rclcpp::NodeOptions & options) : Node("heartbeat", options)
{
  // Move this function so that the timer no longer holds it as a reference.
  const auto on_timer = [this]() {
    autoware_ad_api::system::Heartbeat::Message heartbeat;
    heartbeat.stamp = now();
    heartbeat.seq = ++sequence_;  // Wraps at 65535.
    pub_->publish(heartbeat);
  };

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_);

  const auto period = rclcpp::Rate(10.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, std::move(on_timer));
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::HeartbeatNode)
