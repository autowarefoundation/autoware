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

#include "fail_safe.hpp"

namespace default_ad_api
{

FailSafeNode::FailSafeNode(const rclcpp::NodeOptions & options) : Node("fail_safe", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_mrm_state_);
  adaptor.init_sub(sub_mrm_state_, this, &FailSafeNode::on_state);

  prev_state_.state = MrmState::UNKNOWN;
}

void FailSafeNode::on_state(const MrmState::ConstSharedPtr msg)
{
  prev_state_.stamp = msg->stamp;
  if (prev_state_ != *msg) {
    prev_state_ = *msg;
    pub_mrm_state_->publish(*msg);
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::FailSafeNode)
