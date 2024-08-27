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

#include "localization.hpp"

#include "utils/localization_conversion.hpp"

namespace autoware::default_adapi
{

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("localization", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.relay_message(pub_state_, sub_state_);
  adaptor.init_cli(cli_initialize_);
  adaptor.init_srv(srv_initialize_, this, &LocalizationNode::on_initialize, group_cli_);
}

void LocalizationNode::on_initialize(
  const autoware_ad_api::localization::Initialize::Service::Request::SharedPtr req,
  const autoware_ad_api::localization::Initialize::Service::Response::SharedPtr res)
{
  res->status = localization_conversion::convert_call(cli_initialize_, req);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::LocalizationNode)
