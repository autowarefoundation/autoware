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

#include "interface.hpp"

namespace default_ad_api
{

InterfaceNode::InterfaceNode(const rclcpp::NodeOptions & options) : Node("interface", options)
{
  using InterfaceVersion = autoware_ad_api_msgs::srv::InterfaceVersion;

  const auto on_interface_version = [](SERVICE_ARG_NO_REQ(InterfaceVersion)) {
    response->major = 0;
    response->minor = 1;
    response->patch = 0;
  };

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_srv(srv_, on_interface_version);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::InterfaceNode)
