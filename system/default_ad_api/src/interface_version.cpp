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

#include "default_ad_api/nodes/interface_version.hpp"

namespace default_ad_api
{

InterfaceVersionNode::InterfaceVersionNode(const rclcpp::NodeOptions & options)
: Node("interface_version", options)
{
  srv_ = component_interface_utils::create_service<ad_api::interface::version::T>(
    this, &InterfaceVersionNode::onInterfaceVersion);
}

void InterfaceVersionNode::onInterfaceVersion(
  const InterfaceVersion::Request::SharedPtr, const InterfaceVersion::Response::SharedPtr response)
{
  response->major = 0;
  response->minor = 1;
  response->patch = 0;
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::InterfaceVersionNode)
