// Copyright 2023 TIER IV, Inc.
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

#include "glog_component/glog_component.hpp"

GlogComponent::GlogComponent(const rclcpp::NodeOptions & node_options)
: Node("glog_component", node_options)
{
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("glog_component");
    google::InstallFailureSignalHandler();
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GlogComponent)
