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

#include "vehicle_door.hpp"

#include "utils/topics.hpp"

namespace autoware::default_adapi
{

VehicleDoorNode::VehicleDoorNode(const rclcpp::NodeOptions & options)
: Node("vehicle_door", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.relay_service(cli_command_, srv_command_, group_cli_, 3.0);
  adaptor.relay_service(cli_layout_, srv_layout_, group_cli_, 3.0);
  adaptor.init_pub(pub_status_);
  adaptor.init_sub(sub_status_, this, &VehicleDoorNode::on_status);
}

void VehicleDoorNode::on_status(vehicle_interface::DoorStatus::Message::ConstSharedPtr msg)
{
  utils::notify(
    pub_status_, status_, *msg, utils::ignore_stamp<vehicle_interface::DoorStatus::Message>);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::VehicleDoorNode)
