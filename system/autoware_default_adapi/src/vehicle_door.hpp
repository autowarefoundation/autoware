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

#ifndef VEHICLE_DOOR_HPP_
#define VEHICLE_DOOR_HPP_

#include <autoware_ad_api_specs/vehicle.hpp>
#include <component_interface_specs/vehicle.hpp>
#include <rclcpp/rclcpp.hpp>

#include <optional>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

class VehicleDoorNode : public rclcpp::Node
{
public:
  explicit VehicleDoorNode(const rclcpp::NodeOptions & options);

private:
  void on_status(vehicle_interface::DoorStatus::Message::ConstSharedPtr msg);
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Srv<autoware_ad_api::vehicle::DoorCommand> srv_command_;
  Srv<autoware_ad_api::vehicle::DoorLayout> srv_layout_;
  Pub<autoware_ad_api::vehicle::DoorStatus> pub_status_;
  Cli<vehicle_interface::DoorCommand> cli_command_;
  Cli<vehicle_interface::DoorLayout> cli_layout_;
  Sub<vehicle_interface::DoorStatus> sub_status_;
  std::optional<vehicle_interface::DoorStatus::Message> status_;
};

}  // namespace autoware::default_adapi

#endif  // VEHICLE_DOOR_HPP_
