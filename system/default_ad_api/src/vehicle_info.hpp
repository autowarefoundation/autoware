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

#ifndef VEHICLE_INFO_HPP_
#define VEHICLE_INFO_HPP_

#include <autoware_ad_api_specs/vehicle.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class VehicleInfoNode : public rclcpp::Node
{
public:
  explicit VehicleInfoNode(const rclcpp::NodeOptions & options);

private:
  Srv<autoware_ad_api::vehicle::Dimensions> srv_dimensions_;
  autoware_adapi_v1_msgs::msg::VehicleDimensions dimensions_;
};

}  // namespace default_ad_api

#endif  // VEHICLE_INFO_HPP_
