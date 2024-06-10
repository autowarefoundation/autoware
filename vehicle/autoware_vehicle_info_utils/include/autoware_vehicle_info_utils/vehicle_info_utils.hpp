// Copyright 2015-2021 Autoware Foundation
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

#ifndef AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_
#define AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_

#include "autoware_vehicle_info_utils/vehicle_info.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::vehicle_info_utils
{
/// This is a convenience class for saving you from declaring all parameters
/// manually and calculating derived parameters.
/// This class supposes that necessary parameters are set when the node is launched.
class VehicleInfoUtils
{
public:
  /// Constructor
  explicit VehicleInfoUtils(rclcpp::Node & node);

  /// Get vehicle info
  VehicleInfo getVehicleInfo();

private:
  /// Buffer for base parameters
  VehicleInfo vehicle_info_;
};

}  // namespace autoware::vehicle_info_utils

#endif  // AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_UTILS_HPP_
