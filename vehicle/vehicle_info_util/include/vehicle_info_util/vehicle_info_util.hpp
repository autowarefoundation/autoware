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

#ifndef VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_
#define VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_

#include "vehicle_info_util/vehicle_info.hpp"

#include <rclcpp/rclcpp.hpp>

namespace vehicle_info_util
{
/// This is a convenience class for saving you from declaring all parameters
/// manually and calculating derived parameters.
/// This class supposes that necessary parameters are set when the node is launched.
class VehicleInfoUtil
{
public:
  /// Constructor
  explicit VehicleInfoUtil(rclcpp::Node & node);

  /// Get vehicle info
  VehicleInfo getVehicleInfo();

private:
  /// Buffer for base parameters
  VehicleInfo vehicle_info_;
};

}  // namespace vehicle_info_util

#endif  // VEHICLE_INFO_UTIL__VEHICLE_INFO_UTIL_HPP_
