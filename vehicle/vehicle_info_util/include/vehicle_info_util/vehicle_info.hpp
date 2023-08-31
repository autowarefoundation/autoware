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

#ifndef VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
#define VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

namespace vehicle_info_util
{
/// Data class for vehicle info
struct VehicleInfo
{
  // Base parameters. These describe the vehicle's bounding box and the
  // position and radius of the wheels.
  double wheel_radius_m;
  double wheel_width_m;
  double wheel_base_m;
  double wheel_tread_m;
  double front_overhang_m;
  double rear_overhang_m;
  double left_overhang_m;
  double right_overhang_m;
  double vehicle_height_m;
  double max_steer_angle_rad;

  // Derived parameters, i.e. calculated from base parameters
  // The offset values are relative to the base frame origin, which is located
  // on the ground below the middle of the rear axle, and can be negative.
  double vehicle_length_m;
  double vehicle_width_m;
  double min_longitudinal_offset_m;
  double max_longitudinal_offset_m;
  double min_lateral_offset_m;
  double max_lateral_offset_m;
  double min_height_offset_m;
  double max_height_offset_m;

  tier4_autoware_utils::LinearRing2d createFootprint(const double margin = 0.0) const;
  tier4_autoware_utils::LinearRing2d createFootprint(
    const double lat_margin, const double lon_margin) const;
};

/// Create vehicle info from base parameters
VehicleInfo createVehicleInfo(
  const double wheel_radius_m, const double wheel_width_m, const double wheel_base_m,
  const double wheel_tread_m, const double front_overhang_m, const double rear_overhang_m,
  const double left_overhang_m, const double right_overhang_m, const double vehicle_height_m,
  const double max_steer_angle_rad);

}  // namespace vehicle_info_util

#endif  // VEHICLE_INFO_UTIL__VEHICLE_INFO_HPP_
