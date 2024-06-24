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

#ifndef AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_
#define AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

namespace autoware::vehicle_info_utils
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

  autoware::universe_utils::LinearRing2d createFootprint(const double margin = 0.0) const;
  autoware::universe_utils::LinearRing2d createFootprint(
    const double lat_margin, const double lon_margin) const;

  double calcMaxCurvature() const;
  double calcCurvatureFromSteerAngle(const double steer_angle) const;
};

/// Create vehicle info from base parameters
VehicleInfo createVehicleInfo(
  const double wheel_radius_m, const double wheel_width_m, const double wheel_base_m,
  const double wheel_tread_m, const double front_overhang_m, const double rear_overhang_m,
  const double left_overhang_m, const double right_overhang_m, const double vehicle_height_m,
  const double max_steer_angle_rad);

}  // namespace autoware::vehicle_info_utils

#endif  // AUTOWARE_VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_
