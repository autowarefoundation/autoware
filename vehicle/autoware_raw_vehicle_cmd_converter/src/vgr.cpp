//  Copyright 2024 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware_raw_vehicle_cmd_converter/vgr.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::raw_vehicle_cmd_converter
{

void VGR::setCoefficients(const double a, const double b, const double c)
{
  vgr_coef_a_ = a;
  vgr_coef_b_ = b;
  vgr_coef_c_ = c;
}

double VGR::calculateVariableGearRatio(const double vel, const double steer_wheel) const
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

double VGR::calculateSteeringTireState(const double vel, const double steer_wheel) const
{
  const double adaptive_gear_ratio = calculateVariableGearRatio(vel, steer_wheel);
  return steer_wheel / adaptive_gear_ratio;
}
}  // namespace autoware::raw_vehicle_cmd_converter
