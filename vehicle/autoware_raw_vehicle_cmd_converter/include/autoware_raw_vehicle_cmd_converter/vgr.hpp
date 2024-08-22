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

#ifndef AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__VGR_HPP_
#define AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__VGR_HPP_

namespace autoware::raw_vehicle_cmd_converter
{
class VGR
{
public:
  VGR() = default;
  VGR(const double vgr_coef_a, const double vgr_coef_b, const double vgr_coef_c)
  : vgr_coef_a_(vgr_coef_a), vgr_coef_b_(vgr_coef_b), vgr_coef_c_(vgr_coef_c)
  {
  }
  void setCoefficients(double a, double b, double c);
  double calculateVariableGearRatio(double vel, double steer_wheel) const;
  double calculateSteeringTireState(double vel, double steer_wheel) const;

private:
  double vgr_coef_a_{0.0};
  double vgr_coef_b_{0.0};
  double vgr_coef_c_{0.0};
};
}  // namespace autoware::raw_vehicle_cmd_converter

#endif  // AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__VGR_HPP_
