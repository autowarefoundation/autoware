// Copyright 2021 The Autoware Foundation
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

#include "vehicle_constants_manager/vehicle_constants_manager.hpp"
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware
{
namespace common
{
namespace vehicle_constants_manager
{

using float64_t = VehicleConstants::float64_t;


VehicleConstants::VehicleConstants(
  float64_t wheel_radius,
  float64_t wheel_width,
  float64_t wheel_base,
  float64_t wheel_tread,
  float64_t overhang_front,
  float64_t overhang_rear,
  float64_t overhang_left,
  float64_t overhang_right,
  float64_t vehicle_height,
  float64_t cg_to_rear,
  float64_t tire_cornering_stiffness_front,
  float64_t tire_cornering_stiffness_rear,
  float64_t mass_vehicle,
  float64_t inertia_yaw_kg_m_2,
  float64_t maximum_turning_angle_rad)
: wheel_radius(wheel_radius),
  wheel_width(wheel_width),
  wheel_base(wheel_base),
  wheel_tread(wheel_tread),
  overhang_front(overhang_front),
  overhang_rear(overhang_rear),
  overhang_left(overhang_left),
  overhang_right(overhang_right),
  vehicle_height(vehicle_height),
  cg_to_rear(cg_to_rear),
  tire_cornering_stiffness_front(tire_cornering_stiffness_front),
  tire_cornering_stiffness_rear(tire_cornering_stiffness_rear),
  mass_vehicle(mass_vehicle),
  inertia_yaw_kg_m2(inertia_yaw_kg_m_2),
  maximum_turning_angle_rad(maximum_turning_angle_rad),
  cg_to_front(wheel_base - cg_to_rear),
  vehicle_length(overhang_front + wheel_base + overhang_rear),
  vehicle_width(overhang_left + wheel_tread + overhang_right),
  offset_longitudinal_min(-overhang_rear),
  offset_longitudinal_max(wheel_base + overhang_front),
  offset_lateral_min(-(wheel_tread / 2.0 + overhang_right)),
  offset_lateral_max(wheel_tread / 2.0 + overhang_left),
  offset_height_min(-wheel_radius),
  offset_height_max(vehicle_height - wheel_radius)
{
  // Sanity Checks
  // Center of gravity must be between front and rear axles
  if (wheel_base < cg_to_rear) {
    throw std::runtime_error("wheel_base must be larger than cg_to_rear");
  }

  // These values must be positive
  auto throw_if_negative = [](float64_t number, const std::string & name) {
      if (number < 0.0) {
        throw std::runtime_error(
                name + " = " + std::to_string(number) +
                " shouldn't be negative.");
      }
    };
  throw_if_negative(wheel_radius, "wheel_radius");
  throw_if_negative(wheel_width, "wheel_width");
  throw_if_negative(wheel_base, "wheel_base");
  throw_if_negative(wheel_tread, "wheel_tread");
  throw_if_negative(overhang_front, "overhang_front");
  throw_if_negative(overhang_rear, "overhang_rear");
  throw_if_negative(overhang_left, "overhang_left");
  throw_if_negative(overhang_right, "overhang_right");
  throw_if_negative(vehicle_height, "vehicle_height");
  throw_if_negative(cg_to_rear, "cg_to_rear");
  throw_if_negative(tire_cornering_stiffness_front, "tire_cornering_stiffness_front");
  throw_if_negative(tire_cornering_stiffness_rear, "tire_cornering_stiffness_rear");
  throw_if_negative(mass_vehicle, "mass_vehicle");
  throw_if_negative(inertia_yaw_kg_m_2, "inertia_yaw_kg_m_2");
  throw_if_negative(maximum_turning_angle_rad, "maximum_turning_angle_rad");

  if (!(0.0 < maximum_turning_angle_rad && maximum_turning_angle_rad < (M_PI / 2.0))) {
    throw std::runtime_error(
            "maximum_turning_angle_rad must be positive and cannot be greater than 0.5*PI.");
  }
  minimum_turning_radius = wheel_base / tan(maximum_turning_angle_rad);
}

std::string VehicleConstants::str_pretty() const
{
  return std::string{
    "wheel_radius: " + std::to_string(wheel_radius) + "\n"
    "wheel_width: " + std::to_string(wheel_width) + "\n"
    "wheel_base: " + std::to_string(wheel_base) + "\n"
    "wheel_tread: " + std::to_string(wheel_tread) + "\n"
    "overhang_front: " + std::to_string(overhang_front) + "\n"
    "overhang_rear: " + std::to_string(overhang_rear) + "\n"
    "overhang_left: " + std::to_string(overhang_left) + "\n"
    "overhang_right: " + std::to_string(overhang_right) + "\n"
    "vehicle_height: " + std::to_string(vehicle_height) + "\n"
    "cg_to_rear: " + std::to_string(cg_to_rear) + "\n"
    "tire_cornering_stiffness_front: " + std::to_string(tire_cornering_stiffness_front) + "\n"
    "tire_cornering_stiffness_rear: " + std::to_string(tire_cornering_stiffness_rear) + "\n"
    "mass_vehicle: " + std::to_string(mass_vehicle) + "\n"
    "inertia_yaw_kg_m2: " + std::to_string(inertia_yaw_kg_m2) + "\n"
    "maximum_turning_angle_rad: " + std::to_string(maximum_turning_angle_rad) + "\n"
    "cg_to_front: " + std::to_string(cg_to_front) + "\n"
    "vehicle_length: " + std::to_string(vehicle_length) + "\n"
    "vehicle_width: " + std::to_string(vehicle_width) + "\n"
    "offset_longitudinal_min: " + std::to_string(offset_longitudinal_min) + "\n"
    "offset_longitudinal_max: " + std::to_string(offset_longitudinal_max) + "\n"
    "offset_lateral_min: " + std::to_string(offset_lateral_min) + "\n"
    "offset_lateral_max: " + std::to_string(offset_lateral_max) + "\n"
    "offset_height_min: " + std::to_string(offset_height_min) + "\n"
    "offset_height_max: " + std::to_string(offset_height_max) + "\n"
    "minimum_turning_radius: " + std::to_string(minimum_turning_radius) + "\n"
  };
}

VehicleConstants declare_and_get_vehicle_constants(rclcpp::Node & node)
{
  // Initialize the parameters
  const std::string ns = "vehicle.";
  std::map<std::string, float64_t> params{
    std::make_pair(ns + "wheel_radius", -1.0),
    std::make_pair(ns + "wheel_width", -1.0),
    std::make_pair(ns + "wheel_base", -1.0),
    std::make_pair(ns + "wheel_tread", -1.0),
    std::make_pair(ns + "overhang_front", -1.0),
    std::make_pair(ns + "overhang_rear", -1.0),
    std::make_pair(ns + "overhang_left", -1.0),
    std::make_pair(ns + "overhang_right", -1.0),
    std::make_pair(ns + "vehicle_height", -1.0),
    std::make_pair(ns + "cg_to_rear", -1.0),
    std::make_pair(ns + "tire_cornering_stiffness_front", -1.0),
    std::make_pair(ns + "tire_cornering_stiffness_rear", -1.0),
    std::make_pair(ns + "mass_vehicle", -1.0),
    std::make_pair(ns + "inertia_yaw_kg_m2", -1.0),
    std::make_pair(ns + "maximum_turning_angle_rad", -1.0)
  };

  // Try to get parameter values from parameter_overrides set either from .yaml
  // or with args.
  for (auto & pair : params) {
    // If it is already declared
    if (node.has_parameter(pair.first)) {
      node.get_parameter(pair.first, pair.second);
      continue;
    }

    pair.second = node.declare_parameter(pair.first, rclcpp::ParameterType::PARAMETER_DOUBLE).get<float64_t>();
  }

  return VehicleConstants(
    params.at(ns + "wheel_radius"),
    params.at(ns + "wheel_width"),
    params.at(ns + "wheel_base"),
    params.at(ns + "wheel_tread"),
    params.at(ns + "overhang_front"),
    params.at(ns + "overhang_rear"),
    params.at(ns + "overhang_left"),
    params.at(ns + "overhang_right"),
    params.at(ns + "vehicle_height"),
    params.at(ns + "cg_to_rear"),
    params.at(ns + "tire_cornering_stiffness_front"),
    params.at(ns + "tire_cornering_stiffness_rear"),
    params.at(ns + "mass_vehicle"),
    params.at(ns + "inertia_yaw_kg_m2"),
    params.at(ns + "maximum_turning_angle_rad")
  );
}
}  // namespace vehicle_constants_manager
}  // namespace common
}  // namespace autoware
