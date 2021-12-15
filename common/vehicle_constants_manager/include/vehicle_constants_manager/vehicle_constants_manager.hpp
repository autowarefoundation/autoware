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

/// @copyright Copyright 2021 The Autoware Foundation
/// @file vehicle_constants_manager.hpp
/// @brief This file defines the vehicle_constants_manager class.

#ifndef VEHICLE_CONSTANTS_MANAGER__VEHICLE_CONSTANTS_MANAGER_HPP_
#define VEHICLE_CONSTANTS_MANAGER__VEHICLE_CONSTANTS_MANAGER_HPP_

#include <common/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_constants_manager/visibility_control.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace autoware
{
namespace common
{
namespace vehicle_constants_manager
{
/// @brief A struct that holds vehicle specific parameters that don't change over time.
/// @details These parameters include wheel size, vehicle mass, vehicle size, tire cornering
/// stiffness, moment of inertia, center of gravity.
struct VEHICLE_CONSTANTS_MANAGER_PUBLIC VehicleConstants
{
  using SharedPtr = std::shared_ptr<VehicleConstants>;
  using ConstSharedPtr = const SharedPtr;

  using float64_t = autoware::common::types::float64_t;

  /// @brief Construct a new instance of the measurement from a state vector.
  /// @param[in] wheel_radius Radius of a wheel
  /// @param[in] wheel_width Width of a wheel
  /// @param[in] wheel_base Distance between front and rear axles
  /// @param[in] wheel_tread Distance between centres of left and right wheels
  /// @param[in] overhang_front Distance from front axle to fore-most point of the vehicle
  /// @param[in] overhang_rear Distance from rear axle to rear-most point of the vehicle
  /// @param[in] overhang_left Distance from left wheel center to left-most point of the vehicle
  /// @param[in] overhang_right Distance from left wheel center to right-most point of the vehicle
  /// @param[in] vehicle_height Distance from ground plane to the top-most point of the vehicle
  /// @param[in] cg_to_rear Distance between center of gravity to rear axle
  /// @param[in] tire_cornering_stiffness_front Cornering stiffness for front wheels
  /// @param[in] tire_cornering_stiffness_rear Cornering stiffness for rear wheels
  /// @param[in] mass_vehicle Mass of the vehicle
  /// @param[in] inertia_yaw_kg_m_2 Moment of Inertia of the vehicle on Z axis
  /// @param[in] maximum_turning_angle_rad Maximum turning angle for cars front axis
  /// @throws std::runtime_error if certain parameters are negative.
  /// @throws std::runtime_error if cg_to_rear is larger than wheel_base (center of gravity must be
  /// within front and rear axles.)
  explicit VehicleConstants(
    float64_t wheel_radius, float64_t wheel_width, float64_t wheel_base,
    float64_t wheel_tread, float64_t overhang_front,
    float64_t overhang_rear, float64_t overhang_left,
    float64_t overhang_right, float64_t vehicle_height,
    float64_t cg_to_rear, float64_t tire_cornering_stiffness_front,
    float64_t tire_cornering_stiffness_rear, float64_t mass_vehicle,
    float64_t inertia_yaw_kg_m_2, float64_t maximum_turning_angle_rad);

  // Primary Constants

  /// @brief [m] Radius of the wheel including the tires
  const float64_t wheel_radius;

  /// @brief [m] Horizontal distance between 2 circular sides of the wheel
  const float64_t wheel_width;

  /// @brief [m] Absolute distance between axis centers of front and rear wheels.
  const float64_t wheel_base;

  /// @brief [m] Absolute distance between axis centers of left and right wheels.
  const float64_t wheel_tread;

  /// @brief [m] Absolute distance between the vertical plane passing through the centres of the
  /// front wheels and the foremost point of the vehicle
  const float64_t overhang_front;

  /// @brief [m] Absolute distance between the vertical plane passing through the centres of the
  /// rear wheels and the rearmost point of the vehicle
  const float64_t overhang_rear;

  /// @brief [m] Absolute distance between axis centers of left wheels and the leftmost point of the
  /// vehicle
  const float64_t overhang_left;

  /// @brief [m] Absolute distance between axis centers of right wheels and the rightmost point of
  /// the vehicle
  const float64_t overhang_right;

  /// @brief [m] Absolute vertical distance between ground and topmost point of the vehicle
  /// including mounted sensors
  const float64_t vehicle_height;

  /// @brief [m] Absolute value of longitudinal distance between Center of Gravity and center of
  /// rear axle
  const float64_t cg_to_rear;

  /// @brief [kg/deg] The nominal cornering stiffness is equal to the side force in Newtons divided
  /// by the slip angle in Degrees for small angles for front wheels
  /// @details
  /// https://www.mchenrysoftware.com/medit32/readme/msmac/default.htm?turl=examplestirecorneringstiffnesscalculation1.htm
  const float64_t tire_cornering_stiffness_front;

  /// @brief [kg/deg] The nominal cornering stiffness for rear wheels
  const float64_t tire_cornering_stiffness_rear;

  /// @brief [kg] Total mass of the vehicle including sensors in kilograms
  const float64_t mass_vehicle;

  /// @brief [kg * m^2] Moment of inertia around vertical axis of the vehicle
  const float64_t inertia_yaw_kg_m2;

  /// @brief [rad] Maximum turning angle for cars front axis
  const float64_t maximum_turning_angle_rad;


  // Derived Constants

  /// @brief [m] Absolute value of longitudinal distance between Center of Gravity and center of
  /// front axle
  const float64_t cg_to_front;

  /// @brief [m] Horizontal distance between foremost and rearmost points of the vehicle
  const float64_t vehicle_length;

  /// @brief [m] Horizontal distance between leftmost and rightmost points of the vehicle
  const float64_t vehicle_width;

  /// @brief Offsets from base_link
  ///
  /// These values assume X+ is forward, Y+ is left, Z+ is up

  /// @brief [m] Signed distance from base_link to the rear-most point of the vehicle. (Negative)
  const float64_t offset_longitudinal_min;

  /// @brief [m] Signed distance from base_link to the fore-most point of the vehicle.
  const float64_t offset_longitudinal_max;

  /// @brief [m] Signed distance from base_link to the right-most point of the vehicle. (Negative)
  const float64_t offset_lateral_min;

  /// @brief [m] Signed distance from base_link to the left-most point of the vehicle.
  const float64_t offset_lateral_max;

  /// @brief [m] Signed distance from base_link to the bottom-most point of the vehicle. (Negative)
  const float64_t offset_height_min;

  /// @brief [m] Signed distance from base_link to the top-most point of the vehicle.
  const float64_t offset_height_max;

  /// @brief [rad] Minimum turning radius
  float64_t minimum_turning_radius;

  /// @brief Retrieves a list of vehicle parameters names and values as a string.
  std::string str_pretty() const;
};

/// @brief Declares the vehicle parameters for the node and creates a VehicleConstants object.
/// @details It creates a `rclcpp::SyncParametersClient` object to reach parameters of the
/// `vehicle_constants_manager_node` and attempts to retrieve all required parameters from the node.
/// @throws std::runtime_error if `VehicleConstants` object fails to initialize
/// @throws rclcpp::exceptions::InvalidParameterTypeException if declare_parameter gets a value with
/// wrong type
/// @throws rclcpp::exceptions::InvalidParameterValueException if initial value fails to be set.
/// @return A VehicleConstants object containing vehicle constant parameters.
VEHICLE_CONSTANTS_MANAGER_PUBLIC VehicleConstants
declare_and_get_vehicle_constants(rclcpp::Node & node);
}  // namespace vehicle_constants_manager
}  // namespace common
}  // namespace autoware

#endif  // VEHICLE_CONSTANTS_MANAGER__VEHICLE_CONSTANTS_MANAGER_HPP_
