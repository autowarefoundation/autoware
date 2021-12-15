// Copyright 2019 Christopher Ho
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

#include "motion_common/config.hpp"

#include <limits>
#include <stdexcept>

namespace motion
{
namespace motion_common
{

LimitsConfig::Extremum::Extremum(Real min, Real max)
: m_min{min}, m_max{max}
{
  if (min >= max - std::numeric_limits<decltype(max)>::epsilon()) {
    throw std::domain_error{"Extremum: min >= max - epsilon"};
  }
}

Real LimitsConfig::Extremum::min() const noexcept
{
  return m_min;
}
Real LimitsConfig::Extremum::max() const noexcept
{
  return m_max;
}

////////////////////////////////////////////////////////////////////////////////
LimitsConfig::LimitsConfig(
  Extremum longitudinal_velocity_mps,
  Extremum lateral_velocity_mps,
  Extremum acceleration_mps2,
  Extremum yaw_rate_rps,
  Extremum jerk_mps3,
  Extremum steer_angle_rad,
  Extremum steer_angle_rate_rps)
: m_longitudinal_velocity_limits_mps{longitudinal_velocity_mps},
  m_lateral_velocity_limits_mps{lateral_velocity_mps},
  m_acceleration_limits_mps2{acceleration_mps2},
  m_yaw_rate_limits_rps{yaw_rate_rps},
  m_jerk_limits_mps3{jerk_mps3},
  m_steer_angle_limits_rad{steer_angle_rad},
  m_steer_angle_rate_limits_rps{steer_angle_rate_rps}
{
}

LimitsConfig::Extremum LimitsConfig::longitudinal_velocity() const noexcept
{
  return m_longitudinal_velocity_limits_mps;
}
LimitsConfig::Extremum LimitsConfig::lateral_velocity() const noexcept
{
  return m_lateral_velocity_limits_mps;
}
LimitsConfig::Extremum LimitsConfig::acceleration() const noexcept
{
  return m_acceleration_limits_mps2;
}
LimitsConfig::Extremum LimitsConfig::jerk() const noexcept
{
  return m_jerk_limits_mps3;
}
LimitsConfig::Extremum LimitsConfig::steer_angle() const noexcept
{
  return m_steer_angle_limits_rad;
}
LimitsConfig::Extremum LimitsConfig::steer_angle_rate() const noexcept
{
  return m_steer_angle_rate_limits_rps;
}
LimitsConfig::Extremum LimitsConfig::yaw_rate() const noexcept
{
  return m_yaw_rate_limits_rps;
}

////////////////////////////////////////////////////////////////////////////////
VehicleConfig::VehicleConfig(
  Real length_cg_front_axel_m,
  Real length_cg_rear_axel_m,
  Real front_cornering_stiffness_N,
  Real rear_cornering_stiffness_N,
  Real mass_kg,
  Real inertia_kgm2,
  Real width_m,
  Real front_overhang_m,
  Real rear_overhang_m)
: m_length_cg_to_front_axel_m{length_cg_front_axel_m},
  m_length_cg_to_rear_axel_m{length_cg_rear_axel_m},
  m_front_cornering_stiffness_N{front_cornering_stiffness_N},
  m_rear_cornering_stiffness_N{rear_cornering_stiffness_N},
  m_mass_kg{mass_kg},
  m_inertia_kgm2{inertia_kgm2},
  m_width_m{width_m},
  m_front_overhang_m{front_overhang_m},
  m_rear_overhang_m{rear_overhang_m}
{
}

Real VehicleConfig::length_cg_front_axel() const noexcept
{
  return m_length_cg_to_front_axel_m;
}
Real VehicleConfig::length_cg_rear_axel() const noexcept
{
  return m_length_cg_to_rear_axel_m;
}
Real VehicleConfig::front_cornering_stiffness() const noexcept
{
  return m_front_cornering_stiffness_N;
}
Real VehicleConfig::rear_cornering_stiffness() const noexcept
{
  return m_rear_cornering_stiffness_N;
}
Real VehicleConfig::mass() const noexcept
{
  return m_mass_kg;
}
Real VehicleConfig::inertia() const noexcept
{
  return m_inertia_kgm2;
}
Real VehicleConfig::width() const noexcept
{
  return m_width_m;
}
Real VehicleConfig::front_overhang() const noexcept
{
  return m_front_overhang_m;
}
Real VehicleConfig::rear_overhang() const noexcept
{
  return m_rear_overhang_m;
}

////////////////////////////////////////////////////////////////////////////////
StateWeight::StateWeight(
  Real pose,
  Real heading,
  Real longitudinal_velocity,
  Real lateral_velocity,
  Real yaw_rate,
  Real acceleration,
  Real jerk,
  Real steer_angle,
  Real steer_angle_rate)
: m_pose_weight{pose},
  m_heading_weight{heading},
  m_longitudinal_velocity_weight{longitudinal_velocity},
  m_lateral_velocity_weight{lateral_velocity},
  m_yaw_rate_weight{yaw_rate},
  m_acceleration_weight{acceleration},
  m_jerk_weight{jerk},
  m_steer_angle_weight{steer_angle},
  m_steer_angle_rate_weight{steer_angle_rate}
{
  if (pose < Real{}) {  // zero initialization
    throw std::domain_error{"Pose weight is negative!"};
  }
  if (heading < Real{}) {  // zero initialization
    throw std::domain_error{"Heading weight is negative!"};
  }
  if (longitudinal_velocity < Real{}) {  // zero initialization
    throw std::domain_error{"Longitudinal weight is negative!"};
  }
  if (lateral_velocity < Real{}) {  // zero initialization
    throw std::domain_error{"Lateral velocity weight is negative!"};
  }
  if (yaw_rate < Real{}) {  // zero initialization
    throw std::domain_error{"Yaw rate weight is negative!"};
  }
  if (acceleration < Real{}) {  // zero initialization
    throw std::domain_error{"Acceleration weight is negative!"};
  }
  if (jerk < Real{}) {  // zero initialization
    throw std::domain_error{"Jerk weight is negative!"};
  }
  if (steer_angle < Real{}) {  // zero initialization
    throw std::domain_error{"Steer angle weight is negative!"};
  }
  if (steer_angle_rate < Real{}) {  // zero initialization
    throw std::domain_error{"Steer angle rate weight is negative!"};
  }
}

Real StateWeight::pose() const noexcept
{
  return m_pose_weight;
}

Real StateWeight::heading() const noexcept
{
  return m_heading_weight;
}

Real StateWeight::longitudinal_velocity() const noexcept
{
  return m_longitudinal_velocity_weight;
}

Real StateWeight::lateral_velocity() const noexcept
{
  return m_lateral_velocity_weight;
}

Real StateWeight::yaw_rate() const noexcept
{
  return m_yaw_rate_weight;
}

Real StateWeight::acceleration() const noexcept
{
  return m_acceleration_weight;
}

Real StateWeight::steer_angle() const noexcept
{
  return m_steer_angle_weight;
}

Real StateWeight::steer_angle_rate() const noexcept
{
  return m_steer_angle_rate_weight;
}

Real StateWeight::jerk() const noexcept
{
  return m_jerk_weight;
}

////////////////////////////////////////////////////////////////////////////////
OptimizationConfig::OptimizationConfig(
  StateWeight nominal_weights,
  StateWeight terminal_weights)
: m_nominal_weights{nominal_weights},
  m_terminal_weights{terminal_weights}
{
}

StateWeight OptimizationConfig::nominal() const noexcept
{
  return m_nominal_weights;
}

StateWeight OptimizationConfig::terminal() const noexcept
{
  return m_terminal_weights;
}
}  // namespace motion_common
}  // namespace motion
