// Copyright 2018-2021 The Autoware Foundation
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

#include "trajectory_follower/vehicle_model/vehicle_model_interface.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u, int dim_y, double wheelbase)
: m_dim_x(dim_x), m_dim_u(dim_u), m_dim_y(dim_y), m_wheelbase(wheelbase)
{
}
int VehicleModelInterface::getDimX() { return m_dim_x; }
int VehicleModelInterface::getDimU() { return m_dim_u; }
int VehicleModelInterface::getDimY() { return m_dim_y; }
double VehicleModelInterface::getWheelbase() { return m_wheelbase; }
void VehicleModelInterface::setVelocity(const double velocity) { m_velocity = velocity; }
void VehicleModelInterface::setCurvature(const double curvature) { m_curvature = curvature; }
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
