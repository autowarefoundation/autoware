// Copyright 2023 TIER IV, Inc.
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

#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_interface.hpp"

VehicleModelInterface::VehicleModelInterface(
  const int dim_x, const int dim_u, const int dim_y, const double wheelbase,
  const double steer_limit)
: dim_x_(dim_x), dim_u_(dim_u), dim_y_(dim_y), wheelbase_(wheelbase), steer_limit_(steer_limit)
{
}

int VehicleModelInterface::getDimX() const
{
  return dim_x_;
}
int VehicleModelInterface::getDimU() const
{
  return dim_u_;
}
int VehicleModelInterface::getDimY() const
{
  return dim_y_;
}
