// Copyright 2018-2019 Autoware Foundation
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
  int dim_x, int dim_u, int dim_y, double wheel_base, double steer_limit)
: dim_x_(dim_x),
  dim_u_(dim_u),
  dim_y_(dim_y),
  wheel_base_(wheel_base),
  steer_limit_(steer_limit),
  center_offset_from_base_(0.0)
{
}

int VehicleModelInterface::getDimX() { return dim_x_; }
int VehicleModelInterface::getDimU() { return dim_u_; }
int VehicleModelInterface::getDimY() { return dim_y_; }

void VehicleModelInterface::updateCenterOffset(const double center_offset_from_base)
{
  center_offset_from_base_ = center_offset_from_base;
}

void VehicleModelInterface::setCurvature(const double curvature) { curvature_ = curvature; }
