/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mpc_follower/vehicle_model/vehicle_model_interface.h"

VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u, int dim_y) : dim_x_(dim_x), dim_u_(dim_u), dim_y_(dim_y) {};
int VehicleModelInterface::getDimX() { return dim_x_; };
int VehicleModelInterface::getDimU() { return dim_u_; };
int VehicleModelInterface::getDimY() { return dim_y_; };
void VehicleModelInterface::setVelocity(const double &velocity) { velocity_ = velocity; };
void VehicleModelInterface::setCurvature(const double &curvature) { curvature_ = curvature; };
