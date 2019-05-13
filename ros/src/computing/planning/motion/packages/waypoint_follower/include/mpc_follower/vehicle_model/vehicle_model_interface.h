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

#pragma once
#include <eigen3/Eigen/Core>

class VehicleModelInterface
{
protected:
  const int dim_x_;
  const int dim_u_;
  const int dim_y_;
  double velocity_;
  double curvature_;

public:
  VehicleModelInterface(int dim_x, int dim_u, int dim_y);
  int getDimX();
  int getDimU();
  int getDimY();
  void setVelocity(const double &velocity);
  void setCurvature(const double &curvature);
  virtual void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt) = 0;
  virtual void calculateReferenceInput(Eigen::MatrixXd &Uref) = 0;
};
