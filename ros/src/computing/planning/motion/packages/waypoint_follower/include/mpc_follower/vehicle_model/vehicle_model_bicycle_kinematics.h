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
#include <eigen3/Eigen/LU>
#include "mpc_follower/vehicle_model/vehicle_model_interface.h"

/* 
   The trajectory following error dynamics of the vehicle.
   This is valid in the vicinity of the target trajectory.
*/

class KinematicsBicycleModel : public VehicleModelInterface
{
public:
  KinematicsBicycleModel(const double &wheelbase_, const double &steer_lim_deg, const double &steer_tau);
  ~KinematicsBicycleModel();

  void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt) override;
  void calculateReferenceInput(Eigen::MatrixXd &Uref) override;

private:
  double wheelbase_;
  double steer_lim_deg_;
  double steer_tau_;
};
