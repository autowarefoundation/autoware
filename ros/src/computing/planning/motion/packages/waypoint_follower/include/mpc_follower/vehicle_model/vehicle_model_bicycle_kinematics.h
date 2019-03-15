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
    KinematicsBicycleModel();
    ~KinematicsBicycleModel();

    void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt) override;
    void calculateReferenceInput(Eigen::MatrixXd &Uref) override;
    
    void calculateReferenceInput(Eigen::MatrixXd &Uref, const double &curvature);
    void setParams(double &wheelbase, double &steer_tau, double &steer_lim_deg);
    void setVel(const double &vel);
    void setCurvature(const double &curvature);

  private:
    double wheelbase_;
    double steer_tau_;
    double steer_lim_deg_;

    double vel_;
    double curvature_;
};
