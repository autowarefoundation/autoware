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
 * This is the dynamics model of the car
 */

class DynamicsBicycleModel : public VehicleModelInterface
{
  public:
    DynamicsBicycleModel(double &wheelbase, double &steer_lim_deg, double &mass_fl, double &mass_fr, double &mass_rl, double &mass_rr, double &cf, double &cr);
    ~DynamicsBicycleModel();

    void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &B1d, Eigen::MatrixXd &B2d, Eigen::MatrixXd &Cd, double &dt) override;
    void calculateReferenceInput(Eigen::MatrixXd &Uref) override;

  private:
    double wheelbase_;
    double lf_;
    double lr_;
    double steer_lim_deg_;
    double mass_fl_;
    double mass_fr_;
    double mass_rl_;
    double mass_rr_;
    double mass_front_;
    double mass_rear_;
    double mass_;
    double iz_;   //moment of inertia
    double cf_;   //slip angle of the front tire
    double cr_;   //slip angle of the rear tire

}; 
