
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

#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.h"

KinematicsBicycleModelNoDelay::KinematicsBicycleModelNoDelay(const double &wheelbase, const double &steer_lim)
    : VehicleModelInterface(/* dim_x */ 2, /* dim_u */ 1, /* dim_y */ 2)
{
    wheelbase_ = wheelbase;
    steer_lim_ = steer_lim;
};
KinematicsBicycleModelNoDelay::~KinematicsBicycleModelNoDelay(){};
void KinematicsBicycleModelNoDelay::calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd,
                                                     Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, const double &dt)
{
    auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

    /* Linearize delta around delta_r (referece delta) */
    double delta_r = atan(wheelbase_ * curvature_);
    if (abs(delta_r) >= steer_lim_)
        delta_r = steer_lim_ * (double)sign(delta_r);
    double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

    Ad << 0.0, velocity_,
          0.0, 0.0;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    Ad = I + Ad * dt;

    Bd << 0.0, velocity_ / wheelbase_ * cos_delta_r_squared_inv;
    Bd *= dt;

    Cd << 1.0, 0.0,
          0.0, 1.0;

    Wd << 0.0,
        -velocity_ / wheelbase_ * delta_r * cos_delta_r_squared_inv;
    Wd *= dt;
}

void KinematicsBicycleModelNoDelay::calculateReferenceInput(Eigen::MatrixXd &Uref)
{
    Uref(0, 0) = std::atan(wheelbase_ * curvature_);
}