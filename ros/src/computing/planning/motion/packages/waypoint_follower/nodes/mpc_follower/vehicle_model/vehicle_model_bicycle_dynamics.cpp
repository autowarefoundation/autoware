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

#include "mpc_follower/vehicle_model/vehicle_model_bicycle_dynamics.h"

DynamicsBicycleModel::DynamicsBicycleModel(double &wheelbase, double &mass_fl, double &mass_fr,
                                           double &mass_rl, double &mass_rr, double &cf, double &cr)
    : VehicleModelInterface(/* dim_x */ 4, /* dim_u */ 1, /* dim_y */ 2)
{
    wheelbase_ = wheelbase;

    const double mass_front = mass_fl + mass_fr;
    const double mass_rear = mass_rl + mass_rr;

    mass_ = mass_front + mass_rear;
    lf_ = wheelbase_ * (1.0 - mass_front / mass_);
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
    cf_ = cf;
    cr_ = cr;
};

DynamicsBicycleModel::~DynamicsBicycleModel(){};

void DynamicsBicycleModel::calculateDiscreteMatrix(Eigen::MatrixXd &Ad,
                                                   Eigen::MatrixXd &Bd,
                                                   Eigen::MatrixXd &Cd,
                                                   Eigen::MatrixXd &Wd,
                                                   const double &dt)
{
    /*
     * x[k+1] = Ad*x[k] + Bd*u + Wd
     */

    const double vel = std::max(velocity_, 0.01);

    Ad = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
    Ad(0, 1) = 1.0;
    Ad(1, 1) = -(cf_ + cr_) / (mass_ * vel);
    Ad(1, 2) = (cf_ + cr_) / mass_;
    Ad(1, 3) = (lr_ * cr_ - lf_ * cf_) / (mass_ * vel);
    Ad(2, 3) = 1.0;
    Ad(3, 1) = (lr_ * cr_ - lf_ * cf_) / (iz_ * vel);
    Ad(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    Ad(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (iz_ * vel);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    Eigen::MatrixXd Ad_inverse = (I - dt * 0.5 * Ad).inverse();

    Ad = Ad_inverse * (I + dt * 0.5 * Ad); // bilinear discretization

    Bd = Eigen::MatrixXd::Zero(dim_x_, dim_u_);
    Bd(0, 0) = 0.0;
    Bd(1, 0) = cf_ / mass_;
    Bd(2, 0) = 0.0;
    Bd(3, 0) = lf_ * cf_ / iz_;

    Wd = Eigen::MatrixXd::Zero(dim_x_, 1);
    Wd(0, 0) = 0.0;
    Wd(1, 0) = (lr_ * cr_ - lf_ * cf_) / (mass_ * vel) - vel;
    Wd(2, 0) = 0.0;
    Wd(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (iz_ * vel);

    Bd = (Ad_inverse * dt) * Bd;
    Wd = (Ad_inverse * dt * curvature_ * vel) * Wd;

    Cd = Eigen::MatrixXd::Zero(dim_y_, dim_x_);
    Cd(0, 0) = 1.0;
    Cd(1, 2) = 1.0;
}

void DynamicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd &Uref)
{
    const double vel = std::max(velocity_, 0.01);
    const double Kv = lr_ * mass_ / (2 * cf_ * wheelbase_) - lf_ * mass_ / (2 * cr_ * wheelbase_);
    Uref(0, 0) = wheelbase_ * curvature_ + Kv * vel * vel * curvature_;
}
