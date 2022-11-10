// Copyright 2021 The Autoware Foundation.
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

#include "simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.hpp"

SimModelIdealSteerVel::SimModelIdealSteerVel(double wheelbase)
: SimModelInterface(3 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase)
{
}

double SimModelIdealSteerVel::getX() { return state_(IDX::X); }
double SimModelIdealSteerVel::getY() { return state_(IDX::Y); }
double SimModelIdealSteerVel::getYaw() { return state_(IDX::YAW); }
double SimModelIdealSteerVel::getVx() { return input_(IDX_U::VX_DES); }
double SimModelIdealSteerVel::getVy() { return 0.0; }
double SimModelIdealSteerVel::getAx() { return current_ax_; }
double SimModelIdealSteerVel::getWz()
{
  return input_(IDX_U::VX_DES) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
}
double SimModelIdealSteerVel::getSteer() { return input_(IDX_U::STEER_DES); }
void SimModelIdealSteerVel::update(const double & dt)
{
  updateRungeKutta(dt, input_);
  current_ax_ = (input_(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = input_(IDX_U::VX_DES);
}

Eigen::VectorXd SimModelIdealSteerVel::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double yaw = state(IDX::YAW);
  const double vx = input(IDX_U::VX_DES);
  const double steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * std::cos(yaw);
  d_state(IDX::Y) = vx * std::sin(yaw);
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
}
