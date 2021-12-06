// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.hpp"


SimModelIdealSteerVel::SimModelIdealSteerVel(float64_t wheelbase)
: SimModelInterface(3 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase) {}

float64_t SimModelIdealSteerVel::getX() {return state_(IDX::X);}
float64_t SimModelIdealSteerVel::getY() {return state_(IDX::Y);}
float64_t SimModelIdealSteerVel::getYaw() {return state_(IDX::YAW);}
float64_t SimModelIdealSteerVel::getVx() {return input_(IDX_U::VX_DES);}
float64_t SimModelIdealSteerVel::getVy() {return 0.0;}
float64_t SimModelIdealSteerVel::getAx() {return current_ax_;}
float64_t SimModelIdealSteerVel::getWz()
{
  return input_(IDX_U::VX_DES) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
}
float64_t SimModelIdealSteerVel::getSteer() {return input_(IDX_U::STEER_DES);}
void SimModelIdealSteerVel::update(const float64_t & dt)
{
  updateRungeKutta(dt, input_);
  current_ax_ = (input_(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = input_(IDX_U::VX_DES);
}

Eigen::VectorXd SimModelIdealSteerVel::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const float64_t yaw = state(IDX::YAW);
  const float64_t vx = input(IDX_U::VX_DES);
  const float64_t steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * std::cos(yaw);
  d_state(IDX::Y) = vx * std::sin(yaw);
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
}
