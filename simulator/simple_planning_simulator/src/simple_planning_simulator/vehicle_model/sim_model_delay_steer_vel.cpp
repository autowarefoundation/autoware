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

#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel.hpp"

#include <algorithm>

SimModelDelaySteerVel::SimModelDelaySteerVel(
  float64_t vx_lim, float64_t steer_lim, float64_t vx_rate_lim, float64_t steer_rate_lim,
  float64_t wheelbase, float64_t dt, float64_t vx_delay, float64_t vx_time_constant,
  float64_t steer_delay, float64_t steer_time_constant)
: SimModelInterface(5 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  vx_delay_(vx_delay),
  vx_time_constant_(std::max(vx_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT))
{
  initializeInputQueue(dt);
}

float64_t SimModelDelaySteerVel::getX() { return state_(IDX::X); }
float64_t SimModelDelaySteerVel::getY() { return state_(IDX::Y); }
float64_t SimModelDelaySteerVel::getYaw() { return state_(IDX::YAW); }
float64_t SimModelDelaySteerVel::getVx() { return state_(IDX::VX); }
float64_t SimModelDelaySteerVel::getVy() {return 0.0;}
float64_t SimModelDelaySteerVel::getAx() {return current_ax_;}
float64_t SimModelDelaySteerVel::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
float64_t SimModelDelaySteerVel::getSteer() {return state_(IDX::STEER);}
void SimModelDelaySteerVel::update(const float64_t & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  vx_input_queue_.push_back(input_(IDX_U::VX_DES));
  delayed_input(IDX_U::VX_DES) = vx_input_queue_.front();
  vx_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();
  // do not use deadzone_delta_steer (Steer IF does not exist in this model)
  updateRungeKutta(dt, delayed_input);
  current_ax_ = (input_(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = input_(IDX_U::VX_DES);
}

void SimModelDelaySteerVel::initializeInputQueue(const float64_t & dt)
{
  size_t vx_input_queue_size = static_cast<size_t>(round(vx_delay_ / dt));
  for (size_t i = 0; i < vx_input_queue_size; i++) {
    vx_input_queue_.push_back(0.0);
  }
  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  for (size_t i = 0; i < steer_input_queue_size; i++) {
    steer_input_queue_.push_back(0.0);
  }
}


Eigen::VectorXd SimModelDelaySteerVel::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](float64_t val, float64_t u, float64_t l) {return std::max(std::min(val, u), l);};

  const float64_t vx = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const float64_t steer = sat(state(IDX::STEER), steer_lim_, -steer_lim_);
  const float64_t yaw = state(IDX::YAW);
  const float64_t delay_input_vx = input(IDX_U::VX_DES);
  const float64_t delay_input_steer = input(IDX_U::STEER_DES);
  const float64_t delay_vx_des = sat(delay_input_vx, vx_lim_, -vx_lim_);
  const float64_t delay_steer_des = sat(delay_input_steer, steer_lim_, -steer_lim_);
  float64_t vx_rate = -(vx - delay_vx_des) / vx_time_constant_;
  float64_t steer_rate = -(steer - delay_steer_des) / steer_time_constant_;
  vx_rate = sat(vx_rate, vx_rate_lim_, -vx_rate_lim_);
  steer_rate = sat(steer_rate, steer_rate_lim_, -steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;
  d_state(IDX::VX) = vx_rate;
  d_state(IDX::STEER) = steer_rate;

  return d_state;
}

