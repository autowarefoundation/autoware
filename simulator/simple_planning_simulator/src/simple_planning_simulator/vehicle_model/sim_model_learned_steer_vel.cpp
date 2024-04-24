// Copyright 2024 The Autoware Foundation.
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

#include "simple_planning_simulator/vehicle_model/sim_model_learned_steer_vel.hpp"

#include "learning_based_vehicle_model/interconnected_model.hpp"

#include <algorithm>

SimModelLearnedSteerVel::SimModelLearnedSteerVel(
  double dt, std::vector<std::string> model_python_paths,
  std::vector<std::string> model_param_paths, std::vector<std::string> model_class_names)
: SimModelInterface(7 /* dim x */, 2 /* dim u */)
{
  for (size_t i = 0; i < model_python_paths.size(); i++) {
    std::tuple<std::string, std::string, std::string> descriptor = {
      model_python_paths[i], model_param_paths[i], model_class_names[i]};
    vehicle.addSubmodel(descriptor);
  }

  vehicle.generateConnections(input_names, state_names);

  vehicle.dtSet(dt);
}

double SimModelLearnedSteerVel::getX()
{
  return state_(IDX::X);
}
double SimModelLearnedSteerVel::getY()
{
  return state_(IDX::Y);
}
double SimModelLearnedSteerVel::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelLearnedSteerVel::getVx()
{
  return state_(IDX::VX);
}
double SimModelLearnedSteerVel::getVy()
{
  return state_(IDX::VY);
}
double SimModelLearnedSteerVel::getAx()
{
  return current_ax_;
}
double SimModelLearnedSteerVel::getWz()
{
  return state_(IDX::YAW_RATE);
}
double SimModelLearnedSteerVel::getSteer()
{
  return state_(IDX::STEER);
}
void SimModelLearnedSteerVel::update(const double & dt)
{
  // Eigen::VectorXd to std::vector<double> for model input
  std::vector<double> vehicle_input_(input_.data(), input_.data() + input_.size());

  // Eigen::VectorXd to std::vector<double> for model state
  std::vector<double> new_state(state_.data(), state_.data() + state_.size());
  // set model state
  vehicle.initState(new_state);

  // model forward
  std::vector<double> vehicle_state_ = vehicle.updatePyModel(vehicle_input_);

  // std::vector<double> to Eigen::VectorXd
  for (size_t i = 0; i < vehicle_state_.size(); i++) state_[i] = vehicle_state_[i];

  // Calculate
  current_ax_ = (input_(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = input_(IDX_U::VX_DES);
}
