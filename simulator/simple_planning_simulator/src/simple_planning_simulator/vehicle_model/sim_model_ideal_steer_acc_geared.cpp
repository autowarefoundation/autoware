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

#include "simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.hpp"

#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>

SimModelIdealSteerAccGeared::SimModelIdealSteerAccGeared(double wheelbase)
: SimModelInterface(4 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase), current_acc_(0.0)
{
}

double SimModelIdealSteerAccGeared::getX() { return state_(IDX::X); }
double SimModelIdealSteerAccGeared::getY() { return state_(IDX::Y); }
double SimModelIdealSteerAccGeared::getYaw() { return state_(IDX::YAW); }
double SimModelIdealSteerAccGeared::getVx() { return state_(IDX::VX); }
double SimModelIdealSteerAccGeared::getVy() { return 0.0; }
double SimModelIdealSteerAccGeared::getAx() { return current_acc_; }
double SimModelIdealSteerAccGeared::getWz()
{
  return state_(IDX::VX) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
}
double SimModelIdealSteerAccGeared::getSteer() { return input_(IDX_U::STEER_DES); }
void SimModelIdealSteerAccGeared::update(const double & dt)
{
  const auto prev_state = state_;
  updateRungeKutta(dt, input_);

  // consider gear
  // update position and velocity first, and then acceleration is calculated naturally
  updateStateWithGear(state_, prev_state, gear_, dt);
}

Eigen::VectorXd SimModelIdealSteerAccGeared::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double vx = state(IDX::VX);
  const double yaw = state(IDX::YAW);
  const double ax = input(IDX_U::AX_DES);
  const double steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * std::cos(yaw);
  d_state(IDX::Y) = vx * std::sin(yaw);
  d_state(IDX::VX) = ax;
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
}

void SimModelIdealSteerAccGeared::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
    }
  } else if (gear == GearCommand::PARK) {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
  } else {
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
  }

  current_acc_ = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
}
