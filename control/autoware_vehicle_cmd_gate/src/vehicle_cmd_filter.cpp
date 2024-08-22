// Copyright 2015-2019 Autoware Foundation
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

#include "vehicle_cmd_filter.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::vehicle_cmd_gate
{

VehicleCmdFilter::VehicleCmdFilter() : param_()
{
}

bool VehicleCmdFilter::setParameterWithValidation(const VehicleCmdFilterParam & p)
{
  const auto s = p.reference_speed_points.size();
  if (
    p.lon_acc_lim.size() != s || p.lon_jerk_lim.size() != s || p.lat_acc_lim.size() != s ||
    p.lat_jerk_lim.size() != s || p.actual_steer_diff_lim.size() != s || p.steer_lim.size() != s ||
    p.steer_rate_lim.size() != s) {
    std::cerr << "VehicleCmdFilter::setParam() There is a size mismatch in the parameter. "
                 "Parameter initialization failed."
              << std::endl;
    return false;
  }

  param_ = p;
  return true;
}

void VehicleCmdFilter::setParam(const VehicleCmdFilterParam & p)
{
  if (!setParameterWithValidation(p)) {
    std::exit(EXIT_FAILURE);
  }
}

VehicleCmdFilterParam VehicleCmdFilter::getParam() const
{
  return param_;
}

void VehicleCmdFilter::limitLongitudinalWithVel(Control & input) const
{
  input.longitudinal.velocity = std::max(
    std::min(static_cast<double>(input.longitudinal.velocity), param_.vel_lim), -param_.vel_lim);
}

void VehicleCmdFilter::limitLongitudinalWithAcc(const double dt, Control & input) const
{
  const auto lon_acc_lim = getLonAccLim();
  input.longitudinal.acceleration = std::max(
    std::min(static_cast<double>(input.longitudinal.acceleration), lon_acc_lim), -lon_acc_lim);
  input.longitudinal.velocity =
    limitDiff(input.longitudinal.velocity, prev_cmd_.longitudinal.velocity, lon_acc_lim * dt);
}

void VehicleCmdFilter::VehicleCmdFilter::limitLongitudinalWithJerk(
  const double dt, Control & input) const
{
  const auto lon_jerk_lim = getLonJerkLim();
  input.longitudinal.acceleration = limitDiff(
    input.longitudinal.acceleration, prev_cmd_.longitudinal.acceleration, lon_jerk_lim * dt);
  input.longitudinal.jerk =
    std::clamp(static_cast<double>(input.longitudinal.jerk), -lon_jerk_lim, lon_jerk_lim);
}

// Use ego vehicle speed (not speed command) for the lateral acceleration calculation, otherwise the
// filtered steering angle oscillates if the input velocity oscillates.
void VehicleCmdFilter::limitLateralWithLatAcc(
  [[maybe_unused]] const double dt, Control & input) const
{
  const auto lat_acc_lim = getLatAccLim();

  double latacc = calcLatAcc(input, current_speed_);
  if (std::fabs(latacc) > lat_acc_lim) {
    double v_sq = std::max(static_cast<double>(current_speed_ * current_speed_), 0.001);
    double steer_lim = std::atan(lat_acc_lim * param_.wheel_base / v_sq);
    input.lateral.steering_tire_angle = latacc > 0.0 ? steer_lim : -steer_lim;
  }
}

// Use ego vehicle speed (not speed command) for the lateral acceleration calculation, otherwise the
// filtered steering angle oscillates if the input velocity oscillates.
void VehicleCmdFilter::limitLateralWithLatJerk(const double dt, Control & input) const
{
  double curr_latacc = calcLatAcc(input, current_speed_);
  double prev_latacc = calcLatAcc(prev_cmd_, current_speed_);

  const auto lat_jerk_lim = getLatJerkLim();

  const double latacc_max = prev_latacc + lat_jerk_lim * dt;
  const double latacc_min = prev_latacc - lat_jerk_lim * dt;

  if (curr_latacc > latacc_max) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(current_speed_, latacc_max);
  } else if (curr_latacc < latacc_min) {
    input.lateral.steering_tire_angle = calcSteerFromLatacc(current_speed_, latacc_min);
  }
}

void VehicleCmdFilter::limitActualSteerDiff(const double current_steer_angle, Control & input) const
{
  const auto actual_steer_diff_lim = getSteerDiffLim();

  auto ds = input.lateral.steering_tire_angle - current_steer_angle;
  ds = std::clamp(ds, -actual_steer_diff_lim, actual_steer_diff_lim);
  input.lateral.steering_tire_angle = current_steer_angle + ds;
}

void VehicleCmdFilter::limitLateralSteer(Control & input) const
{
  const float steer_limit = getSteerLim();

  input.lateral.steering_tire_angle =
    std::clamp(input.lateral.steering_tire_angle, -steer_limit, steer_limit);

  // TODO(Horibe): support steering greater than PI/2. Now the lateral acceleration
  // calculation does not support bigger steering value than PI/2 due to tan/atan calculation.
  if (std::abs(input.lateral.steering_tire_angle) > M_PI_2f) {
    std::cerr << "[vehicle_Cmd_gate] limitLateralSteer(): steering limit is set to pi/2 since the "
                 "current filtering logic can not handle the steering larger than pi/2. Please "
                 "check the steering angle limit."
              << std::endl;

    std::clamp(input.lateral.steering_tire_angle, -M_PI_2f, M_PI_2f);
  }
}

void VehicleCmdFilter::limitLateralSteerRate(const double dt, Control & input) const
{
  const float steer_rate_limit = getSteerRateLim();

  // for steering angle rate
  input.lateral.steering_tire_rotation_rate =
    std::clamp(input.lateral.steering_tire_rotation_rate, -steer_rate_limit, steer_rate_limit);

  // for steering angle
  const float steer_diff_limit = steer_rate_limit * dt;
  float ds = input.lateral.steering_tire_angle - prev_cmd_.lateral.steering_tire_angle;
  ds = std::clamp(ds, -steer_diff_limit, steer_diff_limit);
  input.lateral.steering_tire_angle = prev_cmd_.lateral.steering_tire_angle + ds;
}

void VehicleCmdFilter::filterAll(
  const double dt, const double current_steer_angle, Control & cmd,
  IsFilterActivated & is_activated) const
{
  const auto cmd_orig = cmd;
  limitLateralSteer(cmd);
  limitLateralSteerRate(dt, cmd);
  limitLongitudinalWithJerk(dt, cmd);
  limitLongitudinalWithAcc(dt, cmd);
  limitLongitudinalWithVel(cmd);
  limitLateralWithLatJerk(dt, cmd);
  limitLateralWithLatAcc(dt, cmd);
  limitActualSteerDiff(current_steer_angle, cmd);

  is_activated = checkIsActivated(cmd, cmd_orig);
  return;
}

IsFilterActivated VehicleCmdFilter::checkIsActivated(
  const Control & c1, const Control & c2, const double tol)
{
  IsFilterActivated msg;
  msg.is_activated_on_steering =
    std::abs(c1.lateral.steering_tire_angle - c2.lateral.steering_tire_angle) > tol;
  msg.is_activated_on_steering_rate =
    std::abs(c1.lateral.steering_tire_rotation_rate - c2.lateral.steering_tire_rotation_rate) > tol;
  msg.is_activated_on_speed = std::abs(c1.longitudinal.velocity - c2.longitudinal.velocity) > tol;
  msg.is_activated_on_acceleration =
    std::abs(c1.longitudinal.acceleration - c2.longitudinal.acceleration) > tol;
  msg.is_activated_on_jerk = std::abs(c1.longitudinal.jerk - c2.longitudinal.jerk) > tol;

  msg.is_activated =
    (msg.is_activated_on_steering || msg.is_activated_on_steering_rate ||
     msg.is_activated_on_speed || msg.is_activated_on_acceleration || msg.is_activated_on_jerk);

  return msg;
}

double VehicleCmdFilter::calcSteerFromLatacc(const double v, const double latacc) const
{
  const double v_sq = std::max(v * v, 0.001);
  return std::atan(latacc * param_.wheel_base / v_sq);
}

double VehicleCmdFilter::calcLatAcc(const Control & cmd) const
{
  double v = cmd.longitudinal.velocity;
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / param_.wheel_base;
}

double VehicleCmdFilter::calcLatAcc(const Control & cmd, const double v) const
{
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / param_.wheel_base;
}

double VehicleCmdFilter::limitDiff(const double curr, const double prev, const double diff_lim)
{
  double diff = std::max(std::min(curr - prev, diff_lim), -diff_lim);
  return prev + diff;
}

double VehicleCmdFilter::interpolateFromSpeed(const LimitArray & limits) const
{
  // Consider only for the positive velocities.
  const auto current = std::abs(current_speed_);
  const auto reference = param_.reference_speed_points;

  // If the speed is out of range of the reference, apply zero-order hold.
  if (current <= reference.front()) {
    return limits.front();
  }
  if (current >= reference.back()) {
    return limits.back();
  }

  // Apply linear interpolation
  for (size_t i = 0; i < reference.size() - 1; ++i) {
    if (reference.at(i) <= current && current <= reference.at(i + 1)) {
      auto ratio =
        (current - reference.at(i)) / std::max(reference.at(i + 1) - reference.at(i), 1.0e-5);
      ratio = std::clamp(ratio, 0.0, 1.0);
      const auto interp = limits.at(i) + ratio * (limits.at(i + 1) - limits.at(i));
      return interp;
    }
  }

  std::cerr << "VehicleCmdFilter::interpolateFromSpeed() interpolation logic is broken. Command "
               "filter is not working. Please check the code."
            << std::endl;
  return reference.back();
}

double VehicleCmdFilter::getLonAccLim() const
{
  return interpolateFromSpeed(param_.lon_acc_lim);
}
double VehicleCmdFilter::getLonJerkLim() const
{
  return interpolateFromSpeed(param_.lon_jerk_lim);
}
double VehicleCmdFilter::getLatAccLim() const
{
  return interpolateFromSpeed(param_.lat_acc_lim);
}
double VehicleCmdFilter::getLatJerkLim() const
{
  return interpolateFromSpeed(param_.lat_jerk_lim);
}
double VehicleCmdFilter::getSteerLim() const
{
  return interpolateFromSpeed(param_.steer_lim);
}
double VehicleCmdFilter::getSteerRateLim() const
{
  return interpolateFromSpeed(param_.steer_rate_lim);
}
double VehicleCmdFilter::getSteerDiffLim() const
{
  return interpolateFromSpeed(param_.actual_steer_diff_lim);
}

}  // namespace autoware::vehicle_cmd_gate
