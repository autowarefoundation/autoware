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

#ifndef VEHICLE_CMD_FILTER_HPP_
#define VEHICLE_CMD_FILTER_HPP_

#include <autoware_vehicle_cmd_gate/msg/is_filter_activated.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>

#include <vector>

namespace autoware::vehicle_cmd_gate
{
using autoware_control_msgs::msg::Control;
using autoware_vehicle_cmd_gate::msg::IsFilterActivated;
using LimitArray = std::vector<double>;

struct VehicleCmdFilterParam
{
  double wheel_base;
  double vel_lim;
  LimitArray reference_speed_points;
  LimitArray lon_acc_lim;
  LimitArray lon_jerk_lim;
  LimitArray lat_acc_lim;
  LimitArray lat_jerk_lim;
  LimitArray steer_lim;
  LimitArray steer_rate_lim;
  LimitArray actual_steer_diff_lim;
};
class VehicleCmdFilter
{
public:
  VehicleCmdFilter();
  ~VehicleCmdFilter() = default;

  void setWheelBase(double v) { param_.wheel_base = v; }
  void setCurrentSpeed(double v) { current_speed_ = v; }
  void setParam(const VehicleCmdFilterParam & p);
  VehicleCmdFilterParam getParam() const;
  void setPrevCmd(const Control & v) { prev_cmd_ = v; }

  void limitLongitudinalWithVel(Control & input) const;
  void limitLongitudinalWithAcc(const double dt, Control & input) const;
  void limitLongitudinalWithJerk(const double dt, Control & input) const;
  void limitLateralWithLatAcc(const double dt, Control & input) const;
  void limitLateralWithLatJerk(const double dt, Control & input) const;
  void limitActualSteerDiff(const double current_steer_angle, Control & input) const;
  void limitLateralSteer(Control & input) const;
  void limitLateralSteerRate(const double dt, Control & input) const;
  void filterAll(
    const double dt, const double current_steer_angle, Control & input,
    IsFilterActivated & is_activated) const;
  static IsFilterActivated checkIsActivated(
    const Control & c1, const Control & c2, const double tol = 1.0e-3);

private:
  VehicleCmdFilterParam param_;
  Control prev_cmd_;
  double current_speed_ = 0.0;

  bool setParameterWithValidation(const VehicleCmdFilterParam & p);

  double calcLatAcc(const Control & cmd) const;
  double calcLatAcc(const Control & cmd, const double v) const;
  double calcSteerFromLatacc(const double v, const double latacc) const;
  static double limitDiff(const double curr, const double prev, const double diff_lim);

  double interpolateFromSpeed(const LimitArray & limits) const;
  double getLonAccLim() const;
  double getLonJerkLim() const;
  double getLatAccLim() const;
  double getLatJerkLim() const;
  double getSteerLim() const;
  double getSteerRateLim() const;
  double getSteerDiffLim() const;
};
}  // namespace autoware::vehicle_cmd_gate

#endif  // VEHICLE_CMD_FILTER_HPP_
