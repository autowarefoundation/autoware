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

#ifndef VEHICLE_CMD_GATE__VEHICLE_CMD_FILTER_HPP_
#define VEHICLE_CMD_GATE__VEHICLE_CMD_FILTER_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

class VehicleCmdFilter
{
public:
  VehicleCmdFilter();
  ~VehicleCmdFilter() = default;

  void setWheelBase(double v) { wheel_base_ = v; }
  void setVelLim(double v) { vel_lim_ = v; }
  void setLonAccLim(double v) { lon_acc_lim_ = v; }
  void setLonJerkLim(double v) { lon_jerk_lim_ = v; }
  void setLatAccLim(double v) { lat_acc_lim_ = v; }
  void setLatJerkLim(double v) { lat_jerk_lim_ = v; }
  void setPrevCmd(const autoware_auto_control_msgs::msg::AckermannControlCommand & v)
  {
    prev_cmd_ = v;
  }

  void limitLongitudinalWithVel(
    autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLongitudinalWithAcc(
    const double dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLongitudinalWithJerk(
    const double dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLateralWithLatAcc(
    const double dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void limitLateralWithLatJerk(
    const double dt, autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;

private:
  double wheel_base_;
  double vel_lim_;
  double lon_acc_lim_;
  double lon_jerk_lim_;
  double lat_acc_lim_;
  double lat_jerk_lim_;
  autoware_auto_control_msgs::msg::AckermannControlCommand prev_cmd_;

  double calcLatAcc(const autoware_auto_control_msgs::msg::AckermannControlCommand & cmd) const;
  double calcSteerFromLatacc(const double v, const double latacc) const;
  double limitDiff(const double curr, const double prev, const double diff_lim) const;
};

#endif  // VEHICLE_CMD_GATE__VEHICLE_CMD_FILTER_HPP_
