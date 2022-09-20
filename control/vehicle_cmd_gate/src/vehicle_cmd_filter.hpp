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

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

struct VehicleCmdFilterParam
{
  double wheel_base;
  double vel_lim;
  double lon_acc_lim;
  double lon_jerk_lim;
  double lat_acc_lim;
  double lat_jerk_lim;
  double actual_steer_diff_lim;
};
class VehicleCmdFilter
{
public:
  VehicleCmdFilter();
  ~VehicleCmdFilter() = default;

  void setWheelBase(double v) { param_.wheel_base = v; }
  void setVelLim(double v) { param_.vel_lim = v; }
  void setLonAccLim(double v) { param_.lon_acc_lim = v; }
  void setLonJerkLim(double v) { param_.lon_jerk_lim = v; }
  void setLatAccLim(double v) { param_.lat_acc_lim = v; }
  void setLatJerkLim(double v) { param_.lat_jerk_lim = v; }
  void setActualSteerDiffLim(double v) { param_.actual_steer_diff_lim = v; }
  void setParam(const VehicleCmdFilterParam & p) { param_ = p; }
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
  void limitActualSteerDiff(
    const double current_steer_angle,
    autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;
  void filterAll(
    const double dt, const double current_steer_angle,
    autoware_auto_control_msgs::msg::AckermannControlCommand & input) const;

private:
  VehicleCmdFilterParam param_;
  autoware_auto_control_msgs::msg::AckermannControlCommand prev_cmd_;

  double calcLatAcc(const autoware_auto_control_msgs::msg::AckermannControlCommand & cmd) const;
  double calcSteerFromLatacc(const double v, const double latacc) const;
  double limitDiff(const double curr, const double prev, const double diff_lim) const;
};

#endif  // VEHICLE_CMD_FILTER_HPP_
