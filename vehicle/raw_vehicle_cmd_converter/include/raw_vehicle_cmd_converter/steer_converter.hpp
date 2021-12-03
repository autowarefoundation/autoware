//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef RAW_VEHICLE_CMD_CONVERTER__STEER_CONVERTER_HPP_
#define RAW_VEHICLE_CMD_CONVERTER__STEER_CONVERTER_HPP_

#include "raw_vehicle_cmd_converter/csv_loader.hpp"
#include "raw_vehicle_cmd_converter/interpolate.hpp"
#include "raw_vehicle_cmd_converter/pid.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
class SteerConverter
{
public:
  bool setFFMap(const std::string & csv_path);
  void setFBGains(const double kp, const double ki, const double kd);
  bool setFBLimits(
    const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
    const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d);
  double calcFFSteer(
    const double target_steer_angle_velocity, const double current_steer_angle) const;
  double calcFBSteer(
    const double target_steer_angle, const double dt, const double current_velocity,
    const double current_steer_angle, std::vector<double> & pid_contributions,
    std::vector<double> & errors);
  void setDecay(const double decay) { pid_.setDecay(decay); }

private:
  double kp_, ki_, kd_;
  std::string vehicle_name_;
  std::vector<double> vel_index_;
  std::vector<double> output_index_;
  std::vector<std::vector<double>> steer_map_;
  PIDController pid_;
  bool ff_map_initialized_{false};
  bool fb_gains_initialized_{false};
  bool fb_limits_initialized_{false};

  rclcpp::Logger logger_{
    rclcpp::get_logger("raw_vehicle_cmd_converter").get_child("steer_converter")};

  bool readSteerMapFromCSV(
    const std::string & csv_path, std::string & vehicle_name, std::vector<double> & vel_index,
    std::vector<double> & output_index, std::vector<std::vector<double>> & steer_map) const;
  void calcFFMap(double steer_vel, double current_steer_val, double & output) const;
};
}  // namespace raw_vehicle_cmd_converter

#endif  // RAW_VEHICLE_CMD_CONVERTER__STEER_CONVERTER_HPP_
