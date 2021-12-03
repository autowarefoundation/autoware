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

#include "raw_vehicle_cmd_converter/steer_converter.hpp"

#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
bool SteerConverter::setFFMap(const std::string & csv_path)
{
  if (!readSteerMapFromCSV(csv_path, vehicle_name_, vel_index_, output_index_, steer_map_)) {
    return false;
  }
  ff_map_initialized_ = true;
  return true;
}

void SteerConverter::setFBGains(const double kp, const double ki, const double kd)
{
  pid_.setGains(kp, ki, kd);
  fb_gains_initialized_ = true;
}

bool SteerConverter::setFBLimits(
  const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
  const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d)
{
  pid_.setLimits(
    max_ret, min_ret, max_ret_p, min_ret_p, max_ret_i, min_ret_i, max_ret_d, min_ret_d);
  fb_limits_initialized_ = true;
  return true;
}

double SteerConverter::calcFFSteer(
  const double target_steer_angle_velocity, const double current_steer_angle) const
{
  rclcpp::Clock clock{RCL_ROS_TIME};

  if (!ff_map_initialized_) {
    RCLCPP_WARN_THROTTLE(logger_, clock, 3000, "FF map is not initialized!");
    return 0;
  }

  double ff_value = 0;
  calcFFMap(target_steer_angle_velocity, current_steer_angle, ff_value);
  return ff_value;
}

double SteerConverter::calcFBSteer(
  const double target_steer_angle, const double dt, const double current_velocity,
  const double current_steer_angle, std::vector<double> & pid_contributions,
  std::vector<double> & errors)
{
  rclcpp::Clock clock{RCL_ROS_TIME};

  if (!fb_gains_initialized_ || !fb_limits_initialized_) {
    RCLCPP_WARN_THROTTLE(logger_, clock, 3000, "FB params are not initialized!");
    return 0;
  }

  double fb_value = 0;
  const double error_steer_angle = target_steer_angle - current_steer_angle;
  bool enable_integration = true;
  if (std::abs(current_velocity) < 0.01) {
    enable_integration = false;
  }
  fb_value =
    pid_.calculatePID(error_steer_angle, dt, enable_integration, pid_contributions, errors, false);
  return fb_value;
}

bool SteerConverter::readSteerMapFromCSV(
  const std::string & csv_path, std::string & vehicle_name, std::vector<double> & vel_index,
  std::vector<double> & output_index, std::vector<std::vector<double>> & steer_map) const
{
  rclcpp::Clock clock{RCL_ROS_TIME};

  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    RCLCPP_ERROR_THROTTLE(logger_, clock, 3000, "Cannot open %s", csv_path.c_str());
    return false;
  }

  if (table[0].size() < 2) {
    RCLCPP_ERROR_THROTTLE(
      logger_, clock, 3000,
      "Cannot read %s. CSV file should have "
      "at least 2 column",
      csv_path.c_str());
    return false;
  }

  vehicle_name = table[0][0];
  for (unsigned int i = 1; i < table[0].size(); ++i) {
    vel_index.push_back(std::stod(table[0][i]));
  }

  for (unsigned int i = 1; i < table.size(); ++i) {
    if (table[0].size() != table[i].size()) {
      RCLCPP_ERROR_THROTTLE(
        logger_, clock, 3000,
        "Cannot read %s. Each row should have "
        "a same number of columns",
        csv_path.c_str());
      return false;
    }
    output_index.push_back(std::stod(table[i][0]));
    std::vector<double> steer_angle_velocities;
    for (unsigned int j = 1; j < table[i].size(); ++j) {
      steer_angle_velocities.push_back(std::stod(table[i][j]));
    }
    steer_map.push_back(steer_angle_velocities);
  }

  return true;
}

void SteerConverter::calcFFMap(double steer_vel, double vehicle_vel, double & output) const
{
  rclcpp::Clock clock{RCL_ROS_TIME};

  LinearInterpolate linear_interp;
  std::vector<double> steer_angle_velocities_interp;

  if (vehicle_vel < vel_index_.front()) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock, 1000,
      "Exceeding the vel range. Current vel: "
      "%f < min vel on map: %f. Use min velocity.",
      vehicle_vel, vel_index_.front());
    vehicle_vel = vel_index_.front();
  } else if (vel_index_.back() < vehicle_vel) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock, 1000,
      "Exceeding the vel range. Current vel: "
      "%f > max vel on map: %f. Use max velocity.",
      vehicle_vel, vel_index_.back());
    vehicle_vel = vel_index_.back();
  }

  for (std::vector<double> steer_angle_velocities : steer_map_) {
    double steer_angle_vel_interp;
    linear_interp.interpolate(
      vel_index_, steer_angle_velocities, vehicle_vel, steer_angle_vel_interp);
    steer_angle_velocities_interp.push_back(steer_angle_vel_interp);
  }
  if (steer_vel < steer_angle_velocities_interp.front()) {
    steer_vel = steer_angle_velocities_interp.front();
  } else if (steer_angle_velocities_interp.back() < steer_vel) {
    steer_vel = steer_angle_velocities_interp.back();
  }
  linear_interp.interpolate(steer_angle_velocities_interp, output_index_, steer_vel, output);
}
}  // namespace raw_vehicle_cmd_converter
