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

#include "raw_vehicle_cmd_converter/accel_map.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

using namespace std::literals::chrono_literals;

namespace raw_vehicle_cmd_converter
{
bool AccelMap::readAccelMapFromCSV(std::string csv_path)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    RCLCPP_ERROR(logger_, "Cannot open %s", csv_path.c_str());
    return false;
  }

  if (table[0].size() < 2) {
    RCLCPP_ERROR(
      logger_, "Cannot read %s. CSV file should have at least 2 column", csv_path.c_str());
    return false;
  }
  vehicle_name_ = table[0][0];
  for (unsigned int i = 1; i < table[0].size(); i++) {
    vel_index_.push_back(std::stod(table[0][i]));
  }

  for (unsigned int i = 1; i < table.size(); i++) {
    if (table[0].size() != table[i].size()) {
      RCLCPP_ERROR(
        logger_, "Cannot read %s. Each row should have a same number of columns", csv_path.c_str());
      return false;
    }
    throttle_index_.push_back(std::stod(table[i][0]));
    std::vector<double> accs;
    for (unsigned int j = 1; j < table[i].size(); j++) {
      accs.push_back(std::stod(table[i][j]));
    }
    accel_map_.push_back(accs);
  }

  return true;
}

bool AccelMap::getThrottle(double acc, double vel, double & throttle)
{
  LinearInterpolate linear_interp;
  std::vector<double> accs_interpolated;

  if (vel < vel_index_.front()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000,
      "Exceeding the vel range. Current vel: %f < min vel on map: %f. Use min "
      "velocity.",
      vel, vel_index_.front());
    vel = vel_index_.front();
  } else if (vel_index_.back() < vel) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000,
      "Exceeding the vel range. Current vel: %f > max vel on map: %f. Use max "
      "velocity.",
      vel, vel_index_.back());
    vel = vel_index_.back();
  }

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accs : accel_map_) {
    double acc_interpolated;
    linear_interp.interpolate(vel_index_, accs, vel, acc_interpolated);
    accs_interpolated.push_back(acc_interpolated);
  }

  // calculate throttle
  // When the desired acceleration is smaller than the throttle area, return false => brake sequence
  // When the desired acceleration is greater than the throttle area, return max throttle
  if (acc < accs_interpolated.front()) {
    return false;
  } else if (accs_interpolated.back() < acc) {
    throttle = throttle_index_.back();
    return true;
  }
  linear_interp.interpolate(accs_interpolated, throttle_index_, acc, throttle);

  return true;
}

bool AccelMap::getAcceleration(double throttle, double vel, double & acc)
{
  LinearInterpolate linear_interp;
  std::vector<double> accs_interpolated;

  if (vel < vel_index_.front()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000,
      "Exceeding the vel range. Current vel: %f < min vel on map: %f. Use min "
      "velocity.",
      vel, vel_index_.front());
    vel = vel_index_.front();
  } else if (vel_index_.back() < vel) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000,
      "Exceeding the vel range. Current vel: %f > max vel on map: %f. Use max "
      "velocity.",
      vel, vel_index_.back());
    vel = vel_index_.back();
  }

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accs : accel_map_) {
    double acc_interpolated;
    linear_interp.interpolate(vel_index_, accs, vel, acc_interpolated);
    accs_interpolated.push_back(acc_interpolated);
  }

  // calculate throttle
  // When the desired acceleration is smaller than the throttle area, return false => brake sequence
  // When the desired acceleration is greater than the throttle area, return max throttle
  const double max_throttle = throttle_index_.back();
  const double min_throttle = throttle_index_.front();
  if (throttle < min_throttle || max_throttle < throttle) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000, "Input throttle: %f is out off range. use closest value.", throttle);
    throttle = std::min(std::max(throttle, min_throttle), max_throttle);
  }

  linear_interp.interpolate(throttle_index_, accs_interpolated, throttle, acc);

  return true;
}
}  // namespace raw_vehicle_cmd_converter
