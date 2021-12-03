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

#include "raw_vehicle_cmd_converter/brake_map.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
bool BrakeMap::readBrakeMapFromCSV(std::string csv_path)
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
    brake_index_.push_back(std::stod(table[i][0]));
    std::vector<double> accs;
    for (unsigned int j = 1; j < table[i].size(); j++) {
      accs.push_back(std::stod(table[i][j]));
    }
    brake_map_.push_back(accs);
  }

  brake_index_rev_ = brake_index_;
  std::reverse(std::begin(brake_index_rev_), std::end(brake_index_rev_));

  return true;
}

bool BrakeMap::getBrake(double acc, double vel, double & brake)
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
  for (std::vector<double> accs : brake_map_) {
    double acc_interpolated;
    linear_interp.interpolate(vel_index_, accs, vel, acc_interpolated);
    accs_interpolated.push_back(acc_interpolated);
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return max brake on the map
  // When the desired acceleration is greater than the brake area, return min brake on the map
  if (acc < accs_interpolated.back()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000,
      "Exceeding the acc range. Desired acc: %f < min acc on map: %f. return max "
      "value.",
      acc, accs_interpolated.back());
    brake = brake_index_.back();
    return true;
  } else if (accs_interpolated.front() < acc) {
    brake = brake_index_.front();
    return true;
  }

  std::reverse(std::begin(accs_interpolated), std::end(accs_interpolated));
  linear_interp.interpolate(accs_interpolated, brake_index_rev_, acc, brake);

  return true;
}

bool BrakeMap::getAcceleration(double brake, double vel, double & acc)
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
  for (std::vector<double> accs : brake_map_) {
    double acc_interpolated;
    linear_interp.interpolate(vel_index_, accs, vel, acc_interpolated);
    accs_interpolated.push_back(acc_interpolated);
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return max brake on the map
  // When the desired acceleration is greater than the brake area, return min brake on the map
  const double max_brake = brake_index_.back();
  const double min_brake = brake_index_.front();
  if (brake < min_brake || max_brake < brake) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000, "Input brake: %f is out off range. use closest value.", brake);
    brake = std::min(std::max(brake, min_brake), max_brake);
  }

  linear_interp.interpolate(brake_index_, accs_interpolated, brake, acc);

  return true;
}
}  // namespace raw_vehicle_cmd_converter
