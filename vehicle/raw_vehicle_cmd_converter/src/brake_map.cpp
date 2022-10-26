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

#include "interpolation/linear_interpolation.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
bool BrakeMap::readBrakeMapFromCSV(const std::string & csv_path)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  vehicle_name_ = table[0][0];
  vel_index_ = CSVLoader::getRowIndex(table);
  brake_index_ = CSVLoader::getColumnIndex(table);
  brake_map_ = CSVLoader::getMap(table);
  brake_index_rev_ = brake_index_;
  if (!CSVLoader::validateMap(brake_map_, false, false)) {
    return false;
  }
  std::reverse(std::begin(brake_index_rev_), std::end(brake_index_rev_));

  return true;
}

bool BrakeMap::getBrake(const double acc, const double vel, double & brake)
{
  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_, "brake: vel");

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accelerations : brake_map_) {
    interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, accelerations, clamped_vel));
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return max brake on the map
  // When the desired acceleration is greater than the brake area, return min brake on the map
  if (acc < interpolated_acc_vec.back()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, clock_, 1000,
      "Exceeding the acc range. Desired acc: %f < min acc on map: %f. return max "
      "value.",
      acc, interpolated_acc_vec.back());
    brake = brake_index_.back();
    return true;
  } else if (interpolated_acc_vec.front() < acc) {
    brake = brake_index_.front();
    return true;
  }

  std::reverse(std::begin(interpolated_acc_vec), std::end(interpolated_acc_vec));
  brake = interpolation::lerp(interpolated_acc_vec, brake_index_rev_, acc);

  return true;
}

bool BrakeMap::getAcceleration(const double brake, const double vel, double & acc) const
{
  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_index_, "brake: vel");

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (const auto & acc_vec : brake_map_) {
    interpolated_acc_vec.push_back(interpolation::lerp(vel_index_, acc_vec, clamped_vel));
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return min acc
  // When the desired acceleration is greater than the brake area, return min acc
  const double clamped_brake = CSVLoader::clampValue(brake, brake_index_, "brake: acc");
  acc = interpolation::lerp(brake_index_, interpolated_acc_vec, clamped_brake);

  return true;
}
}  // namespace raw_vehicle_cmd_converter
