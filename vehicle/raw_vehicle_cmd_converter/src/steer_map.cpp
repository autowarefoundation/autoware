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

#include "raw_vehicle_cmd_converter/steer_map.hpp"

#include "interpolation/linear_interpolation.hpp"

#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{

bool SteerMap::readSteerMapFromCSV(const std::string & csv_path, const bool validation)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  vehicle_name_ = table[0][0];
  steer_index_ = CSVLoader::getRowIndex(table);
  output_index_ = CSVLoader::getColumnIndex(table);
  steer_map_ = CSVLoader::getMap(table);
  if (validation && !CSVLoader::validateMap(steer_map_, true)) {
    return false;
  }
  return true;
}

void SteerMap::getSteer(const double steer_rate, const double steer, double & output) const
{
  const double clamped_steer = CSVLoader::clampValue(steer, steer_index_, "steer: steer");
  std::vector<double> steer_rate_interp = {};
  for (const auto & steer_rate_vec : steer_map_) {
    steer_rate_interp.push_back(interpolation::lerp(steer_index_, steer_rate_vec, clamped_steer));
  }

  const double clamped_steer_rate =
    CSVLoader::clampValue(steer_rate, steer_rate_interp, "steer: steer_rate");
  output = interpolation::lerp(steer_rate_interp, output_index_, clamped_steer_rate);
}
}  // namespace raw_vehicle_cmd_converter
