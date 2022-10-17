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

#ifndef RAW_VEHICLE_CMD_CONVERTER__STEER_MAP_HPP_
#define RAW_VEHICLE_CMD_CONVERTER__STEER_MAP_HPP_

#include "raw_vehicle_cmd_converter/csv_loader.hpp"
#include "raw_vehicle_cmd_converter/pid.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
class SteerMap
{
public:
  bool readSteerMapFromCSV(const std::string & csv_path);
  void getSteer(const double steer_rate, const double steer, double & output) const;

private:
  std::string vehicle_name_;
  std::vector<double> steer_index_;
  std::vector<double> output_index_;
  std::vector<std::vector<double>> steer_map_;
  rclcpp::Logger logger_{rclcpp::get_logger("raw_vehicle_cmd_converter").get_child("steer_map")};
};
}  // namespace raw_vehicle_cmd_converter

#endif  // RAW_VEHICLE_CMD_CONVERTER__STEER_MAP_HPP_
