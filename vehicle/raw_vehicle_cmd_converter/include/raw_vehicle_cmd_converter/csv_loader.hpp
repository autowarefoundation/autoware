// Copyright 2018-2019 Autoware Foundation
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

#ifndef RAW_VEHICLE_CMD_CONVERTER__CSV_LOADER_HPP_
#define RAW_VEHICLE_CMD_CONVERTER__CSV_LOADER_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
using Table = std::vector<std::vector<std::string>>;
using Map = std::vector<std::vector<double>>;
class CSVLoader
{
public:
  explicit CSVLoader(const std::string & csv_path);

  bool readCSV(Table & result, const char delim = ',');
  static bool validateData(const Table & table, const std::string & csv_path);
  static bool validateMap(const Map & map, const bool is_col_decent);
  static Map getMap(const Table & table);
  static std::vector<double> getRowIndex(const Table & table);
  static std::vector<double> getColumnIndex(const Table & table);
  static double clampValue(
    const double val, const std::vector<double> & ranges, const std::string & name);

private:
  std::string csv_path_;
};
}  // namespace raw_vehicle_cmd_converter

#endif  // RAW_VEHICLE_CMD_CONVERTER__CSV_LOADER_HPP_
