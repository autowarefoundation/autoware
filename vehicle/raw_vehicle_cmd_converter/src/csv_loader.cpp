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

#include "raw_vehicle_cmd_converter/csv_loader.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
CSVLoader::CSVLoader(const std::string & csv_path)
{
  csv_path_ = csv_path;
}

bool CSVLoader::readCSV(Table & result, const char delim)
{
  std::ifstream ifs(csv_path_);
  if (!ifs.is_open()) {
    std::cerr << "Cannot open " << csv_path_.c_str() << std::endl;
    return false;
  }

  std::string buf;
  while (std::getline(ifs, buf)) {
    std::vector<std::string> tokens;

    std::istringstream iss(buf);
    std::string token;
    while (std::getline(iss, token, delim)) {
      tokens.push_back(token);
    }

    if (tokens.size() != 0) {
      result.push_back(tokens);
    }
  }
  if (!validateData(result, csv_path_)) {
    return false;
  }
  return true;
}

bool CSVLoader::validateMap(const Map & map, const bool is_col_decent)
{
  std::pair<size_t, size_t> invalid_index_pair;
  bool is_invalid = false;
  // validate interpolation
  for (size_t i = 1; i < map.size(); i++) {
    const auto & vec = map.at(i);
    const auto & prev_vec = map.at(i - 1);
    // validate row data
    for (size_t j = 0; j < vec.size(); j++) {
      // validate col
      if (vec.at(j) <= prev_vec.at(j) && is_col_decent) {
        invalid_index_pair = std::make_pair(i, j);
        is_invalid = true;
      }
      if (vec.at(j) >= prev_vec.at(j) && !is_col_decent) {
        invalid_index_pair = std::make_pair(i, j);
        is_invalid = true;
      }
    }
  }
  if (is_invalid) {
    std::cerr << "index around (i,j) is invalid ( " << invalid_index_pair.first << ", "
              << invalid_index_pair.second << " )" << std::endl;
    return false;
  }
  return true;
}

bool CSVLoader::validateData(const Table & table, const std::string & csv_path)
{
  if (table.empty()) {
    std::cerr << "The table is empty." << std::endl;
    return false;
  }
  if (table[0].size() < 2) {
    std::cerr << "Cannot read " << csv_path.c_str() << " CSV file should have at least 2 column"
              << std::endl;
    return false;
  }
  // validate map size
  for (size_t i = 1; i < table.size(); i++) {
    // validate row size
    if (table[0].size() != table[i].size()) {
      std::cerr << "Cannot read " << csv_path.c_str()
                << ". Each row should have a same number of columns" << std::endl;
      return false;
    }
  }
  return true;
}

Map CSVLoader::getMap(const Table & table)
{
  Map map = {};
  for (size_t i = 1; i < table.size(); i++) {
    std::vector<double> accelerations;
    for (size_t j = 1; j < table[i].size(); j++) {
      accelerations.push_back(std::stod(table[i][j]));
    }
    map.push_back(accelerations);
  }
  return map;
}

std::vector<double> CSVLoader::getRowIndex(const Table & table)
{
  std::vector<double> index = {};
  for (size_t i = 1; i < table[0].size(); i++) {
    index.push_back(std::stod(table[0][i]));
  }
  return index;
}

std::vector<double> CSVLoader::getColumnIndex(const Table & table)
{
  std::vector<double> index = {};
  for (size_t i = 1; i < table.size(); i++) {
    index.push_back(std::stod(table[i][0]));
  }
  return index;
}

double CSVLoader::clampValue(
  const double val, const std::vector<double> & ranges, const std::string & name)
{
  const double max_value = *std::max_element(ranges.begin(), ranges.end());
  const double min_value = *std::min_element(ranges.begin(), ranges.end());
  if (val < min_value || max_value < val) {
    std::cerr << "Input " << name << ": " << val << " is out of range. use closest value."
              << std::endl;
    return std::min(std::max(val, min_value), max_value);
  }
  return val;
}

}  // namespace raw_vehicle_cmd_converter
