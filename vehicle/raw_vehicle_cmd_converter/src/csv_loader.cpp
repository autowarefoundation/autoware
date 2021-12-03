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

#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
CSVLoader::CSVLoader(std::string csv_path) { csv_path_ = csv_path; }

bool CSVLoader::readCSV(std::vector<std::vector<std::string>> & result, const char delim)
{
  std::ifstream ifs(csv_path_);
  if (!ifs.is_open()) {
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

  return true;
}
}  // namespace raw_vehicle_cmd_converter
