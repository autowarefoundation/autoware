// Copyright 2020-2023 TIER IV, Inc.
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

#include "lidar_apollo_instance_segmentation/log_table.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace
{
struct LogTable
{
  std::vector<float> data;
  LogTable()
  {
    data.resize(256 * 10);
    for (size_t i = 0; i < data.size(); ++i) {
      data[i] = std::log1p(static_cast<float>(i / 10.0));
    }
  }
};
}  // namespace

static ::LogTable log_table;

namespace lidar_apollo_instance_segmentation
{
float calcApproximateLog(float num)
{
  int integer_num = static_cast<int>(num * 10.0);
  if (integer_num < static_cast<int>(log_table.data.size())) {
    return log_table.data[integer_num];
  }
  return std::log(static_cast<float>(1.0 + num));
}
}  // namespace lidar_apollo_instance_segmentation
