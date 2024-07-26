// Copyright 2021 Tier IV, Inc.
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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COST_VALUE__COST_VALUE_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COST_VALUE__COST_VALUE_HPP_

#include <algorithm>

namespace autoware::occupancy_grid_map
{
namespace cost_value
{
static const unsigned char NO_INFORMATION = 128;  // 0.5 * 255
static const unsigned char LETHAL_OBSTACLE = 255;
static const unsigned char FREE_SPACE = 0;

static const unsigned char OCCUPIED_THRESHOLD = 180;
static const unsigned char FREE_THRESHOLD = 50;

struct CostTranslationTable
{
  CostTranslationTable()
  {
    for (int i = 0; i < 256; i++) {
      const auto value =
        static_cast<char>(static_cast<float>(i) * 100.f / 255.f);  // 0-255 to 0-100
      data[i] =
        std::max(std::min(value, static_cast<char>(99)), static_cast<char>(1));  // 0-100 to 1-99
    }
  }
  char operator[](unsigned char n) const { return data[n]; }
  char data[256];
};
struct InverseCostTranslationTable
{
  InverseCostTranslationTable()
  {
    // 0-100 to 0-255
    for (int i = 0; i < 100; i++) {
      data[i] = static_cast<unsigned char>(i * 255 / 99);
    }
  }
  unsigned char operator[](char n) const
  {
    if (n > 99) {
      return data[99];
    } else if (n < 1) {
      return data[1];
    } else {
      const unsigned char u_n = static_cast<unsigned char>(n);
      return data[u_n];
    }
  }
  unsigned char data[100];
};

static const CostTranslationTable cost_translation_table;
static const InverseCostTranslationTable inverse_cost_translation_table;

}  // namespace cost_value
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COST_VALUE__COST_VALUE_HPP_
