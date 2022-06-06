// Copyright 2022 TIER IV, Inc.
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
#ifndef OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__S_BOUNDARY_HPP_
#define OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__S_BOUNDARY_HPP_

#include <limits>
#include <vector>

class SBoundary
{
public:
  SBoundary(const double _max_s, const double _min_s) : max_s(_max_s), min_s(_min_s) {}
  SBoundary() : max_s(std::numeric_limits<double>::max()), min_s(0.0) {}

  double max_s = std::numeric_limits<double>::max();
  double min_s = 0.0;
  bool is_object = false;
};

using SBoundaries = std::vector<SBoundary>;

#endif  // OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__S_BOUNDARY_HPP_
