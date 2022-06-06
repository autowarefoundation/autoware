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
#ifndef OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__ST_POINT_HPP_
#define OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__ST_POINT_HPP_

#include <vector>

class STPoint
{
public:
  STPoint(const double _s, const double _t) : s(_s), t(_t) {}
  STPoint() : s(0.0), t(0.0) {}

  double s;
  double t;
};

using STPoints = std::vector<STPoint>;

#endif  // OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__ST_POINT_HPP_
