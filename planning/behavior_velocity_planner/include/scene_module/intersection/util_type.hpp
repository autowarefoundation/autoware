// Copyright 2022 Tier IV, Inc.
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

#ifndef SCENE_MODULE__INTERSECTION__UTIL_TYPE_HPP_
#define SCENE_MODULE__INTERSECTION__UTIL_TYPE_HPP_

#include <lanelet2_core/primitives/Lanelet.h>

#include <vector>

namespace behavior_velocity_planner::util
{
struct IntersectionLanelets
{
  bool tl_arrow_solid_on;
  lanelet::ConstLanelets attention;
  lanelet::ConstLanelets conflicting;
  lanelet::ConstLanelets adjacent;
  std::vector<lanelet::CompoundPolygon3d> attention_area;
  std::vector<lanelet::CompoundPolygon3d> conflicting_area;
  std::vector<lanelet::CompoundPolygon3d> adjacent_area;
};

struct StopLineIdx
{
  size_t pass_judge_line = 0;
  size_t collision_stop_line = 0;
};

}  // namespace behavior_velocity_planner::util

#endif  // SCENE_MODULE__INTERSECTION__UTIL_TYPE_HPP_
