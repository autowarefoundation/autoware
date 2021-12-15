// Copyright 2021 Tier IV, Inc
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

#ifndef HAD_MAP_UTILS__HAD_MAP_COMPUTATION_HPP_
#define HAD_MAP_UTILS__HAD_MAP_COMPUTATION_HPP_

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <lanelet2_core/LaneletMap.h>

#include "visibility_control.hpp"

namespace autoware
{
namespace common
{
namespace had_map_utils
{

lanelet::Polygon3d HAD_MAP_UTILS_PUBLIC coalesce_drivable_areas(
  const autoware_auto_planning_msgs::msg::HADMapRoute & had_map_route,
  const lanelet::LaneletMapPtr & lanelet_map_ptr);

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware

#endif  // HAD_MAP_UTILS__HAD_MAP_COMPUTATION_HPP_
