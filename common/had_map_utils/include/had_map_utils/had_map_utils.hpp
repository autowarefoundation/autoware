// Copyright 2020 Tier IV, Inc
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

#ifndef HAD_MAP_UTILS__HAD_MAP_UTILS_HPP_
#define HAD_MAP_UTILS__HAD_MAP_UTILS_HPP_


#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Units.h>
#include <common/types.hpp>

#include <cmath>
#include "had_map_utils/visibility_control.hpp"

namespace autoware
{
namespace common
{
namespace had_map_utils
{

using autoware::common::types::float64_t;

void HAD_MAP_UTILS_PUBLIC overwriteLaneletsCenterline(
  lanelet::LaneletMapPtr lanelet_map, const autoware::common::types::bool8_t force_overwrite);
lanelet::LineString3d HAD_MAP_UTILS_PUBLIC generateFineCenterline(
  const lanelet::ConstLanelet & lanelet_obj, const float64_t resolution);

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware

#endif  // HAD_MAP_UTILS__HAD_MAP_UTILS_HPP_
