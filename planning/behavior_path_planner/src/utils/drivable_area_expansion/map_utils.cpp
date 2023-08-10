
// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/drivable_area_expansion/map_utils.hpp"

#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"
#include "lanelet2_core/primitives/LineString.h"

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>

#include <algorithm>

namespace drivable_area_expansion
{
multi_linestring_t extractUncrossableLines(
  const lanelet::LaneletMap & lanelet_map, const std::vector<std::string> & uncrossable_types)
{
  multi_linestring_t lines;
  linestring_t line;
  for (const auto & ls : lanelet_map.lineStringLayer) {
    if (hasTypes(ls, uncrossable_types)) {
      line.clear();
      for (const auto & p : ls) line.push_back(point_t{p.x(), p.y()});
      lines.push_back(line);
    }
  }
  return lines;
}

bool hasTypes(const lanelet::ConstLineString3d & ls, const std::vector<std::string> & types)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  return (type != no_type && std::find(types.begin(), types.end(), type) != types.end());
}
}  // namespace drivable_area_expansion
