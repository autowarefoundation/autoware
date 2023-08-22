
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

#include "obstacle_velocity_limiter/map_utils.hpp"

#include "lanelet2_core/primitives/LineString.h"
#include "obstacle_velocity_limiter/types.hpp"

#include <boost/geometry.hpp>

#include <lanelet2_core/Attribute.h>

#include <algorithm>

namespace obstacle_velocity_limiter
{
multi_linestring_t extractStaticObstacles(
  const lanelet::LaneletMap & lanelet_map, const std::vector<std::string> & tags)
{
  multi_linestring_t lines;
  linestring_t line;
  linestring_t simplified_line;
  for (const auto & ls : lanelet_map.lineStringLayer) {
    if (isObstacle(ls, tags)) {
      line.clear();
      simplified_line.clear();
      for (const auto & p : ls) line.push_back(point_t{p.x(), p.y()});
      boost::geometry::simplify(line, simplified_line, 0.5);
      lines.push_back(simplified_line);
    }
  }
  return lines;
}

bool isObstacle(const lanelet::ConstLineString3d & ls, const std::vector<std::string> & tags)
{
  constexpr auto no_type = "";
  const auto type = ls.attributeOr(lanelet::AttributeName::Type, no_type);
  return (type != no_type && std::find(tags.begin(), tags.end(), type) != tags.end());
}
}  // namespace obstacle_velocity_limiter
