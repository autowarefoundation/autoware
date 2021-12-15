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

#ifndef HAD_MAP_UTILS__HAD_MAP_QUERY_HPP_
#define HAD_MAP_UTILS__HAD_MAP_QUERY_HPP_


#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Units.h>

#include <cmath>
#include <memory>
#include "had_map_utils/visibility_control.hpp"

namespace autoware
{
namespace common
{
namespace had_map_utils
{

lanelet::Areas HAD_MAP_UTILS_PUBLIC getAreaLayer(const lanelet::LaneletMapPtr ll_map);

lanelet::Areas HAD_MAP_UTILS_PUBLIC subtypeAreas(
  const lanelet::Areas areas, const char subtype[]);

lanelet::Polygons3d HAD_MAP_UTILS_PUBLIC getPolygonLayer(const lanelet::LaneletMapPtr ll_map);

lanelet::Polygons3d HAD_MAP_UTILS_PUBLIC subtypePolygons(
  const lanelet::Polygons3d polygons, const char subtype[]);

lanelet::LineStrings3d HAD_MAP_UTILS_PUBLIC getLineStringLayer(const lanelet::LaneletMapPtr ll_map);

lanelet::LineStrings3d HAD_MAP_UTILS_PUBLIC subtypeLineStrings(
  const lanelet::LineStrings3d linestrings, const char subtype[]);

lanelet::ConstLanelets HAD_MAP_UTILS_PUBLIC getConstLaneletLayer(
  const std::shared_ptr<lanelet::LaneletMap> & ll_map);

lanelet::Lanelets HAD_MAP_UTILS_PUBLIC getLaneletLayer(
  const std::shared_ptr<lanelet::LaneletMap> & ll_map);

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware

#endif  // HAD_MAP_UTILS__HAD_MAP_QUERY_HPP_
