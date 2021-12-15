// Copyright 2020 TierIV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

//lint -e537 pclint vs cpplint NOLINT

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "had_map_utils/had_map_query.hpp"

namespace autoware
{
namespace common
{
namespace had_map_utils
{

lanelet::Areas getAreaLayer(const lanelet::LaneletMapPtr ll_map)
{
  lanelet::Areas areas;
  for (auto ai = ll_map->areaLayer.begin(); ai != ll_map->areaLayer.end(); ai++) {
    areas.push_back(*ai);
  }
  return areas;
}

lanelet::Areas subtypeAreas(const lanelet::Areas areas, const char subtype[])
{
  lanelet::Areas subtype_areas;
  for (auto ai = areas.begin(); ai != areas.end(); ai++) {
    lanelet::Area a = *ai;
    if (a.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = a.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype) {
        subtype_areas.push_back(a);
      }
    }
  }
  return subtype_areas;
}

lanelet::Polygons3d getPolygonLayer(const lanelet::LaneletMapPtr ll_map)
{
  lanelet::Polygons3d polygons;
  for (auto ai = ll_map->polygonLayer.begin(); ai != ll_map->polygonLayer.end(); ai++) {
    polygons.push_back(*ai);
  }
  return polygons;
}

lanelet::Polygons3d subtypePolygons(const lanelet::Polygons3d polygons, const char subtype[])
{
  lanelet::Polygons3d subtype_polygons;
  for (auto pi = polygons.begin(); pi != polygons.end(); pi++) {
    lanelet::Polygon3d p = *pi;
    if (p.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = p.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype) {
        subtype_polygons.push_back(p);
      }
    }
  }
  return subtype_polygons;
}


lanelet::LineStrings3d getLineStringLayer(const lanelet::LaneletMapPtr ll_map)
{
  lanelet::LineStrings3d linestrings;
  for (auto lsi = ll_map->lineStringLayer.begin();
    lsi != ll_map->lineStringLayer.end(); lsi++)
  {
    linestrings.push_back(*lsi);
  }
  return linestrings;
}

lanelet::LineStrings3d subtypeLineStrings(
  const lanelet::LineStrings3d linestrings,
  const char subtype[])
{
  lanelet::LineStrings3d subtype_linestrings;
  for (auto lsi = linestrings.begin(); lsi != linestrings.end(); lsi++) {
    lanelet::LineString3d ls = *lsi;
    if (ls.hasAttribute(lanelet::AttributeName::Subtype)) {
      lanelet::Attribute attr = ls.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype) {
        subtype_linestrings.push_back(ls);
      }
    }
  }
  return subtype_linestrings;
}

lanelet::ConstLanelets getConstLaneletLayer(const std::shared_ptr<lanelet::LaneletMap> & ll_map)
{
  lanelet::ConstLanelets lanelets;
  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++) {
    lanelets.push_back(*li);
  }

  return lanelets;
}
lanelet::Lanelets getLaneletLayer(const std::shared_ptr<lanelet::LaneletMap> & ll_map)
{
  lanelet::Lanelets lanelets;
  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++) {
    lanelets.push_back(*li);
  }

  return lanelets;
}

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware
