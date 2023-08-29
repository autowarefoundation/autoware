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

#include "map_projection_loader/load_info_from_lanelet2_map.hpp"

#include "tier4_map_msgs/msg/map_projector_info.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <string>

tier4_map_msgs::msg::MapProjectorInfo load_info_from_lanelet2_map(const std::string & filename)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  const lanelet::LaneletMapPtr map = lanelet::load(filename, projector, &errors);
  if (!errors.empty()) {
    throw std::runtime_error("Error occurred while loading lanelet2 map");
  }

  // If the lat & lon values in all the points of lanelet2 map are all zeros,
  // it will be interpreted as a local map.
  // If any single point exists with non-zero lat or lon values, it will be interpreted as MGRS.
  bool is_local = true;
  for (const auto & point : map->pointLayer) {
    const auto gps_point = projector.reverse(point);
    if (gps_point.lat != 0.0 || gps_point.lon != 0.0) {
      is_local = false;
      break;
    }
  }

  tier4_map_msgs::msg::MapProjectorInfo msg;
  if (is_local) {
    msg.projector_type = tier4_map_msgs::msg::MapProjectorInfo::LOCAL;
  } else {
    msg.projector_type = tier4_map_msgs::msg::MapProjectorInfo::MGRS;
    msg.mgrs_grid = projector.getProjectedMGRSGrid();
  }

  // We assume that the vertical datum of the map is WGS84 when using lanelet2 map.
  // However, do note that this is not always true, and may cause problems in the future.
  // Thus, please consider using the map_projector_info.yaml instead of this deprecated function.
  msg.vertical_datum = tier4_map_msgs::msg::MapProjectorInfo::WGS84;
  return msg;
}
