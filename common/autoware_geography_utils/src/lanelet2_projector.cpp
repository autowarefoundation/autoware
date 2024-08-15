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

#include <GeographicLib/Geoid.hpp>
#include <autoware/geography_utils/lanelet2_projector.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/projection/transverse_mercator_projector.hpp>

#include <lanelet2_projection/UTM.h>

namespace autoware::geography_utils
{

std::unique_ptr<lanelet::Projector> get_lanelet2_projector(const MapProjectorInfo & projector_info)
{
  if (projector_info.projector_type == MapProjectorInfo::LOCAL_CARTESIAN_UTM) {
    lanelet::GPSPoint position{
      projector_info.map_origin.latitude, projector_info.map_origin.longitude,
      projector_info.map_origin.altitude};
    lanelet::Origin origin{position};
    lanelet::projection::UtmProjector projector{origin};
    return std::make_unique<lanelet::projection::UtmProjector>(projector);

  } else if (projector_info.projector_type == MapProjectorInfo::MGRS) {
    lanelet::projection::MGRSProjector projector{};
    projector.setMGRSCode(projector_info.mgrs_grid);
    return std::make_unique<lanelet::projection::MGRSProjector>(projector);

  } else if (projector_info.projector_type == MapProjectorInfo::TRANSVERSE_MERCATOR) {
    lanelet::GPSPoint position{
      projector_info.map_origin.latitude, projector_info.map_origin.longitude,
      projector_info.map_origin.altitude};
    lanelet::Origin origin{position};
    lanelet::projection::TransverseMercatorProjector projector{origin};
    return std::make_unique<lanelet::projection::TransverseMercatorProjector>(projector);
  }
  const std::string error_msg =
    "Invalid map projector type: " + projector_info.projector_type +
    ". Currently supported types: MGRS, LocalCartesianUTM, and TransverseMercator";
  throw std::invalid_argument(error_msg);
}

}  // namespace autoware::geography_utils
