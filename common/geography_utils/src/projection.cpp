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
#include <geography_utils/lanelet2_projector.hpp>
#include <geography_utils/projection.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>

namespace geography_utils
{

Eigen::Vector3d to_basic_point_3d_pt(const LocalPoint src)
{
  Eigen::Vector3d dst;
  dst.x() = src.x;
  dst.y() = src.y;
  dst.z() = src.z;
  return dst;
}

LocalPoint project_forward(const GeoPoint & geo_point, const MapProjectorInfo & projector_info)
{
  std::unique_ptr<lanelet::Projector> projector = get_lanelet2_projector(projector_info);
  lanelet::GPSPoint position{geo_point.latitude, geo_point.longitude, geo_point.altitude};

  lanelet::BasicPoint3d projected_local_point;
  if (projector_info.projector_type == MapProjectorInfo::MGRS) {
    const int mgrs_precision = 9;  // set precision as 100 micro meter
    const auto mgrs_projector = dynamic_cast<lanelet::projection::MGRSProjector *>(projector.get());

    // project x and y using projector
    // note that the altitude is ignored in MGRS projection conventionally
    projected_local_point = mgrs_projector->forward(position, mgrs_precision);
  } else {
    // project x and y using projector
    // note that the original projector such as UTM projector does not compensate for the altitude
    // offset
    projected_local_point = projector->forward(position);

    // correct z based on the map origin
    // note that the converted altitude in local point is in the same vertical datum as the geo
    // point
    projected_local_point.z() = geo_point.altitude - projector_info.map_origin.altitude;
  }

  LocalPoint local_point;
  local_point.x = projected_local_point.x();
  local_point.y = projected_local_point.y();
  local_point.z = projected_local_point.z();

  return local_point;
}

GeoPoint project_reverse(const LocalPoint & local_point, const MapProjectorInfo & projector_info)
{
  std::unique_ptr<lanelet::Projector> projector = get_lanelet2_projector(projector_info);

  lanelet::GPSPoint projected_gps_point;
  if (projector_info.projector_type == MapProjectorInfo::MGRS) {
    const auto mgrs_projector = dynamic_cast<lanelet::projection::MGRSProjector *>(projector.get());
    // project latitude and longitude using projector
    // note that the z is ignored in MGRS projection conventionally
    projected_gps_point =
      mgrs_projector->reverse(to_basic_point_3d_pt(local_point), projector_info.mgrs_grid);
  } else {
    // project latitude and longitude using projector
    // note that the original projector such as UTM projector does not compensate for the altitude
    // offset
    projected_gps_point = projector->reverse(to_basic_point_3d_pt(local_point));

    // correct altitude based on the map origin
    // note that the converted altitude in local point is in the same vertical datum as the geo
    // point
    projected_gps_point.ele = local_point.z + projector_info.map_origin.altitude;
  }

  GeoPoint geo_point;
  geo_point.latitude = projected_gps_point.lat;
  geo_point.longitude = projected_gps_point.lon;
  geo_point.altitude = projected_gps_point.ele;
  return geo_point;
}

}  // namespace geography_utils
