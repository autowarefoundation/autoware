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
#include <geography_utils/height.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace geography_utils
{

double convert_wgs84_to_egm2008(const double height, const double latitude, const double longitude)
{
  GeographicLib::Geoid egm2008("egm2008-1");
  // cSpell: ignore ELLIPSOIDTOGEOID
  return egm2008.ConvertHeight(latitude, longitude, height, GeographicLib::Geoid::ELLIPSOIDTOGEOID);
}

double convert_egm2008_to_wgs84(const double height, const double latitude, const double longitude)
{
  GeographicLib::Geoid egm2008("egm2008-1");
  // cSpell: ignore GEOIDTOELLIPSOID
  return egm2008.ConvertHeight(latitude, longitude, height, GeographicLib::Geoid::GEOIDTOELLIPSOID);
}

double convert_height(
  const double height, const double latitude, const double longitude,
  const std::string & source_vertical_datum, const std::string & target_vertical_datum)
{
  if (source_vertical_datum == target_vertical_datum) {
    return height;
  }
  std::map<std::pair<std::string, std::string>, HeightConversionFunction> conversion_map;
  conversion_map[{"WGS84", "EGM2008"}] = convert_wgs84_to_egm2008;
  conversion_map[{"EGM2008", "WGS84"}] = convert_egm2008_to_wgs84;

  auto key = std::make_pair(source_vertical_datum, target_vertical_datum);
  if (conversion_map.find(key) != conversion_map.end()) {
    return conversion_map[key](height, latitude, longitude);
  } else {
    std::string error_message =
      "Invalid conversion types: " + std::string(source_vertical_datum.c_str()) + " to " +
      std::string(target_vertical_datum.c_str());

    throw std::invalid_argument(error_message);
  }
}

}  // namespace geography_utils
