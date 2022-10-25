// Copyright 2020 Tier IV, Inc.
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
#ifndef GNSS_POSER__GNSS_STAT_HPP_
#define GNSS_POSER__GNSS_STAT_HPP_

#include <string>

namespace gnss_poser
{
enum class CoordinateSystem {
  UTM = 0,
  MGRS = 1,
  PLANE = 2,
  LOCAL_CARTESIAN_WGS84 = 3,
  LOCAL_CARTESIAN_UTM = 4
};

struct GNSSStat
{
  GNSSStat()
  : coordinate_system(CoordinateSystem::MGRS),
    northup(true),
    zone(0),
    mgrs_zone(""),
    x(0),
    y(0),
    z(0),
    latitude(0),
    longitude(0),
    altitude(0)
  {
  }

  CoordinateSystem coordinate_system;
  bool northup;
  int zone;
  std::string mgrs_zone;
  double x;
  double y;
  double z;
  double latitude;
  double longitude;
  double altitude;
};
}  // namespace gnss_poser

#endif  // GNSS_POSER__GNSS_STAT_HPP_
