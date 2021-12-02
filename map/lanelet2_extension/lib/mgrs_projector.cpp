// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
//
// Authors: Simon Thompson, Ryohsuke Mitsudome

#include "lanelet2_extension/projection/mgrs_projector.hpp"

#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace lanelet
{
namespace projection
{
MGRSProjector::MGRSProjector(Origin origin) : Projector(origin) {}

BasicPoint3d MGRSProjector::forward(const GPSPoint & gps) const
{
  BasicPoint3d mgrs_point(forward(gps, 0));
  return mgrs_point;
}

BasicPoint3d MGRSProjector::forward(const GPSPoint & gps, const int precision) const
{
  std::string prev_projected_grid = projected_grid_;

  BasicPoint3d mgrs_point{0., 0., gps.ele};
  BasicPoint3d utm_point{0., 0., gps.ele};
  int zone{};
  bool northp{};
  std::string mgrs_code;

  try {
    GeographicLib::UTMUPS::Forward(gps.lat, gps.lon, zone, northp, utm_point.x(), utm_point.y());
    GeographicLib::MGRS::Forward(
      zone, northp, utm_point.x(), utm_point.y(), gps.lat, precision, mgrs_code);
  } catch (const GeographicLib::GeographicErr & err) {
    std::cerr << err.what() << std::endl;
    return mgrs_point;
  }

  // get mgrs values from utm values
  mgrs_point.x() = fmod(utm_point.x(), 1e5);
  mgrs_point.y() = fmod(utm_point.y(), 1e5);
  projected_grid_ = mgrs_code;

  if (!prev_projected_grid.empty() && prev_projected_grid != projected_grid_) {
    std::cerr << "Projected MGRS Grid changed from last projection. Projected point"
                 "might be far away from previously projected point."
              << std::endl
              << "You may want to use different projector.";
  }

  return mgrs_point;
}

GPSPoint MGRSProjector::reverse(const BasicPoint3d & mgrs_point) const
{
  GPSPoint gps{0., 0., 0.};
  // reverse function cannot be used if mgrs_code_ is not set
  if (isMGRSCodeSet()) {
    gps = reverse(mgrs_point, mgrs_code_);
  } else if (!projected_grid_.empty()) {
    gps = reverse(mgrs_point, projected_grid_);
  } else {
    std::cerr << "cannot run reverse operation if mgrs code is not set in projector." << std::endl
              << "use setMGRSCode function or explicitly give mgrs code as an argument.";
  }
  return gps;
}

GPSPoint MGRSProjector::reverse(
  const BasicPoint3d & mgrs_point, const std::string & mgrs_code) const
{
  GPSPoint gps{0., 0., mgrs_point.z()};
  BasicPoint3d utm_point{0., 0., gps.ele};

  int zone{};
  int prec{};
  bool northp{};
  try {
    GeographicLib::MGRS::Reverse(
      mgrs_code, zone, northp, utm_point.x(), utm_point.y(), prec, false);
    utm_point.x() += fmod(mgrs_point.x(), pow(10, 5 - prec));
    utm_point.y() += fmod(mgrs_point.y(), pow(10, 5 - prec));
    GeographicLib::UTMUPS::Reverse(zone, northp, utm_point.x(), utm_point.y(), gps.lat, gps.lon);
  } catch (const GeographicLib::GeographicErr & err) {
    std::cerr << "Failed to convert from MGRS to WGS";
    return gps;
  }

  return gps;
}

void MGRSProjector::setMGRSCode(const std::string & mgrs_code) { mgrs_code_ = mgrs_code; }

void MGRSProjector::setMGRSCode(const GPSPoint & gps, const int precision)
{
  BasicPoint3d utm_point{0., 0., gps.ele};
  int zone{};
  bool northp{};
  std::string mgrs_code;

  try {
    GeographicLib::UTMUPS::Forward(gps.lat, gps.lon, zone, northp, utm_point.x(), utm_point.y());
    GeographicLib::MGRS::Forward(
      zone, northp, utm_point.x(), utm_point.y(), gps.lat, precision, mgrs_code);
  } catch (const GeographicLib::GeographicErr & err) {
    std::cerr << err.what() << std::endl;
  }

  setMGRSCode(mgrs_code);
}

}  // namespace projection
}  // namespace lanelet
