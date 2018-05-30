/*
 * This UTM converter source code is based on python code:
 * Copyright (c) 2012-2017 Tobias Bieniek <Tobias.Bieniek@gmx.de>
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <proj_api.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "mgrs_converter.hpp"

namespace map_tools
{
MgrsConverter::MgrsConverter()
  : K0_(0.9996)
  , E_(0.00669438)
  , E2_(E_ * E_)
  , E3_(E2_ * E_)
  , E_P2_(E_ / (1.0 - E_))
  , SQRT_E_(std::sqrt(1 - E_))
  , Es_((1 - SQRT_E_) / (1 + SQRT_E_))
  , Es2_(Es_ * Es_)
  , Es3_(Es2_ * Es_)
  , Es4_(Es3_ * Es_)
  , Es5_(Es4_ * Es_)
  , M1_(1 - E_ / 4 - 3 * E2_ / 64 - 5 * E3_ / 256)
  , M2_(3 * E_ / 8 + 3 * E2_ / 32 + 45 * E3_ / 1024)
  , M3_(15 * E2_ / 256 + 45 * E3_ / 1024)
  , M4_(35 * E3_ / 3072)
  , R_(6378137)
  , P2_(3. / 2 * Es_ - 27. / 32 * Es3_ + 269. / 512 * Es5_)
  , P3_(21. / 16 * Es2_ - 55. / 32 * Es4_)
  , P4_(151. / 96 * Es3_ - 417. / 128 * Es5_)
  , P5_(1097. / 512 * Es4_)
{
}

MgrsConverter::~MgrsConverter() = default;

double MgrsConverter::getRadian(const double& deg)
{
  return deg * M_PI / 180;
}

int MgrsConverter::getUtmZoneNumber(const double& lat, const double& lon)
{
  if ((56 <= lat && lat < 64) && (3 <= lon && lon < 12))
    return 32;

  if ((72 <= lat && lat <= 84) && lon >= 0)
  {
    if (lon < 9)
      return 31;
    else if (lon < 21)
      return 33;
    else if (lon < 33)
      return 35;
    else if (lon < 42)
      return 37;
  }

  return int((lon + 180) / 6) + 1;
}

void MgrsConverter::setPlaneRef(int num, double& lat_0, double& lon_0)
{
  double lat_deg, lat_min, lon_deg, lon_min;  // longitude and latitude of origin of each plane in Japan
  if (num == 1)
  {
    lat_deg = 33;
    lat_min = 0;
    lon_deg = 129;
    lon_min = 30;
  }
  else if (num == 2)
  {
    lat_deg = 33;
    lat_min = 0;
    lon_deg = 131;
    lon_min = 0;
  }
  else if (num == 3)
  {
    lat_deg = 36;
    lat_min = 0;
    lon_deg = 132;
    lon_min = 10;
  }
  else if (num == 4)
  {
    lat_deg = 33;
    lat_min = 0;
    lon_deg = 133;
    lon_min = 30;
  }
  else if (num == 5)
  {
    lat_deg = 36;
    lat_min = 0;
    lon_deg = 134;
    lon_min = 20;
  }
  else if (num == 6)
  {
    lat_deg = 36;
    lat_min = 0;
    lon_deg = 136;
    lon_min = 0;
  }
  else if (num == 7)
  {
    lat_deg = 36;
    lat_min = 0;
    lon_deg = 137;
    lon_min = 10;
  }
  else if (num == 8)
  {
    lat_deg = 36;
    lat_min = 0;
    lon_deg = 138;
    lon_min = 30;
  }
  else if (num == 9)
  {
    lat_deg = 36;
    lat_min = 0;
    lon_deg = 139;
    lon_min = 50;
  }
  else if (num == 10)
  {
    lat_deg = 40;
    lat_min = 0;
    lon_deg = 140;
    lon_min = 50;
  }
  else if (num == 11)
  {
    lat_deg = 44;
    lat_min = 0;
    lon_deg = 140;
    lon_min = 15;
  }
  else if (num == 12)
  {
    lat_deg = 44;
    lat_min = 0;
    lon_deg = 142;
    lon_min = 15;
  }
  else if (num == 13)
  {
    lat_deg = 44;
    lat_min = 0;
    lon_deg = 144;
    lon_min = 15;
  }
  else if (num == 14)
  {
    lat_deg = 26;
    lat_min = 0;
    lon_deg = 142;
    lon_min = 0;
  }
  else if (num == 15)
  {
    lat_deg = 26;
    lat_min = 0;
    lon_deg = 127;
    lon_min = 30;
  }
  else if (num == 16)
  {
    lat_deg = 26;
    lat_min = 0;
    lon_deg = 124;
    lon_min = 0;
  }
  else if (num == 17)
  {
    lat_deg = 26;
    lat_min = 0;
    lon_deg = 131;
    lon_min = 0;
  }
  else if (num == 18)
  {
    lat_deg = 20;
    lat_min = 0;
    lon_deg = 136;
    lon_min = 0;
  }
  else if (num == 19)
  {
    lat_deg = 26;
    lat_min = 0;
    lon_deg = 154;
    lon_min = 0;
  }
  else
  {  // default is plane 7
    lat_deg = 36;
    lat_min = 0;
    lon_deg = 137;
    lon_min = 10;
  }

  // longitude and latitude
  lat_0 = lat_deg + lat_min / 60.0;
  lon_0 = lon_deg + lon_min / 60.0;
}

double MgrsConverter::getCentralLongitude(const int& zone_num)
{
  return (zone_num - 1) * 6 - 180 + 3;
}

std::string MgrsConverter::getUtmZoneLetter(const double& lat)
{
  const char* ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX";
  std::string val;
  if (-80.0 <= lat && lat <= 84.0)
    val = ZONE_LETTERS[int(lat + 80) >> 3];
  return val;
}

void MgrsConverter::latlon2utm(const double& lat, const double& lon, double& x, double& y)
{
  int zone_num;
  std::string zone_letter;

  if (-80.0 >= lat || lat >= 84.0)
  {
    std::cout << "latitude out of range (must be between 80 deg S and 84 deg N)" << std::endl;
  }

  if (-180.0 >= lon || lon >= 180.0)
  {
    std::cout << "longitude out of range (must be between 180 deg W and 180 deg E)" << std::endl;
  }

  double lat_rad = MgrsConverter::getRadian(lat);
  double lat_sin = std::sin(lat_rad);
  double lat_cos = std::cos(lat_rad);

  double lat_tan = lat_sin / lat_cos;
  double lat_tan2 = lat_tan * lat_tan;
  double lat_tan4 = lat_tan2 * lat_tan2;

  zone_num = MgrsConverter::getUtmZoneNumber(lat, lon);
  zone_letter = MgrsConverter::getUtmZoneLetter(lat);

  double lng_rad = MgrsConverter::getRadian(lon);

  double central_lng = MgrsConverter::getCentralLongitude(zone_num);
  double central_lng_rad = MgrsConverter::getRadian(central_lng);

  double n = R_ / std::sqrt(1 - E_ * lat_sin * lat_sin);
  double c = E_P2_ * lat_cos * lat_cos;

  double a = lat_cos * (lng_rad - central_lng_rad);
  double a2 = a * a;
  double a3 = a2 * a;
  double a4 = a3 * a;
  double a5 = a4 * a;
  double a6 = a5 * a;

  double m =
      R_ * (M1_ * lat_rad - M2_ * std::sin(2 * lat_rad) + M3_ * std::sin(4 * lat_rad) - M4_ * std::sin(6 * lat_rad));

  x = K0_ * n * (a + a3 / 6 * (1 - lat_tan2 + c) + a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2_)) +
      500000;

  y = K0_ * (m +
             n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) +
                            a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2_)));

  if (lat < 0)
    y += 10000000;
}

std::tuple<std::string, double, double> MgrsConverter::latlon2mgrs(double lat, double lon)
{
  double utm_x, utm_y;       // UTM
  double easting, northing;  // MGRS
  int zone;

  zone = getUtmZoneNumber(lat, lon);
  latlon2utm(lat, lon, utm_x, utm_y);
  easting = utm_x;
  northing = utm_y;

  if (lat <= 0.0 && utm_x == 1.0e7)
  {
    lat = 0;
    northing = 0;
  }

  int ltr2low;
  double pattern_offset;
  int setnum = zone % 6;

  if (setnum == 1 || setnum == 4)
  {
    ltr2low = 0;  // A
  }
  else if (setnum == 2 || setnum == 5)
  {
    ltr2low = 9;  // J
  }
  else  // if (setnum == 3 || setnum == 0)
  {
    ltr2low = 18;  // S
  }

  if (setnum % 2)
  {
    pattern_offset = 0.0;
  }
  else
  {
    pattern_offset = 500000.0;
  }

  // calculate northing(MGRS Y in meters)
  northing = std::fmod(northing, 2000000.0);
  northing += pattern_offset;
  if (northing >= 2000000.0)
  {
    northing -= 2000000.0;
  }

  int letters[3];

  // latitudeLetter
  if (lat >= 72 && lat < 84.5)
  {
    letters[0] = 23;
  }
  else if (lat > -80.5 && lat < 72)
  {
    int idx = int(((lat + 80.0) / 8.0) + 1.0e-12);
    letters[0] = lat_band[idx];
  }
  else
  {
    std::cerr << "latitude is out of scope" << std::endl;
    exit(1);
  }

  if (letters[0] == 21 && zone == 31 && easting == 500000.0)
  {
    easting = easting - 1.0;
  }

  letters[1] = ltr2low + int((easting / 100000.0) - 1);
  if (ltr2low == 9 && letters[1] > 13)
  {
    letters[1] += 1;
  }

  letters[2] = int(northing / 100000.0);
  if (letters[2] > 7)
  {
    ++letters[2];
  }
  if (letters[2] > 13)
  {
    ++letters[2];  // if letter[2] > 13 increment twice
  }

  easting = std::fmod(easting + 1e-8, 100000.0);
  northing = std::fmod(northing + 1e-8, 100000.0);

  //_mgrs_code(100m square)
  std::string mgrs_code;
  std::ostringstream s_zone, s_eid, s_nid;
  s_zone << std::setfill('0') << std::setw(2) << zone;
  std::string band = std::string{ alphabet[letters[0]] };
  std::string grid_id = std::string{ alphabet[letters[1]] } + std::string{ alphabet[letters[2]] };

  int e_id = static_cast<int>(easting) / 100;
  int n_id = static_cast<int>(northing) / 100;
  s_eid << std::setfill('0') << std::setw(3) << e_id;
  s_nid << std::setfill('0') << std::setw(3) << n_id;

  mgrs_code = s_zone.str() + band + grid_id + s_eid.str() + s_nid.str();

  return std::forward_as_tuple(mgrs_code, easting, northing);
}

void MgrsConverter::jpxy2latlon(const double& x, const double& y, const double& z, const int& plane, double& lat,
                                double& lon, double& alt)
{
  projPJ pj_latlong, pj_utm;
  double lat_0, lon_0;  // reference point of Japanese plane coordinate system
  setPlaneRef(plane, lat_0, lon_0);
  pj_latlong = pj_init_plus("+proj=latlong");
  std::stringstream ss;
  ss << "+proj=tmerc +lat_0=" << lat_0 << " +lon_0=" << lon_0 << " +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 "
                                                                 "+towgs84=0,0,0,0,0,0,0 +units=m +no_defs";
  pj_utm = pj_init_plus(ss.str().c_str());

  double _lat = x;
  double _lon = y;
  double _alt = z;

  if (pj_latlong != 0 && pj_utm != 0)
  {
    pj_transform(pj_utm, pj_latlong, 1, 1, &_lon, &_lat, &_alt);
    _lon = _lon * RAD_TO_DEG;
    _lat = _lat * RAD_TO_DEG;

    lon = _lon;
    lat = _lat;
    alt = _alt;
  }
  else
  {
    lon = lat = alt = 0;
  }
}
}
