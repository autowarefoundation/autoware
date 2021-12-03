// Copyright 2015-2019 Autoware Foundation
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

#ifndef GEO_POS_CONV__GEO_POS_CONV_HPP_
#define GEO_POS_CONV__GEO_POS_CONV_HPP_

#include <math.h>

class geo_pos_conv
{
private:
  double m_x;  // m
  double m_y;  // m
  double m_z;  // m

  double m_lat;  // latitude
  double m_lon;  // longitude
  double m_h;

  double m_PLato;  // plane lat
  double m_PLo;    // plane lon

public:
  geo_pos_conv();
  double x() const;
  double y() const;
  double z() const;

  void set_plane(double lat, double lon);
  void set_plane(int num);
  void set_xyz(double cx, double cy, double cz);

  // set llh in radians
  void set_llh(double lat, double lon, double h);

  // set llh in nmea degrees
  void set_llh_nmea_degrees(double latd, double lond, double h);

  void llh_to_xyz(double lat, double lon, double ele);

  void conv_llh2xyz(void);
  void conv_xyz2llh(void);
};

#endif  // GEO_POS_CONV__GEO_POS_CONV_HPP_
