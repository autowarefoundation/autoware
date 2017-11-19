/*
 * This source code based on:
 * Copyright (c) 2012-2017 Tobias Bieniek <Tobias.Bieniek@gmx.de>
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "utm_converter.hpp"
#include <cmath>
#include <iostream>


UtmConverter::UtmConverter()
    : K0_(0.9996)
    , E_(0.00669438)
    , E2_(E_ * E_)
    , E3_(E2_ * E_)
    , E_P2_(E_ / (1.0 - E_))

    , SQRT_E_(std::sqrt(1 - E_))
    , _E((1 - SQRT_E_) / (1 + SQRT_E_))
    , _E2(_E * _E)
    , _E3(_E2 * _E)
    , _E4(_E3 * _E)
    , _E5(_E4 * _E)

    , M1_(1 - E_ / 4 - 3 * E2_ / 64 - 5 * E3_ / 256)
    , M2_(3 * E_ / 8 + 3 * E2_ / 32 + 45 * E3_ / 1024)
    , M3_(15 * E2_ / 256 + 45 * E3_ / 1024), M4_(35 * E3_ / 3072)

    , R_(6378137)

    , P2_(3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5)
    , P3_(21. / 16 * _E2 - 55. / 32 * _E4)
    , P4_(151. / 96 * _E3 - 417. / 128 * _E5)
    , P5_(1097. / 512 * _E4)

{}

UtmConverter::~UtmConverter() = default;

double UtmConverter::getRadian(double deg) { return deg * M_PI / 180; }

int UtmConverter::getZoneNumber(double lat, double lng) {
  if ((56 <= lat && lat < 64) && (3 <= lng && lng < 12))
    return 32;

  if ((72 <= lat && lat <= 84) && lng >= 0) {
    if (lng < 9)
      return 31;
    else if (lng < 21)
      return 33;
    else if (lng < 33)
      return 35;
    else if (lng < 42)
      return 37;
  }

  return int((lng + 180) / 6) + 1;
}

double UtmConverter::getCentralLongitude(int zone_num) {
  return (zone_num - 1) * 6 - 180 + 3;
}

std::string UtmConverter::getZoneLetter(double lat) {
  const char *ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX";
  std::string val;
  if (-80 <= lat && lat <= 84)
    val = ZONE_LETTERS[int(lat + 80) >> 3];
  return val;
}

std::tuple<double, double, int, std::string>
UtmConverter::latlng2utm(double lat, double lng) {
  int zone_num;
  std::string zone_letter;

  if (-80.0 >= lat || lat >= 84.0) {
    std::cout << "latitude out of range (must be between 80 deg S and 84 deg N)"
              << std::endl;
  }

  if (-180.0 >= lng || lng >= 180.0) {
    std::cout
        << "longitude out of range (must be between 180 deg W and 180 deg E)"
        << std::endl;
  }

  double lat_rad = UtmConverter::getRadian(lat);
  double lat_sin = std::sin(lat_rad);
  double lat_cos = std::cos(lat_rad);

  double lat_tan = lat_sin / lat_cos;
  double lat_tan2 = lat_tan * lat_tan;
  double lat_tan4 = lat_tan2 * lat_tan2;

  zone_num = UtmConverter::getZoneNumber(lat, lng);
  zone_letter = UtmConverter::getZoneLetter(lat);

  double lng_rad = UtmConverter::getRadian(lng);

  double central_lng = UtmConverter::getCentralLongitude(zone_num);
  double central_lng_rad = UtmConverter::getRadian(central_lng);

  double n = R_ / std::sqrt(1 - E_ * lat_sin * lat_sin);
  double c = E_P2_ * lat_cos * lat_cos;

  double a = lat_cos * (lng_rad - central_lng_rad);
  double a2 = a * a;
  double a3 = a2 * a;
  double a4 = a3 * a;
  double a5 = a4 * a;
  double a6 = a5 * a;

  double m = R_ * (M1_ * lat_rad - M2_ * std::sin(2 * lat_rad) +
                   M3_ * std::sin(4 * lat_rad) - M4_ * std::sin(6 * lat_rad));

  double x =
      K0_ * n *
          (a + a3 / 6 * (1 - lat_tan2 + c) +
           a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2_)) +
      500000;

  double y = K0_ * (m +
                    n * lat_tan *
                        (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) +
                         a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c -
                                     330 * E_P2_)));

  if (lat < 0)
    y += 10000000;

  return std::forward_as_tuple(x, y, zone_num, zone_letter);
}