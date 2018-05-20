/*
 * The rights of this source code conform to
 * https://github.com/CPFL/Autoware/blob/master/LICENSE
 *
 * (latitude, longitude) convert to UTM(x,y,zone num, zone letter)
 */

#include <iostream>
#include <string>
#include <tuple>
#include "mgrs_converter.hpp"

int main(int argc, char **argv)
{
  double lat, lon;
  double x, y;
  map_tools::MgrsConverter converter;

  std::cout << "input latitude: ";
  std::cin >> lat;
  std::cout << "input longitude: ";
  std::cin >> lon;

  int zone_num = converter.getUtmZoneNumber(lat, lon);
  std::string zone_letter = converter.getUtmZoneLetter(lat);
  converter.latlon2utm(lat, lon, &x, &y);

  std::string mgrs_code;
  double easting, northing;

  std::tie(mgrs_code, easting, northing) = converter.latlon2mgrs(lat, lon);

  std::cout << "UTM: " << x << ", " << y << ", " << zone_num << ", " << zone_letter << std::endl;
  std::cout << "MGRS: " << mgrs_code << ", " << easting << ", " << northing << std::endl;

  return 0;
}