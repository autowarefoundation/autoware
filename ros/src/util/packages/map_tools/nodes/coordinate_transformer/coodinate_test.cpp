/*
 * The rights of this source code conform to
 * https://github.com/CPFL/Autoware/blob/master/LICENSE
 *
 * (latitude, longitude) convert to MGRS
 * (JP x, y) convert to (latitude, longitude) or MGRS
 */

#include <iostream>
#include <string>
#include <tuple>
#include <iomanip>
#include <limits>
#include "mgrs_converter.hpp"

int main(int argc, char **argv)
{
  double lat, lon, alt;
  double x, y, z;
  int plane, test;
  map_tools::MgrsConverter converter;

  std::cout << "choose  1(JP to MGRS) or 2(lat lon to MGRS): ";

  std::cin >> test;

  if (test != 1 && test != 2)
  {
    std::cout << "please input 1 or 2" << std::endl;
    return -1;
  }
  else if (test == 1)
  {
    std::cout << "input x: ";
    std::cin >> x;
    std::cout << "input y: ";
    std::cin >> y;
    std::cout << "input z: ";
    std::cin >> z;
    std::cout << "input plane: ";
    std::cin >> plane;

    converter.jpxy2latlon(x, y, z, plane, lat, lon, alt);

    std::cout << "lat, lon, alt: " << std::setprecision(std::numeric_limits<double>::max_digits10) << lat << ", "
              << std::setprecision(std::numeric_limits<double>::max_digits10) << lon << ", " << alt << std::endl;
  }
  else
  {
    std::cout << "input latitude: ";
    std::cin >> lat;
    std::cout << "input longitude: ";
    std::cin >> lon;
  }

  int zone_num = converter.getUtmZoneNumber(lat, lon);
  std::string zone_letter = converter.getUtmZoneLetter(lat);
  converter.latlon2utm(lat, lon, x, y);

  std::string mgrs_code;
  double easting, northing;

  std::tie(mgrs_code, easting, northing) = converter.latlon2mgrs(lat, lon);

  std::cout << "UTM: " << x << ", " << y << ", " << zone_num << ", " << zone_letter << std::endl;
  std::cout << "MGRS: " << mgrs_code << ", " << easting << ", " << northing << std::endl;

  return 0;
}