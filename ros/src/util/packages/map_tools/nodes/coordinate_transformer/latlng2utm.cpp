/*
 * The rights of this source code conform to
 * https://github.com/CPFL/Autoware/blob/master/LICENSE
 *
 * (latitude, longitude) convert to UTM(x,y,zone num, zone letter)
 *
 */

#include "utm_converter.hpp"
#include <iostream>
#include <string>
#include <tuple>

int main(int argc, char **argv) {
  double lat, lng;
  double x, y;
  int zone_num;
  std::string zone_letter;

  UtmConverter converter;

  std::cout << "input latitude: ";
  std::cin >> lat;
  std::cout << "input longitude: ";
  std::cin >> lng;

  std::tie(x, y, zone_num, zone_letter) = converter.latlng2utm(lat, lng);
  std::cout << x << ", " << y << ", " << zone_num << ", " << zone_letter
            << std::endl;

  return 0;
}