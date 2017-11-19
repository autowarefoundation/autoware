//
// Created by gpu on 17/11/19.
//

#ifndef UTM_CONVERTER_HPP
#define UTM_CONVERTER_HPP


#include <string>
#include <tuple>

class UtmConverter
{
public:
  UtmConverter();
  ~UtmConverter();

  std::tuple<double, double, int, std::string> latlng2utm(double lat, double lng);

private:
  double K0_;

  double E_;
  double E2_;
  double E3_;
  double E_P2_;

  double SQRT_E_;
  double _E;
  double _E2;
  double _E3;
  double _E4;
  double _E5;

  double M1_;
  double M2_;
  double M3_;
  double M4_;

  double R_;

  double P2_;
  double P3_;
  double P4_;
  double P5_;

  double getRadian(double deg);
  int getZoneNumber(double lat, double lng);
  double getCentralLongitude(int zone_num);
  std::string getZoneLetter(double lat);
};

#endif  // UTM_CONVERTER_HPP
