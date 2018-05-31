/*
 * The rights of this source code conform to
 * https://github.com/CPFL/Autoware/blob/master/LICENSE
 */

#ifndef MGRS_CONVERTER_HPP
#define MGRS_CONVERTER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <tuple>

namespace map_tools
{
class MgrsConverter
{
public:
  MgrsConverter();
  ~MgrsConverter();

  std::string getUtmZoneLetter(const double& lat);
  int getUtmZoneNumber(const double& lat, const double& lon);
  void latlon2utm(const double& lat, const double& lon, double& x, double& y);
  std::tuple<std::string, double, double> latlon2mgrs(double lat, double lon);
  void jpxy2latlon(const double& x, const double& y, const double& z, const int& plane, double& lat, double& lon,
                   double& alt);

private:
  double K0_;

  double E_;
  double E2_;
  double E3_;
  double E_P2_;

  double SQRT_E_;
  double Es_;
  double Es2_;
  double Es3_;
  double Es4_;
  double Es5_;

  double M1_;
  double M2_;
  double M3_;
  double M4_;

  double R_;

  double P2_;
  double P3_;
  double P4_;
  double P5_;

  std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  int lat_band[20] = { 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20, 21, 22, 23 };

  double getRadian(const double& deg);
  double getCentralLongitude(const int& zone_num);
  void setPlaneRef(int num, double &lat_0, double &lon_0);
};
}
#endif  // MGRS_CONVERTER_HPP
