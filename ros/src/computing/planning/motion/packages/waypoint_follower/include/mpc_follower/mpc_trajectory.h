#pragma once
#include <vector>
#include <iostream>

class MPCTrajectory
{
public:
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<double> yaw;
  std::vector<double> vx;
  std::vector<double> k;
  std::vector<double> relative_time;
  void push_back(const double &xp, const double &yp, const double &zp,
                 const double &yawp, const double &vxp, const double &kp,
                 const double &tp);

  void clear();

  unsigned int size() const;
};
