#include "mpc_follower/mpc_trajectory.h"

void MPCTrajectory::push_back(const double &xp, const double &yp, const double &zp,
                              const double &yawp, const double &vxp, const double &kp,
                              const double &tp)
{
  x.push_back(xp);
  y.push_back(yp);
  z.push_back(zp);
  yaw.push_back(yawp);
  vx.push_back(vxp);
  k.push_back(kp);
  relative_time.push_back(tp);
};

void MPCTrajectory::clear()
{
  x.clear();
  y.clear();
  z.clear();
  yaw.clear();
  vx.clear();
  k.clear();
  relative_time.clear();
};

unsigned int MPCTrajectory::size() const
{
  unsigned int a = x.size();
  if (a == y.size() && a == z.size() && a == yaw.size() && a == vx.size() &&
      a == k.size() && a == relative_time.size())
  {
    return a;
  }
  else
  {
    std::cout << "[MPC trajectory] trajectory size is inappropriate" << std::endl;
    return 0;
  }
}
