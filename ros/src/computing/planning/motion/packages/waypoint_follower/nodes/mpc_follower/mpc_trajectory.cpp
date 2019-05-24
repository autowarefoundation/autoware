/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
  if (x.size() == y.size() && x.size() == z.size() && x.size() == yaw.size() &&
      x.size() == vx.size() && x.size() == k.size() && x.size() == relative_time.size())
  {
    return x.size();
  }
  else
  {
    std::cerr << "[MPC trajectory] trajectory size is inappropriate" << std::endl;
    return 0;
  }
}
