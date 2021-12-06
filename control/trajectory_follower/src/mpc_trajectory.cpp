// Copyright 2018-2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_follower/mpc_trajectory.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
void MPCTrajectory::push_back(
  const float64_t & xp, const float64_t & yp, const float64_t & zp, const float64_t & yawp,
  const float64_t & vxp,
  const float64_t & kp, const float64_t & smooth_kp, const float64_t & tp)
{
  x.push_back(xp);
  y.push_back(yp);
  z.push_back(zp);
  yaw.push_back(yawp);
  vx.push_back(vxp);
  k.push_back(kp);
  smooth_k.push_back(smooth_kp);
  relative_time.push_back(tp);
}

void MPCTrajectory::clear()
{
  x.clear();
  y.clear();
  z.clear();
  yaw.clear();
  vx.clear();
  k.clear();
  smooth_k.clear();
  relative_time.clear();
}

size_t MPCTrajectory::size() const
{
  if (
    x.size() == y.size() && x.size() == z.size() && x.size() == yaw.size() &&
    x.size() == vx.size() && x.size() == k.size() && x.size() == smooth_k.size() &&
    x.size() == relative_time.size())
  {
    return x.size();
  } else {
    std::cerr << "[MPC trajectory] trajectory size is inappropriate" << std::endl;
    return 0;
  }
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
