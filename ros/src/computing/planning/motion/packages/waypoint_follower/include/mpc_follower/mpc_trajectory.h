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
