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
#include <cmath>
#include <vector>
#include <algorithm>

class Butterworth2dFilter
{
private:
  double y1_, y2_, u1_, u2_, a0_, a1_, a2_, b0_, b1_, b2_; // weight parameters set by dt & cutoff_hz

public:
  Butterworth2dFilter(double dt = 0.1, double f_cutoff_hz = 10.0);
  ~Butterworth2dFilter();
  void initialize(const double &dt, const double &f_cutoff_hz);
  double filter(const double &u0);
  void filt_vector(const std::vector<double> &t, std::vector<double> &u);
  void filtfilt_vector(const std::vector<double> &t, std::vector<double> &u); // filtering forward and backward direction
};

class MoveAverageFilter
{
public:
  MoveAverageFilter();
  ~MoveAverageFilter();
  static bool filt_vector(const int num, std::vector<double> &u);
};