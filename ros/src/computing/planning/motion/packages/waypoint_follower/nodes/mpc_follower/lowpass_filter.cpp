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

#include "mpc_follower/lowpass_filter.h"

Butterworth2dFilter::Butterworth2dFilter(double dt, double f_cutoff_hz)
{
  initialize(dt, f_cutoff_hz);
};

Butterworth2dFilter::~Butterworth2dFilter(){};

void Butterworth2dFilter::initialize(const double &dt, const double &f_cutoff_hz)
{
  y1_ = 0.0;
  y2_ = 0.0;
  u2_ = 0.0;
  u1_ = 0.0;

  /* 2d butterworth lowpass filter with bi-linear transformation */
  double wc = 2.0 * M_PI * f_cutoff_hz;
  double n = 2 / dt;
  a0_ = n * n + sqrt(2) * wc * n + wc * wc;
  a1_ = 2 * wc * wc - 2 * n * n;
  a2_ = n * n - sqrt(2) * wc * n + wc * wc;
  b0_ = wc * wc;
  b1_ = 2 * b0_;
  b2_ = b0_;
}

double Butterworth2dFilter::filter(const double &u0)
{
  double y0 = (b2_ * u2_ + b1_ * u1_ + b0_ * u0 - a2_ * y2_ - a1_ * y1_) / a0_;
  y2_ = y1_;
  y1_ = y0;
  u2_ = u1_;
  u1_ = u0;
  return y0;
}

void Butterworth2dFilter::filt_vector(const std::vector<double> &t, std::vector<double> &u)
{
  double y1 = u.at(0);
  double y2 = u.at(0);
  double u2 = u.at(0);
  double u1 = u.at(0);
  double y0 = 0.0;
  double u0 = 0.0;
  for (unsigned int i = 0; i < u.size(); ++i)
  {
    u0 = u.at(i);
    y0 = (b2_ * u2 + b1_ * u1 + b0_ * u0 - a2_ * y2 - a1_ * y1) / a0_;
    y2 = y1;
    y1 = y0;
    u2 = u1;
    u1 = u0;
    u.at(i) = y0;
  }
}

// filtering forward and backward direction
void Butterworth2dFilter::filtfilt_vector(const std::vector<double> &t, std::vector<double> &u)
{
  std::vector<double> u_rev(u);

  // forward filtering
  filt_vector(t, u);

  // backward filtering
  std::reverse(u_rev.begin(), u_rev.end());
  filt_vector(t, u_rev);
  std::reverse(u_rev.begin(), u_rev.end());

  // merge
  for (unsigned int i = 0; i < u.size(); ++i)
  {
    u[i] = (u[i] + u_rev[i]) * 0.5;
  }
}

void Butterworth2dFilter::getCoefficients(std::vector<double> &coeffs)
{
  coeffs.clear();
  coeffs.push_back(a0_);
  coeffs.push_back(a1_);
  coeffs.push_back(a2_);
  coeffs.push_back(b0_);
  coeffs.push_back(b1_);
  coeffs.push_back(b2_);
  return;
}

bool MoveAverageFilter::filt_vector(const int num, std::vector<double> &u)
{

  if ((int)u.size() < num)
  {
    printf("[MovingAverageFilter] vector size is lower than moving average number\n");
    return false;
  }
  std::vector<double> filtered_u(u);
  for (int i = 0; i < (int)u.size(); ++i)
  {
    double tmp = 0.0;
    int num_tmp = 0;
    int count = 0;
    if (i - num < 0)
    {
      num_tmp = i;
    }
    else if (i + num > (int)u.size() - 1)
    {
      num_tmp = (int)u.size() - i - 1;
    }
    else
    {
      num_tmp = num;
    }

    for (int j = -num_tmp; j <= num_tmp; ++j)
    {
      tmp += u[i + j];
      ++count;
    }
    filtered_u[i] = tmp / count;
  }
  u = filtered_u;
  return true;
}