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

#include "trajectory_follower/lowpass_filter.hpp"

#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
Butterworth2dFilter::Butterworth2dFilter(float64_t dt, float64_t f_cutoff_hz)
{
  initialize(dt, f_cutoff_hz);
}

Butterworth2dFilter::~Butterworth2dFilter() {}

void Butterworth2dFilter::initialize(const float64_t & dt, const float64_t & f_cutoff_hz)
{
  m_y1 = 0.0;
  m_y2 = 0.0;
  m_u2 = 0.0;
  m_u1 = 0.0;

  /* 2d butterworth lowpass filter with bi-linear transformation */
  float64_t wc = 2.0 * M_PI * f_cutoff_hz;
  float64_t n = 2 / dt;
  m_a0 = n * n + sqrt(2) * wc * n + wc * wc;
  m_a1 = 2 * wc * wc - 2 * n * n;
  m_a2 = n * n - sqrt(2) * wc * n + wc * wc;
  m_b0 = wc * wc;
  m_b1 = 2 * m_b0;
  m_b2 = m_b0;
}

float64_t Butterworth2dFilter::filter(const float64_t & u0)
{
  float64_t y0 = (m_b2 * m_u2 + m_b1 * m_u1 + m_b0 * u0 - m_a2 * m_y2 - m_a1 * m_y1) / m_a0;
  m_y2 = m_y1;
  m_y1 = y0;
  m_u2 = m_u1;
  m_u1 = u0;
  return y0;
}

void Butterworth2dFilter::filt_vector(
  const std::vector<float64_t> & t,
  std::vector<float64_t> & u) const
{
  u.resize(t.size());
  float64_t y1 = t.at(0);
  float64_t y2 = t.at(0);
  float64_t u2 = t.at(0);
  float64_t u1 = t.at(0);
  float64_t y0 = 0.0;
  float64_t u0 = 0.0;
  for (size_t i = 0; i < t.size(); ++i) {
    u0 = t.at(i);
    y0 = (m_b2 * u2 + m_b1 * u1 + m_b0 * u0 - m_a2 * y2 - m_a1 * y1) / m_a0;
    y2 = y1;
    y1 = y0;
    u2 = u1;
    u1 = u0;
    u.at(i) = y0;
  }
}

// filtering forward and backward direction
void Butterworth2dFilter::filtfilt_vector(
  const std::vector<float64_t> & t,
  std::vector<float64_t> & u) const
{
  std::vector<float64_t> t_fwd(t);
  std::vector<float64_t> t_rev(t);

  // forward filtering
  filt_vector(t, t_fwd);

  // backward filtering
  std::reverse(t_rev.begin(), t_rev.end());
  filt_vector(t, t_rev);
  std::reverse(t_rev.begin(), t_rev.end());

  // merge
  u.clear();
  for (size_t i = 0; i < t.size(); ++i) {
    u.push_back((t_fwd[i] + t_rev[i]) * 0.5);
  }
}

void Butterworth2dFilter::getCoefficients(std::vector<float64_t> & coeffs) const
{
  coeffs.clear();
  coeffs.push_back(m_a0);
  coeffs.push_back(m_a1);
  coeffs.push_back(m_a2);
  coeffs.push_back(m_b0);
  coeffs.push_back(m_b1);
  coeffs.push_back(m_b2);
}

namespace MoveAverageFilter
{
bool8_t filt_vector(const int64_t num, std::vector<float64_t> & u)
{
  if (static_cast<int64_t>(u.size()) < num) {
    return false;
  }
  std::vector<float64_t> filtered_u(u);
  for (int64_t i = 0; i < static_cast<int64_t>(u.size()); ++i) {
    float64_t tmp = 0.0;
    int64_t num_tmp = 0;
    float64_t count = 0;
    if (i - num < 0) {
      num_tmp = i;
    } else if (i + num > static_cast<int64_t>(u.size()) - 1) {
      num_tmp = static_cast<int64_t>(u.size()) - i - 1;
    } else {
      num_tmp = num;
    }

    for (int64_t j = -num_tmp; j <= num_tmp; ++j) {
      tmp += u[static_cast<size_t>(i + j)];
      ++count;
    }
    filtered_u[static_cast<size_t>(i)] = tmp / count;
  }
  u = filtered_u;
  return true;
}
}  // namespace MoveAverageFilter
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
