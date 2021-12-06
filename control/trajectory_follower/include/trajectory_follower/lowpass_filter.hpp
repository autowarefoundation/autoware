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

#ifndef TRAJECTORY_FOLLOWER__LOWPASS_FILTER_HPP_
#define TRAJECTORY_FOLLOWER__LOWPASS_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "common/types.hpp"
#include "trajectory_follower/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

/**
 * @brief Simple filter with gain on the first derivative of the value
 */
class LowpassFilter1d
{
private:
  float64_t m_x;     //!< @brief current filtered value
  float64_t m_gain;  //!< @brief gain value of first-order low-pass filter

public:
  /**
   * @brief constructor with initial value and first-order gain
   * @param [in] x initial value
   * @param [in] gain first-order gain
   */
  LowpassFilter1d(const float64_t x, const float64_t gain)
  : m_x(x), m_gain(gain) {}

  /**
   * @brief set the current value of the filter
   * @param [in] x new value
   */
  void reset(const float64_t x) {m_x = x;}

  /**
   * @brief get the current value of the filter
   */
  float64_t getValue() const {return m_x;}

  /**
   * @brief filter a new value
   * @param [in] u new value
   */
  float64_t filter(const float64_t u)
  {
    const float64_t ret = m_gain * m_x + (1.0 - m_gain) * u;
    m_x = ret;
    return ret;
  }
};


/**
 * @brief 2nd-order Butterworth Filter
 * reference : S. Butterworth, "On the Theory of Filter Amplifier", Experimental wireless, 1930.
 */
class TRAJECTORY_FOLLOWER_PUBLIC Butterworth2dFilter
{
private:
  float64_t m_y1;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_y2;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_u1;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_u2;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_a0;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_a1;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_a2;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_b0;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_b1;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  float64_t m_b2;  //!< @brief filter coefficient calculated with cutoff frequency and sampling time

public:
  /**
   * @brief constructor with initialization
   * @param [in] dt sampling time
   * @param [in] f_cutoff_hz cutoff frequency [Hz]
   */
  explicit Butterworth2dFilter(float64_t dt = 0.01, float64_t f_cutoff_hz = 5.0);

  /**
   * @brief destructor
   */
  ~Butterworth2dFilter();

  /**
   * @brief constructor
   * @param [in] dt sampling time
   * @param [in] f_cutoff_hz cutoff frequency [Hz]
   */
  void initialize(const float64_t & dt, const float64_t & f_cutoff_hz);

  /**
   * @brief filtering (call this function at each sampling time with input)
   * @param [in] u scalar input for filter
   * @return filtered scalar value
   */
  float64_t filter(const float64_t & u);

  /**
   * @brief filtering for time-series data
   * @param [in] t time-series data for input vector
   * @param [out] u object vector
   */
  void filt_vector(const std::vector<float64_t> & t, std::vector<float64_t> & u) const;

  /**
   * @brief filtering for time-series data from both forward-backward direction for zero phase delay
   * @param [in] t time-series data for input vector
   * @param [out] u object vector
   */
  void filtfilt_vector(
    const std::vector<float64_t> & t,
    std::vector<float64_t> & u) const;  // filtering forward and backward direction

  /**
   * @brief get filter coefficients
   * @param [out] coeffs coefficients of filter [a0, a1, a2, b0, b1, b2].
   */
  void getCoefficients(std::vector<float64_t> & coeffs) const;
};

/**
 * @brief Move Average Filter
 */
namespace MoveAverageFilter
{
/**
 * @brief filtering vector
 * @param [in] num index distance for moving average filter
 * @param [out] u object vector
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t filt_vector(const int64_t num, std::vector<float64_t> & u);
}  // namespace MoveAverageFilter
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__LOWPASS_FILTER_HPP_
