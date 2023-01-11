// Copyright 2018-2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PID_LONGITUDINAL_CONTROLLER__LOWPASS_FILTER_HPP_
#define PID_LONGITUDINAL_CONTROLLER__LOWPASS_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{

/**
 * @brief Simple filter with gain on the first derivative of the value
 */
class LowpassFilter1d
{
private:
  double m_x;     //!< @brief current filtered value
  double m_gain;  //!< @brief gain value of first-order low-pass filter

public:
  /**
   * @brief constructor with initial value and first-order gain
   * @param [in] x initial value
   * @param [in] gain first-order gain
   */
  LowpassFilter1d(const double x, const double gain) : m_x(x), m_gain(gain) {}

  /**
   * @brief set the current value of the filter
   * @param [in] x new value
   */
  void reset(const double x) { m_x = x; }

  /**
   * @brief get the current value of the filter
   */
  double getValue() const { return m_x; }

  /**
   * @brief filter a new value
   * @param [in] u new value
   */
  double filter(const double u)
  {
    const double ret = m_gain * m_x + (1.0 - m_gain) * u;
    m_x = ret;
    return ret;
  }
};
}  // namespace autoware::motion::control::pid_longitudinal_controller
#endif  // PID_LONGITUDINAL_CONTROLLER__LOWPASS_FILTER_HPP_
