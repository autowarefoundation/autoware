// Copyright 2018-2021 Tier IV, Inc.
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

#ifndef TRAJECTORY_FOLLOWER__PID_HPP_
#define TRAJECTORY_FOLLOWER__PID_HPP_

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
/// @brief implementation of a PID controller
class TRAJECTORY_FOLLOWER_PUBLIC PIDController
{
public:
  PIDController();

  /**
   * @brief calculate the output of this PID
   * @param [in] error previous error
   * @param [in] dt time step [s]
   * @param [in] is_integrated if true, will use the integral component for calculation
   * @param [out] pid_contributions values of the proportional, integral, and derivative components
   * @return PID output
   * @throw std::runtime_error if gains or limits have not been set
   */
  float64_t calculate(
    const float64_t error, const float64_t dt, const bool8_t is_integrated,
    std::vector<float64_t> & pid_contributions);
  /**
   * @brief set the coefficients for the P (proportional) I (integral) D (derivative) terms
   * @param [in] kp proportional coefficient
   * @param [in] ki integral coefficient
   * @param [in] kd derivative coefficient
   */
  void setGains(const float64_t kp, const float64_t ki, const float64_t kd);
  /**
   * @brief set limits on the total, proportional, integral, and derivative components
   * @param [in] max_ret maximum return value of this PID
   * @param [in] min_ret minimum return value of this PID
   * @param [in] max_ret_p maximum value of the proportional component
   * @param [in] min_ret_p minimum value of the proportional component
   * @param [in] max_ret_i maximum value of the integral component
   * @param [in] min_ret_i minimum value of the integral component
   * @param [in] max_ret_d maximum value of the derivative component
   * @param [in] min_ret_d minimum value of the derivative component
   */
  void setLimits(
    const float64_t max_ret, const float64_t min_ret, const float64_t max_ret_p,
    const float64_t min_ret_p,
    const float64_t max_ret_i, const float64_t min_ret_i, const float64_t max_ret_d,
    const float64_t min_ret_d);
  /**
   * @brief reset this PID to its initial state
   */
  void reset();

private:
  // PID parameters
  struct Params
  {
    float64_t kp;
    float64_t ki;
    float64_t kd;
    float64_t max_ret_p;
    float64_t min_ret_p;
    float64_t max_ret_i;
    float64_t min_ret_i;
    float64_t max_ret_d;
    float64_t min_ret_d;
    float64_t max_ret;
    float64_t min_ret;
  };
  Params m_params;

  // state variables
  float64_t m_error_integral;
  float64_t m_prev_error;
  bool8_t m_is_first_time;
  bool8_t m_is_gains_set;
  bool8_t m_is_limits_set;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__PID_HPP_
