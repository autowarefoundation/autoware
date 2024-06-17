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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_HPP_

#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{

/// @brief implementation of a PID controller
class PIDController
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
  double calculate(
    const double error, const double dt, const bool is_integrated,
    std::vector<double> & pid_contributions);
  /**
   * @brief set the coefficients for the P (proportional) I (integral) D (derivative) terms
   * @param [in] kp proportional coefficient
   * @param [in] ki integral coefficient
   * @param [in] kd derivative coefficient
   */
  void setGains(const double kp, const double ki, const double kd);
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
    const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
    const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d);
  /**
   * @brief reset this PID to its initial state
   */
  void reset();

private:
  // PID parameters
  struct Params
  {
    double kp;
    double ki;
    double kd;
    double max_ret_p;
    double min_ret_p;
    double max_ret_i;
    double min_ret_i;
    double max_ret_d;
    double min_ret_d;
    double max_ret;
    double min_ret;
  };
  Params m_params;

  // state variables
  double m_error_integral;
  double m_prev_error;
  bool m_is_first_time;
  bool m_is_gains_set;
  bool m_is_limits_set;
};
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_HPP_
