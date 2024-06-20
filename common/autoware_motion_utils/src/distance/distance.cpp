// Copyright 2023 TIER IV, Inc.
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

#include "autoware/motion_utils/distance/distance.hpp"

namespace autoware::motion_utils
{
namespace
{
bool validCheckDecelPlan(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin)
{
  const double v_min = v_target - std::abs(v_margin);
  const double v_max = v_target + std::abs(v_margin);
  const double a_min = a_target - std::abs(a_margin);
  const double a_max = a_target + std::abs(a_margin);

  if (v_end < v_min || v_max < v_end) {
    return false;
  }
  if (a_end < a_min || a_max < a_end) {
    return false;
  }

  return true;
}

/**
 * @brief update traveling distance, velocity and acceleration under constant jerk.
 * @param (x) current traveling distance [m/s]
 * @param (v) current velocity [m/s]
 * @param (a) current acceleration [m/ss]
 * @param (j) target jerk [m/sss]
 * @param (t) time [s]
 * @return updated traveling distance, velocity and acceleration
 */
std::tuple<double, double, double> update(
  const double x, const double v, const double a, const double j, const double t)
{
  const double a_new = a + j * t;
  const double v_new = v + a * t + 0.5 * j * t * t;
  const double x_new = x + v * t + 0.5 * a * t * t + (1.0 / 6.0) * j * t * t * t;

  return {x_new, v_new, a_new};
}

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE: TRAPEZOID
 * ACCELERATION PROFILE). this type of profile has ZERO JERK time.
 *
 * [ACCELERATION PROFILE]
 *  a  ^
 *     |
 *  a0 *
 *     |*
 * ----+-*-------------------*------> t
 *     |  *                 *
 *     |   *               *
 *     | a1 ***************
 *     |
 *
 * [JERK PROFILE]
 *  j  ^
 *     |
 *     |               ja ****
 *     |                  *
 * ----+----***************---------> t
 *     |    *
 *     |    *
 *  jd ******
 *     |
 *
 * @param (v0) current velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (vt) target velocity [m/s]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @param (t_during_min_acc) duration of constant deceleration [s]
 * @return moving distance until velocity is reached vt [m]
 */
std::optional<double> calcDecelDistPlanType1(
  const double v0, const double vt, const double a0, const double am, const double ja,
  const double jd, const double t_during_min_acc)
{
  constexpr double epsilon = 1e-3;

  // negative jerk time
  const double j1 = am < a0 ? jd : ja;
  const double t1 = epsilon < (am - a0) / j1 ? (am - a0) / j1 : 0.0;
  const auto [x1, v1, a1] = update(0.0, v0, a0, j1, t1);

  // zero jerk time
  const double t2 = epsilon < t_during_min_acc ? t_during_min_acc : 0.0;
  const auto [x2, v2, a2] = update(x1, v1, a1, 0.0, t2);

  // positive jerk time
  const double t3 = epsilon < (0.0 - am) / ja ? (0.0 - am) / ja : 0.0;
  const auto [x3, v3, a3] = update(x2, v2, a2, ja, t3);

  const double a_target = 0.0;
  const double v_margin = 0.3;  // [m/s]
  const double a_margin = 0.1;  // [m/s^2]

  if (!validCheckDecelPlan(v3, a3, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x3;
}

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE: TRIANGLE
 * ACCELERATION PROFILE), This type of profile do NOT have ZERO JERK time.
 *
 * [ACCELERATION PROFILE]
 *  a  ^
 *     |
 *  a0 *
 *     |*
 * ----+-*-----*--------------------> t
 *     |  *   *
 *     |   * *
 *     | a1 *
 *     |
 *
 * [JERK PROFILE]
 *  j  ^
 *     |
 *     | ja ****
 *     |    *
 * ----+----*-----------------------> t
 *     |    *
 *     |    *
 *  jd ******
 *     |
 *
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 */
std::optional<double> calcDecelDistPlanType2(
  const double v0, const double vt, const double a0, const double ja, const double jd)
{
  constexpr double epsilon = 1e-3;

  const double a1_square = (vt - v0 - 0.5 * (0.0 - a0) / jd * a0) * (2.0 * ja * jd / (ja - jd));
  const double a1 = -std::sqrt(a1_square);

  // negative jerk time
  const double t1 = epsilon < (a1 - a0) / jd ? (a1 - a0) / jd : 0.0;
  const auto [x1, v1, no_use_a1] = update(0.0, v0, a0, jd, t1);

  // positive jerk time
  const double t2 = epsilon < (0.0 - a1) / ja ? (0.0 - a1) / ja : 0.0;
  const auto [x2, v2, a2] = update(x1, v1, a1, ja, t2);

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;

  if (!validCheckDecelPlan(v2, a2, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x2;
}

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE: LINEAR ACCELERATION
 * PROFILE). This type of profile has only positive jerk time.
 *
 * [ACCELERATION PROFILE]
 *  a  ^
 *     |
 * ----+----*-----------------------> t
 *     |   *
 *     |  *
 *     | *
 *     |*
 *  a0 *
 *     |
 *
 * [JERK PROFILE]
 *  j  ^
 *     |
 *  ja ******
 *     |    *
 *     |    *
 * ----+----*-----------------------> t
 *     |
 *
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 */
std::optional<double> calcDecelDistPlanType3(
  const double v0, const double vt, const double a0, const double ja)
{
  constexpr double epsilon = 1e-3;

  // positive jerk time
  const double t_acc = (0.0 - a0) / ja;
  const double t1 = epsilon < t_acc ? t_acc : 0.0;
  const auto [x1, v1, a1] = update(0.0, v0, a0, ja, t1);

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;

  if (!validCheckDecelPlan(v1, a1, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x1;
}
}  // namespace

std::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec)
{
  if (current_vel < target_vel) {
    return {};
  }

  constexpr double epsilon = 1e-3;
  const double jerk_before_min_acc = acc_min < current_acc ? jerk_dec : jerk_acc;
  const double t_before_min_acc = (acc_min - current_acc) / jerk_before_min_acc;
  const double jerk_after_min_acc = jerk_acc;
  const double t_after_min_acc = (0.0 - acc_min) / jerk_after_min_acc;

  const double t_during_min_acc =
    (target_vel - current_vel - current_acc * t_before_min_acc -
     0.5 * jerk_before_min_acc * std::pow(t_before_min_acc, 2) - acc_min * t_after_min_acc -
     0.5 * jerk_after_min_acc * std::pow(t_after_min_acc, 2)) /
    acc_min;

  // check if it is possible to decelerate to the target velocity
  // by simply bringing the current acceleration to zero.
  const auto is_decel_needed =
    0.5 * (0.0 - current_acc) / jerk_acc * current_acc > target_vel - current_vel;

  if (t_during_min_acc > epsilon) {
    return calcDecelDistPlanType1(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec, t_during_min_acc);
  }
  if (is_decel_needed || current_acc > epsilon) {
    return calcDecelDistPlanType2(current_vel, target_vel, current_acc, jerk_acc, jerk_dec);
  }

  return calcDecelDistPlanType3(current_vel, target_vel, current_acc, jerk_acc);
}
}  // namespace autoware::motion_utils
