// Copyright 2021 Tier IV, Inc.
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

#include "../../src/vehicle_cmd_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#define THRESHOLD 1.0e-5
#define ASSERT_LT_NEAR(x, y) ASSERT_LT(x, y + THRESHOLD)
#define ASSERT_GT_NEAR(x, y) ASSERT_GT(x, y - THRESHOLD)

using autoware::vehicle_cmd_gate::LimitArray;
using autoware_control_msgs::msg::Control;

constexpr double NOMINAL_INTERVAL = 1.0;

void setFilterParams(
  autoware::vehicle_cmd_gate::VehicleCmdFilter & f, double v, LimitArray speed_points, LimitArray a,
  LimitArray j, LimitArray lat_a, LimitArray lat_j, LimitArray steer_diff, LimitArray steer_lim,
  LimitArray steer_rate_lim, const double wheelbase)
{
  autoware::vehicle_cmd_gate::VehicleCmdFilterParam p;
  p.vel_lim = v;
  p.wheel_base = wheelbase;
  p.reference_speed_points = speed_points;
  p.steer_lim = steer_lim;
  p.steer_rate_lim = steer_rate_lim;
  p.lat_acc_lim = lat_a;
  p.lat_jerk_lim = lat_j;
  p.lon_acc_lim = a;
  p.lon_jerk_lim = j;
  p.actual_steer_diff_lim = steer_diff;

  f.setParam(p);
}

Control genCmd(double s, double sr, double v, double a)
{
  Control cmd;
  cmd.lateral.steering_tire_angle = s;
  cmd.lateral.steering_tire_rotation_rate = sr;
  cmd.longitudinal.velocity = v;
  cmd.longitudinal.acceleration = a;
  return cmd;
}

// calc from ego velocity
double calcLatAcc(const Control & cmd, const double wheelbase, const double ego_v)
{
  return ego_v * ego_v * std::tan(cmd.lateral.steering_tire_angle) / wheelbase;
}

// calc from command velocity
double calcLatAcc(const Control & cmd, const double wheelbase)
{
  double v = cmd.longitudinal.velocity;
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / wheelbase;
}

// calc from command velocity
double calcLatJerk(
  const Control & cmd, const Control & prev_cmd, const double wheelbase, const double dt)
{
  const auto prev_v = prev_cmd.longitudinal.velocity;
  const auto prev = prev_v * prev_v * std::tan(prev_cmd.lateral.steering_tire_angle) / wheelbase;

  const auto curr_v = cmd.longitudinal.velocity;
  const auto curr = curr_v * curr_v * std::tan(cmd.lateral.steering_tire_angle) / wheelbase;

  return (curr - prev) / dt;
}

// calc from ego velocity
double calcLatJerk(
  const Control & cmd, const Control & prev_cmd, const double wheelbase, const double dt,
  const double ego_v)
{
  const auto prev = ego_v * ego_v * std::tan(prev_cmd.lateral.steering_tire_angle) / wheelbase;

  const auto curr = ego_v * ego_v * std::tan(cmd.lateral.steering_tire_angle) / wheelbase;

  return (curr - prev) / dt;
}

void test_1d_limit(
  double ego_v, double V_LIM, double A_LIM, double J_LIM, double LAT_A_LIM, double LAT_J_LIM,
  double STEER_DIFF, double STEER_LIM, double STEER_RATE_LIM, const Control & prev_cmd,
  const Control & raw_cmd)
{
  const double WHEELBASE = 3.0;
  const double DT = 0.1;  // [s]

  autoware::vehicle_cmd_gate::VehicleCmdFilter filter;
  filter.setCurrentSpeed(ego_v);
  setFilterParams(
    filter, V_LIM, {0.0}, {A_LIM}, {J_LIM}, {LAT_A_LIM}, {LAT_J_LIM}, {STEER_DIFF}, {STEER_LIM},
    {STEER_RATE_LIM}, WHEELBASE);
  filter.setPrevCmd(prev_cmd);

  // velocity filter
  {
    auto filtered_cmd = raw_cmd;
    filter.limitLongitudinalWithVel(filtered_cmd);

    // check if the filtered value does not exceed the limit.
    ASSERT_LT_NEAR(filtered_cmd.longitudinal.velocity, V_LIM);

    // check if the undesired filter is not applied.
    if (std::abs(raw_cmd.longitudinal.velocity) < V_LIM) {
      ASSERT_NEAR(filtered_cmd.longitudinal.velocity, raw_cmd.longitudinal.velocity, THRESHOLD);
    }
  }

  // acceleration filter
  {
    auto filtered_cmd = raw_cmd;
    filter.limitLongitudinalWithAcc(DT, filtered_cmd);

    // check if the filtered value does not exceed the limit.
    ASSERT_LT_NEAR(filtered_cmd.longitudinal.acceleration, A_LIM);
    ASSERT_GT_NEAR(filtered_cmd.longitudinal.acceleration, -A_LIM);

    // check if the undesired filter is not applied.
    if (
      -A_LIM < filtered_cmd.longitudinal.acceleration &&
      filtered_cmd.longitudinal.acceleration < A_LIM) {
      ASSERT_NEAR(
        filtered_cmd.longitudinal.acceleration, raw_cmd.longitudinal.acceleration, THRESHOLD);
    }

    // check if the filtered value does not exceed the limit.
    const double v_max = prev_cmd.longitudinal.velocity + A_LIM * DT;
    const double v_min = prev_cmd.longitudinal.velocity - A_LIM * DT;
    ASSERT_LT_NEAR(filtered_cmd.longitudinal.velocity, v_max);
    ASSERT_GT_NEAR(filtered_cmd.longitudinal.velocity, v_min);

    // check if the undesired filter is not applied.
    if (v_min < raw_cmd.longitudinal.velocity && raw_cmd.longitudinal.velocity < v_max) {
      ASSERT_NEAR(filtered_cmd.longitudinal.velocity, raw_cmd.longitudinal.velocity, THRESHOLD);
    }
  }

  // jerk filter
  {
    auto filtered_cmd = raw_cmd;
    filter.limitLongitudinalWithJerk(DT, filtered_cmd);
    const double acc_max = prev_cmd.longitudinal.acceleration + J_LIM * DT;
    const double acc_min = prev_cmd.longitudinal.acceleration - J_LIM * DT;

    // check if the filtered value does not exceed the limit.
    ASSERT_LT_NEAR(filtered_cmd.longitudinal.acceleration, acc_max);
    ASSERT_GT_NEAR(filtered_cmd.longitudinal.acceleration, acc_min);

    // check if the undesired filter is not applied.
    if (
      acc_min < raw_cmd.longitudinal.acceleration && raw_cmd.longitudinal.acceleration < acc_max) {
      ASSERT_NEAR(
        filtered_cmd.longitudinal.acceleration, raw_cmd.longitudinal.acceleration, THRESHOLD);
    }
  }

  // lateral acceleration filter
  {
    auto filtered_cmd = raw_cmd;
    filter.limitLateralWithLatAcc(DT, filtered_cmd);
    const double filtered_latacc = calcLatAcc(filtered_cmd, WHEELBASE, ego_v);
    const double raw_latacc = calcLatAcc(raw_cmd, WHEELBASE, ego_v);

    // check if the filtered value does not exceed the limit.
    ASSERT_LT_NEAR(std::abs(filtered_latacc), LAT_A_LIM);

    // check if the undesired filter is not applied.
    if (std::abs(raw_latacc) < LAT_A_LIM) {
      ASSERT_NEAR(filtered_latacc, raw_latacc, THRESHOLD);
    }
  }

  // lateral jerk filter
  {
    auto filtered_cmd = raw_cmd;
    filter.limitLateralWithLatJerk(DT, filtered_cmd);
    const double prev_lat_acc = calcLatAcc(prev_cmd, WHEELBASE, ego_v);
    const double filtered_lat_acc = calcLatAcc(filtered_cmd, WHEELBASE, ego_v);
    const double raw_lat_acc = calcLatAcc(raw_cmd, WHEELBASE, ego_v);
    const double filtered_lateral_jerk = (filtered_lat_acc - prev_lat_acc) / DT;

    // check if the filtered value does not exceed the limit.
    ASSERT_LT_NEAR(std::abs(filtered_lateral_jerk), LAT_J_LIM);

    // check if the undesired filter is not applied.
    const double raw_lateral_jerk = (raw_lat_acc - prev_lat_acc) / DT;
    if (std::abs(raw_lateral_jerk) < LAT_J_LIM) {
      ASSERT_NEAR(
        filtered_cmd.lateral.steering_tire_angle, raw_cmd.lateral.steering_tire_angle, THRESHOLD);
    }
  }

  // steer diff
  {
    const auto current_steering = 0.1;
    auto filtered_cmd = raw_cmd;
    filter.limitActualSteerDiff(current_steering, filtered_cmd);
    const auto filtered_steer_diff = filtered_cmd.lateral.steering_tire_angle - current_steering;
    const auto raw_steer_diff = raw_cmd.lateral.steering_tire_angle - current_steering;
    // check if the filtered value does not exceed the limit.
    ASSERT_LT_NEAR(std::abs(filtered_steer_diff), STEER_DIFF);

    // check if the undesired filter is not applied.
    if (std::abs(raw_steer_diff) < STEER_DIFF) {
      ASSERT_NEAR(
        filtered_cmd.lateral.steering_tire_angle, raw_cmd.lateral.steering_tire_angle, THRESHOLD);
    }
  }
}

TEST(VehicleCmdFilter, VehicleCmdFilter)
{
  const std::vector<double> v_arr = {0.0, 1.0, 100.0};
  const std::vector<double> a_arr = {0.0, 1.0, 100.0};
  const std::vector<double> j_arr = {0.0, 0.1, 1.0};
  const std::vector<double> lat_a_arr = {0.01, 1.0, 100.0};
  const std::vector<double> lat_j_arr = {0.01, 1.0, 100.0};
  const std::vector<double> steer_diff_arr = {0.01, 1.0, 100.0};
  const std::vector<double> steer_lim_arr = {0.01, 1.0, 100.0};
  const std::vector<double> steer_rate_lim_arr = {0.01, 1.0, 100.0};
  const std::vector<double> ego_v_arr = {0.0, 0.1, 1.0, 3.0, 15.0};

  const std::vector<Control> prev_cmd_arr = {
    genCmd(0.0, 0.0, 0.0, 0.0), genCmd(1.0, 1.0, 1.0, 1.0)};

  const std::vector<Control> raw_cmd_arr = {
    genCmd(1.0, 1.0, 1.0, 1.0), genCmd(10.0, -1.0, -1.0, -1.0)};

  for (const auto & v : v_arr) {
    for (const auto & a : a_arr) {
      for (const auto & j : j_arr) {
        for (const auto & la : lat_a_arr) {
          for (const auto & lj : lat_j_arr) {
            for (const auto & prev_cmd : prev_cmd_arr) {
              for (const auto & raw_cmd : raw_cmd_arr) {
                for (const auto & steer_diff : steer_diff_arr) {
                  for (const auto & steer : steer_lim_arr) {
                    for (const auto & steer_rate : steer_rate_lim_arr) {
                      for (const auto & ego_v : ego_v_arr) {
                        test_1d_limit(
                          ego_v, v, a, j, la, lj, steer_diff, steer, steer_rate, prev_cmd, raw_cmd);
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

TEST(VehicleCmdFilter, VehicleCmdFilterInterpolate)
{
  constexpr double WHEELBASE = 2.8;
  autoware::vehicle_cmd_gate::VehicleCmdFilter filter;

  autoware::vehicle_cmd_gate::VehicleCmdFilterParam p;
  p.wheel_base = WHEELBASE;
  p.vel_lim = 20.0;
  p.reference_speed_points = std::vector<double>{2.0, 4.0, 10.0};
  p.steer_lim = std::vector<double>{0.1, 0.2, 0.3};
  p.steer_rate_lim = std::vector<double>{0.2, 0.1, 0.05};
  p.lon_acc_lim = std::vector<double>{0.3, 0.4, 0.5};
  p.lon_jerk_lim = std::vector<double>{0.4, 0.4, 0.7};
  p.lat_acc_lim = std::vector<double>{0.1, 0.2, 0.3};
  p.lat_jerk_lim = std::vector<double>{0.9, 0.7, 0.1};
  p.actual_steer_diff_lim = std::vector<double>{0.1, 0.3, 0.2};
  filter.setParam(p);

  const auto DT = 0.033;

  const auto orig_cmd = []() {
    Control cmd;
    cmd.lateral.steering_tire_angle = 0.5;
    cmd.lateral.steering_tire_rotation_rate = 0.5;
    cmd.longitudinal.velocity = 30.0;
    cmd.longitudinal.acceleration = 10.0;
    cmd.longitudinal.jerk = 10.0;
    return cmd;
  }();

  const auto set_speed_and_reset_prev = [&](const auto & current_vel) {
    filter.setCurrentSpeed(current_vel);
  };
  const auto _limitSteer = [&](const auto & in) {
    auto out = in;
    filter.limitLateralSteer(out);
    return out;
  };
  const auto _limitSteerRate = [&](const auto & in) {
    auto out = in;
    filter.limitLateralSteerRate(DT, out);
    return out;
  };
  const auto _limitLongitudinalWithVel = [&](const auto & in) {
    auto out = in;
    filter.limitLongitudinalWithVel(out);
    return out;
  };
  const auto _limitLongitudinalWithAcc = [&](const auto & in) {
    auto out = in;
    filter.limitLongitudinalWithAcc(DT, out);
    return out;
  };
  const auto _limitLongitudinalWithJerk = [&](const auto & in) {
    auto out = in;
    filter.limitLongitudinalWithJerk(DT, out);
    return out;
  };
  const auto _limitLateralWithLatAcc = [&](const auto & in) {
    auto out = in;
    filter.limitLateralWithLatAcc(DT, out);
    return out;
  };
  const auto _limitLateralWithLatJerk = [&](const auto & in) {
    auto out = in;
    filter.limitLateralWithLatJerk(DT, out);
    return out;
  };
  const auto _limitActualSteerDiff = [&](const auto & in) {
    auto out = in;
    const auto prev_steering = 0.0;
    filter.limitActualSteerDiff(prev_steering, out);
    return out;
  };

  constexpr double ep = 1.0e-5;

  // vel lim
  {
    set_speed_and_reset_prev(0.0);
    EXPECT_NEAR(_limitLongitudinalWithVel(orig_cmd).longitudinal.velocity, 20.0, ep);
  }

  // steer angle lim
  // p.reference_speed_points = std::vector<double>{2.0, 4.0, 10.0};
  // p.steer_lim = std::vector<double>{0.1, 0.2, 0.3};
  {
    set_speed_and_reset_prev(0.0);
    EXPECT_NEAR(_limitSteer(orig_cmd).lateral.steering_tire_angle, 0.1, ep);

    set_speed_and_reset_prev(2.0);
    EXPECT_NEAR(_limitSteer(orig_cmd).lateral.steering_tire_angle, 0.1, ep);

    set_speed_and_reset_prev(3.0);
    EXPECT_NEAR(_limitSteer(orig_cmd).lateral.steering_tire_angle, 0.15, ep);

    set_speed_and_reset_prev(5.0);
    EXPECT_NEAR(_limitSteer(orig_cmd).lateral.steering_tire_angle, 0.2 + 0.1 / 6.0, ep);

    set_speed_and_reset_prev(8.0);
    EXPECT_NEAR(_limitSteer(orig_cmd).lateral.steering_tire_angle, 0.2 + 0.1 * 4.0 / 6.0, ep);

    set_speed_and_reset_prev(10.0);
    EXPECT_NEAR(_limitSteer(orig_cmd).lateral.steering_tire_angle, 0.3, ep);

    set_speed_and_reset_prev(15.0);
    EXPECT_NEAR(_limitSteer(orig_cmd).lateral.steering_tire_angle, 0.3, ep);
  }

  // steer angle rate lim
  // p.reference_speed_points = std::vector<double>{2.0, 4.0, 10.0};
  // p.steer_rate_lim = std::vector<double>{0.2, 0.1, 0.05};
  {
    const auto calcSteerRateFromAngle = [&](const auto & cmd) {
      return (cmd.steering_tire_angle - 0.0) / DT;
    };
    autoware_control_msgs::msg::Lateral filtered;

    set_speed_and_reset_prev(0.0);
    filtered = _limitSteerRate(orig_cmd).lateral;
    EXPECT_NEAR(calcSteerRateFromAngle(filtered), 0.2, ep);
    EXPECT_NEAR(filtered.steering_tire_rotation_rate, 0.2, ep);

    set_speed_and_reset_prev(2.0);
    filtered = _limitSteerRate(orig_cmd).lateral;
    EXPECT_NEAR(calcSteerRateFromAngle(filtered), 0.2, ep);
    EXPECT_NEAR(filtered.steering_tire_rotation_rate, 0.2, ep);

    set_speed_and_reset_prev(3.0);
    filtered = _limitSteerRate(orig_cmd).lateral;
    EXPECT_NEAR(calcSteerRateFromAngle(filtered), 0.15, ep);
    EXPECT_NEAR(filtered.steering_tire_rotation_rate, 0.15, ep);

    set_speed_and_reset_prev(5.0);
    filtered = _limitSteerRate(orig_cmd).lateral;
    EXPECT_NEAR(calcSteerRateFromAngle(filtered), 0.1 - 0.05 * 1.0 / 6.0, ep);
    EXPECT_NEAR(filtered.steering_tire_rotation_rate, 0.1 - 0.05 * 1.0 / 6.0, ep);

    set_speed_and_reset_prev(8.0);
    filtered = _limitSteerRate(orig_cmd).lateral;
    EXPECT_NEAR(calcSteerRateFromAngle(filtered), 0.1 - 0.05 * 4.0 / 6.0, ep);
    EXPECT_NEAR(filtered.steering_tire_rotation_rate, 0.1 - 0.05 * 4.0 / 6.0, ep);

    set_speed_and_reset_prev(10.0);
    filtered = _limitSteerRate(orig_cmd).lateral;
    EXPECT_NEAR(calcSteerRateFromAngle(filtered), 0.05, ep);
    EXPECT_NEAR(filtered.steering_tire_rotation_rate, 0.05, ep);

    set_speed_and_reset_prev(15.0);
    filtered = _limitSteerRate(orig_cmd).lateral;
    EXPECT_NEAR(calcSteerRateFromAngle(filtered), 0.05, ep);
    EXPECT_NEAR(filtered.steering_tire_rotation_rate, 0.05, ep);
  }

  // longitudinal acc lim
  {
    set_speed_and_reset_prev(0.0);
    EXPECT_NEAR(_limitLongitudinalWithAcc(orig_cmd).longitudinal.acceleration, 0.3, ep);

    set_speed_and_reset_prev(2.0);
    EXPECT_NEAR(_limitLongitudinalWithAcc(orig_cmd).longitudinal.acceleration, 0.3, ep);

    set_speed_and_reset_prev(3.0);
    EXPECT_NEAR(_limitLongitudinalWithAcc(orig_cmd).longitudinal.acceleration, 0.35, ep);

    set_speed_and_reset_prev(5.0);
    EXPECT_NEAR(_limitLongitudinalWithAcc(orig_cmd).longitudinal.acceleration, 0.4 + 0.1 / 6.0, ep);

    set_speed_and_reset_prev(8.0);
    EXPECT_NEAR(_limitLongitudinalWithAcc(orig_cmd).longitudinal.acceleration, 0.4 + 0.4 / 6.0, ep);

    set_speed_and_reset_prev(10.0);
    EXPECT_NEAR(_limitLongitudinalWithAcc(orig_cmd).longitudinal.acceleration, 0.5, ep);

    set_speed_and_reset_prev(15.0);
    EXPECT_NEAR(_limitLongitudinalWithAcc(orig_cmd).longitudinal.acceleration, 0.5, ep);
  }

  // longitudinal jerk lim
  // p.reference_speed_points = std::vector<double>{2.0, 4.0, 10.0};
  // p.lon_jerk_lim = std::vector<double>{0.4, 0.4, 0.7};
  {
    set_speed_and_reset_prev(0.0);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.jerk, 0.4, ep);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.acceleration, DT * 0.4, ep);

    set_speed_and_reset_prev(2.0);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.jerk, 0.4, ep);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.acceleration, DT * 0.4, ep);

    set_speed_and_reset_prev(3.0);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.jerk, 0.4, ep);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.acceleration, DT * 0.4, ep);

    set_speed_and_reset_prev(5.0);
    const auto expect_v5 = 0.4 + 0.3 / 6.0;
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.jerk, expect_v5, ep);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.acceleration, DT * expect_v5, ep);

    set_speed_and_reset_prev(8.0);
    const auto expect_v8 = 0.4 + 1.2 / 6.0;
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.jerk, expect_v8, ep);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.acceleration, DT * expect_v8, ep);

    set_speed_and_reset_prev(10.0);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.jerk, 0.7, ep);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.acceleration, DT * 0.7, ep);

    set_speed_and_reset_prev(15.0);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.jerk, 0.7, ep);
    EXPECT_NEAR(_limitLongitudinalWithJerk(orig_cmd).longitudinal.acceleration, DT * 0.7, ep);
  }

  // lateral acc lim
  // p.reference_speed_points = std::vector<double>{2.0, 4.0, 10.0};
  // p.lat_acc_lim = std::vector<double>{0.1, 0.2, 0.3};
  const auto _calcLatAcc = [&](const auto & cmd, const double ego_v) {
    return calcLatAcc(cmd, WHEELBASE, ego_v);
  };
  {
    // since the lateral acceleration is 0 when the velocity is 0, the target value is 0 only in
    // this case
    set_speed_and_reset_prev(0.0);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatAcc(orig_cmd), 0.0), 0.0, ep);

    set_speed_and_reset_prev(2.0);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatAcc(orig_cmd), 2.0), 0.1, ep);

    set_speed_and_reset_prev(3.0);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatAcc(orig_cmd), 3.0), 0.15, ep);

    set_speed_and_reset_prev(5.0);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatAcc(orig_cmd), 5.0), 0.2 + 0.1 / 6.0, ep);

    set_speed_and_reset_prev(8.0);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatAcc(orig_cmd), 8.0), 0.2 + 0.4 / 6.0, ep);

    set_speed_and_reset_prev(10.0);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatAcc(orig_cmd), 10.0), 0.3, ep);

    set_speed_and_reset_prev(15.0);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatAcc(orig_cmd), 15.0), 0.3, ep);
  }

  // lateral jerk lim
  // p.reference_speed_points = std::vector<double>{2.0, 4.0, 10.0};
  // p.lat_jerk_lim = std::vector<double>{0.9, 0.7, 0.1};
  const auto _calcLatJerk = [&](const auto & cmd, const double ego_v) {
    return calcLatJerk(cmd, Control{}, WHEELBASE, DT, ego_v);
  };
  {
    // since the lateral acceleration and jerk is 0 when the velocity is 0, the target value is 0
    // only in this case
    set_speed_and_reset_prev(0.0);
    EXPECT_NEAR(_calcLatJerk(_limitLateralWithLatJerk(orig_cmd), 0.0), 0.0, ep);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatJerk(orig_cmd), 0.0), DT * 0.0, ep);

    set_speed_and_reset_prev(2.0);
    EXPECT_NEAR(_calcLatJerk(_limitLateralWithLatJerk(orig_cmd), 2.0), 0.9, ep);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatJerk(orig_cmd), 2.0), DT * 0.9, ep);

    set_speed_and_reset_prev(3.0);
    EXPECT_NEAR(_calcLatJerk(_limitLateralWithLatJerk(orig_cmd), 3.0), 0.8, ep);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatJerk(orig_cmd), 3.0), DT * 0.8, ep);

    set_speed_and_reset_prev(5.0);
    const auto expect_v5 = 0.7 - 0.6 * (1.0 / 6.0);
    EXPECT_NEAR(_calcLatJerk(_limitLateralWithLatJerk(orig_cmd), 5.0), expect_v5, ep);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatJerk(orig_cmd), 5.0), DT * expect_v5, ep);

    set_speed_and_reset_prev(8.0);
    const auto expect_v8 = 0.7 - 0.6 * (4.0 / 6.0);
    EXPECT_NEAR(_calcLatJerk(_limitLateralWithLatJerk(orig_cmd), 8.0), expect_v8, ep);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatJerk(orig_cmd), 8.0), DT * expect_v8, ep);

    set_speed_and_reset_prev(10.0);
    EXPECT_NEAR(_calcLatJerk(_limitLateralWithLatJerk(orig_cmd), 10.0), 0.1, ep);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatJerk(orig_cmd), 10.0), DT * 0.1, ep);

    set_speed_and_reset_prev(15.0);
    EXPECT_NEAR(_calcLatJerk(_limitLateralWithLatJerk(orig_cmd), 15.0), 0.1, ep);
    EXPECT_NEAR(_calcLatAcc(_limitLateralWithLatJerk(orig_cmd), 15.0), DT * 0.1, ep);
  }

  // steering diff lim
  // p.reference_speed_points = std::vector<double>{2.0, 4.0, 10.0};
  // p.actual_steer_diff_lim = std::vector<double>{0.1, 0.3, 0.2};
  {
    set_speed_and_reset_prev(0.0);
    EXPECT_NEAR((_limitActualSteerDiff(orig_cmd).lateral.steering_tire_angle), 0.1, ep);

    set_speed_and_reset_prev(2.0);
    EXPECT_NEAR((_limitActualSteerDiff(orig_cmd).lateral.steering_tire_angle), 0.1, ep);

    set_speed_and_reset_prev(3.0);
    EXPECT_NEAR((_limitActualSteerDiff(orig_cmd).lateral.steering_tire_angle), 0.2, ep);

    set_speed_and_reset_prev(5.0);
    const auto expect_v5 = 0.3 - 0.1 / 6.0;
    EXPECT_NEAR((_limitActualSteerDiff(orig_cmd).lateral.steering_tire_angle), expect_v5, ep);

    set_speed_and_reset_prev(8.0);
    const auto expect_v8 = 0.3 - 0.4 / 6.0;
    EXPECT_NEAR((_limitActualSteerDiff(orig_cmd).lateral.steering_tire_angle), expect_v8, ep);

    set_speed_and_reset_prev(10.0);
    EXPECT_NEAR((_limitActualSteerDiff(orig_cmd).lateral.steering_tire_angle), 0.2, ep);

    set_speed_and_reset_prev(15.0);
    EXPECT_NEAR((_limitActualSteerDiff(orig_cmd).lateral.steering_tire_angle), 0.2, ep);
  }
}
