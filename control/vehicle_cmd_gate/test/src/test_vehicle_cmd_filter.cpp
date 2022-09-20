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

using autoware_auto_control_msgs::msg::AckermannControlCommand;

constexpr double NOMINAL_INTERVAL = 1.0;

void setFilterParams(
  VehicleCmdFilter & f, double v, double a, double j, double lat_a, double lat_j, double wheelbase)
{
  f.setVelLim(v);
  f.setLonAccLim(a);
  f.setLonJerkLim(j);
  f.setLatAccLim(lat_a);
  f.setLatJerkLim(lat_j);
  f.setWheelBase(wheelbase);
}

AckermannControlCommand genCmd(double s, double sr, double v, double a)
{
  AckermannControlCommand cmd;
  cmd.lateral.steering_tire_angle = s;
  cmd.lateral.steering_tire_rotation_rate = sr;
  cmd.longitudinal.speed = v;
  cmd.longitudinal.acceleration = a;
  return cmd;
}

double calcLatAcc(const AckermannControlCommand & cmd, const double wheelbase)
{
  double v = cmd.longitudinal.speed;
  return v * v * std::tan(cmd.lateral.steering_tire_angle) / wheelbase;
}

void test_all(
  double V_LIM, double A_LIM, double J_LIM, double LAT_A_LIM, double LAT_J_LIM,
  const AckermannControlCommand & prev_cmd, const AckermannControlCommand & raw_cmd)
{
  const double WHEELBASE = 3.0;
  const double DT = 0.1;  // [s]

  VehicleCmdFilter filter;
  setFilterParams(filter, V_LIM, A_LIM, J_LIM, LAT_A_LIM, LAT_J_LIM, WHEELBASE);
  filter.setPrevCmd(prev_cmd);

  // velocity filter
  {
    auto filtered_cmd = raw_cmd;
    filter.limitLongitudinalWithVel(filtered_cmd);

    // check if the filtered value does not exceed the limit.
    ASSERT_LT_NEAR(filtered_cmd.longitudinal.speed, V_LIM);

    // check if the undesired filter is not applied.
    if (std::abs(raw_cmd.longitudinal.speed) < V_LIM) {
      ASSERT_NEAR(filtered_cmd.longitudinal.speed, raw_cmd.longitudinal.speed, THRESHOLD);
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
    const double v_max = prev_cmd.longitudinal.speed + A_LIM * DT;
    const double v_min = prev_cmd.longitudinal.speed - A_LIM * DT;
    ASSERT_LT_NEAR(filtered_cmd.longitudinal.speed, v_max);
    ASSERT_GT_NEAR(filtered_cmd.longitudinal.speed, v_min);

    // check if the undesired filter is not applied.
    if (v_min < raw_cmd.longitudinal.speed && raw_cmd.longitudinal.speed < v_max) {
      ASSERT_NEAR(filtered_cmd.longitudinal.speed, raw_cmd.longitudinal.speed, THRESHOLD);
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
    const double filtered_latacc = calcLatAcc(filtered_cmd, WHEELBASE);
    const double raw_latacc = calcLatAcc(raw_cmd, WHEELBASE);

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
    const double prev_lat_acc = calcLatAcc(prev_cmd, WHEELBASE);
    const double filtered_lat_acc = calcLatAcc(filtered_cmd, WHEELBASE);
    const double raw_lat_acc = calcLatAcc(raw_cmd, WHEELBASE);
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
}

TEST(VehicleCmdFilter, VehicleCmdFilter)
{
  const std::vector<double> v_arr = {0.0, 1.0, 100.0};
  const std::vector<double> a_arr = {0.0, 1.0, 100.0};
  const std::vector<double> j_arr = {0.0, 0.1, 1.0};
  const std::vector<double> lat_a_arr = {0.01, 1.0, 100.0};
  const std::vector<double> lat_j_arr = {0.01, 1.0, 100.0};

  const std::vector<AckermannControlCommand> prev_cmd_arr = {
    genCmd(0.0, 0.0, 0.0, 0.0), genCmd(1.0, 1.0, 1.0, 1.0)};

  const std::vector<AckermannControlCommand> raw_cmd_arr = {
    genCmd(1.0, 1.0, 1.0, 1.0), genCmd(10.0, -1.0, -1.0, -1.0)};

  for (const auto & v : v_arr) {
    for (const auto & a : a_arr) {
      for (const auto & j : j_arr) {
        for (const auto & la : lat_a_arr) {
          for (const auto & lj : lat_j_arr) {
            for (const auto & prev_cmd : prev_cmd_arr) {
              for (const auto & raw_cmd : raw_cmd_arr) {
                test_all(v, a, j, la, lj, prev_cmd, raw_cmd);
              }
            }
          }
        }
      }
    }
  }
}
