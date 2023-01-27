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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "raw_vehicle_cmd_converter/accel_map.hpp"
#include "raw_vehicle_cmd_converter/brake_map.hpp"
#include "raw_vehicle_cmd_converter/pid.hpp"
#include "raw_vehicle_cmd_converter/steer_map.hpp"

#include <cmath>
#include <stdexcept>

/*
 * Throttle data: (vel, throttle -> acc)
 *         0.0,  10.0,  20.0  (vel)
 * 0,      1.0,  11.0,  21.0
 * 0.5,    2.0,  22.0,  42.0
 * 1.0,    3.0,  33.0,  46.0
 * (throttle)
 *
 *
 * Brake data: (vel, brake -> acc)
 *          0.0,   10.0,   20.0 (vel)
 * 0,      -1.0,  -11.0,  -21.0
 * 0.5,    -2.0,  -22.0,  -42.0
 * 1.0,    -3.0,  -33.0,  -46.0
 *(brake)
 *
 *
 * Steer data: (steer, vol -> steer_rotation_rate)
 *       -10,   0,  10 (steer)
 * -10,  -10, -10, -10
 *   0,    0,   0,   0
 *  10,   10,  10,  10
 * (voltage)
 *
 */

using raw_vehicle_cmd_converter::AccelMap;
using raw_vehicle_cmd_converter::BrakeMap;
using raw_vehicle_cmd_converter::PIDController;
using raw_vehicle_cmd_converter::SteerMap;
double epsilon = 1e-4;
// may throw PackageNotFoundError exception for invalid package
const auto map_path =
  ament_index_cpp::get_package_share_directory("raw_vehicle_cmd_converter") + "/test/map_data/";

bool loadAccelMapData(AccelMap & accel_map)
{
  return accel_map.readAccelMapFromCSV(map_path + "test_accel_map.csv");
}

bool loadBrakeMapData(BrakeMap & brake_map)
{
  return brake_map.readBrakeMapFromCSV(map_path + "test_brake_map.csv");
}

bool loadSteerMapData(SteerMap & steer_map)
{
  return steer_map.readSteerMapFromCSV(map_path + "test_steer_map.csv");
}

PIDController createSteerPid(
  double kp, double ki, double kd, double max_ret, double min_ret, double max_ret_p,
  double min_ret_p, double max_ret_i, double min_ret_i, double max_ret_d, double min_ret_d,
  double invalid_integration_decay)
{
  PIDController steer_pid;
  steer_pid.setDecay(invalid_integration_decay);
  steer_pid.setGains(kp, ki, kd);
  steer_pid.setLimits(
    max_ret, min_ret, max_ret_p, min_ret_p, max_ret_i, min_ret_i, max_ret_d, min_ret_d);
  steer_pid.setInitialized();

  return steer_pid;
}

TEST(ConverterTests, LoadExampleMap)
{
  AccelMap accel_map;
  BrakeMap brake_map;
  SteerMap steer_map;
  const auto data_path =
    ament_index_cpp::get_package_share_directory("raw_vehicle_cmd_converter") + "/data/default/";
  // for invalid path
  EXPECT_TRUE(accel_map.readAccelMapFromCSV(data_path + "accel_map.csv", true));
  EXPECT_TRUE(brake_map.readBrakeMapFromCSV(data_path + "brake_map.csv", true));
  EXPECT_TRUE(steer_map.readSteerMapFromCSV(data_path + "steer_map.csv", true));
}

TEST(ConverterTests, LoadValidPath)
{
  AccelMap accel_map;
  BrakeMap brake_map;
  SteerMap steer_map;

  // for valid path
  EXPECT_TRUE(loadAccelMapData(accel_map));
  EXPECT_TRUE(loadBrakeMapData(brake_map));
  EXPECT_TRUE(loadSteerMapData(steer_map));

  // for invalid path
  EXPECT_FALSE(accel_map.readAccelMapFromCSV("invalid.csv", true));
  EXPECT_FALSE(brake_map.readBrakeMapFromCSV("invalid.csv", true));
  EXPECT_FALSE(steer_map.readSteerMapFromCSV("invalid.csv", true));

  // for invalid maps
  EXPECT_FALSE(accel_map.readAccelMapFromCSV(map_path + "test_1col_map.csv", true));
  EXPECT_FALSE(accel_map.readAccelMapFromCSV(map_path + "test_inconsistent_rows_map.csv", true));
  EXPECT_FALSE(accel_map.readAccelMapFromCSV(map_path + "test_not_interpolatable.csv", true));
}

TEST(ConverterTests, AccelMapCalculation)
{
  AccelMap accel_map;
  loadAccelMapData(accel_map);
  const auto calcThrottle = [&](double acc, double vel) {
    double output = 0.0;
    accel_map.getThrottle(acc, vel, output);
    return output;
  };

  // case for max vel nominal acc
  EXPECT_DOUBLE_EQ(calcThrottle(0.0, 20.0), 0.5);

  // case for max vel max acc
  EXPECT_DOUBLE_EQ(calcThrottle(2.0, 5.0), 1.0);

  // case for direct access
  EXPECT_DOUBLE_EQ(calcThrottle(0.5, 5.0), 0.5);

  // case for interpolation
  EXPECT_DOUBLE_EQ(calcThrottle(2.0, 0.0), 0.75);

  const auto calcAcceleration = [&](double throttle, double vel) {
    double output;
    accel_map.getAcceleration(throttle, vel, output);
    return output;
  };

  // case for min vel max throttle
  EXPECT_DOUBLE_EQ(calcAcceleration(1.0, 0.0), 3.0);

  // case for max vel max throttle
  EXPECT_DOUBLE_EQ(calcAcceleration(2.0, 10.0), 1.5);

  // case for direct access
  EXPECT_DOUBLE_EQ(calcAcceleration(0.0, 10.0), -0.5);

  // case for interpolation
  EXPECT_DOUBLE_EQ(calcAcceleration(0.75, 5.0), 1.25);
}

TEST(ConverterTests, BrakeMapCalculation)
{
  BrakeMap brake_map;
  loadBrakeMapData(brake_map);
  const auto calcBrake = [&](double acc, double vel) {
    double output;
    brake_map.getBrake(acc, vel, output);
    return output;
  };

  // case for min vel min acc
  EXPECT_DOUBLE_EQ(calcBrake(-2.5, 0.0), 1.0);

  // case for max vel low acc
  EXPECT_DOUBLE_EQ(calcBrake(-2.0, 11.0), 0.5);

  // case for direct access
  EXPECT_DOUBLE_EQ(calcBrake(-2.0, 5.0), 0.5);

  // case for interpolation
  EXPECT_DOUBLE_EQ(calcBrake(-2.25, 5.0), 0.75);

  const auto calcAcceleration = [&](double brake, double vel) {
    double output;
    brake_map.getAcceleration(brake, vel, output);
    return output;
  };

  // case for min vel max brake
  EXPECT_DOUBLE_EQ(calcAcceleration(1.1, 0.0), -2.0);

  // case for max vel max brake
  EXPECT_DOUBLE_EQ(calcAcceleration(1.1, 12.0), -3.0);

  // case for direct access
  EXPECT_DOUBLE_EQ(calcAcceleration(0.5, 10.0), -2.0);

  // case for interpolation
  EXPECT_DOUBLE_EQ(calcAcceleration(0.75, 5.0), -2.25);
}

TEST(ConverterTests, SteerMapCalculation)
{
  SteerMap steer_map;
  loadSteerMapData(steer_map);
  const auto calcSteer = [&](double steer_vel, double steer) {
    double output;
    steer_map.getSteer(steer_vel, steer, output);
    return output;
  };

  // case for min steer
  EXPECT_DOUBLE_EQ(calcSteer(-10.0, -10.0), -10.0);
  EXPECT_DOUBLE_EQ(calcSteer(-5.0, -10.0), -5.0);
  EXPECT_DOUBLE_EQ(calcSteer(10.0, -10.0), 10.0);

  // case for min voltage
  EXPECT_DOUBLE_EQ(calcSteer(-10.0, 5.0), -10.0);

  // case for direct access
  EXPECT_DOUBLE_EQ(calcSteer(0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(calcSteer(10.0, 0.0), 10.0);

  // case for interpolation
  EXPECT_DOUBLE_EQ(calcSteer(5.0, 5.0), 5.0);
}

TEST(PIDTests, calculateFB)
{
  PIDController steer_pid;

  double dt = 0.1;
  double vel = 5.0;
  double target_value = 0.15;
  double current_value;
  std::vector<double> pid_contributions(3, 0.0);
  std::vector<double> pid_errors(3, 0.0);
  double fb_value;

  std::vector<double> fb_values{8.0, 8.0, 8.0, 8.0, 8.0, 7.85, 6.4, 4.9, 3.4, 1.9};
  std::vector<double> ret_p{8.0, 8.0, 8.0, 8.0, 8.0, 7.5, 6.0, 4.5, 3.0, 1.5};
  std::vector<double> ret_i{0.225, 0.42, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  std::vector<double> ret_d{0.0, -0.15, -0.15, -0.15, -0.15, -0.15, -0.1, -0.1, -0.1, -0.1};

  steer_pid = createSteerPid(150.0, 15.0, 1.0, 8.0, -8.0, 8.0, -8.0, 0.5, -0.5, 0.15, -0.15, 0.97);
  current_value = 0;
  for (int i = 0; i < 10; i++) {
    fb_value =
      steer_pid.calculateFB(target_value, dt, vel, current_value, pid_contributions, pid_errors);
    EXPECT_NEAR(fb_value, fb_values.at(i), epsilon);
    EXPECT_NEAR(pid_contributions.at(0), ret_p.at(i), epsilon);
    EXPECT_NEAR(pid_contributions.at(1), ret_i.at(i), epsilon);
    EXPECT_NEAR(pid_contributions.at(2), ret_d.at(i), epsilon);

    if (i < 5) {
      current_value += 0.02;
    } else {
      current_value += 0.01;
    }
  }

  steer_pid.reset();
  current_value = 0;
  for (int i = 0; i < 10; i++) {
    fb_value =
      steer_pid.calculateFB(-target_value, dt, vel, current_value, pid_contributions, pid_errors);
    EXPECT_NEAR(fb_value, -fb_values.at(i), epsilon);
    EXPECT_NEAR(pid_contributions.at(0), -ret_p.at(i), epsilon);
    EXPECT_NEAR(pid_contributions.at(1), -ret_i.at(i), epsilon);
    EXPECT_NEAR(pid_contributions.at(2), -ret_d.at(i), epsilon);

    if (i < 5) {
      current_value -= 0.02;
    } else {
      current_value -= 0.01;
    }
  }

  // invalid_integration_decay
  steer_pid.reset();
  current_value = 0;
  vel = 5.0;
  steer_pid.calculateFB(target_value, dt, vel, current_value, pid_contributions, pid_errors);
  current_value = 0.02;
  vel = 0.001;
  fb_value =
    steer_pid.calculateFB(target_value, dt, vel, current_value, pid_contributions, pid_errors);
  EXPECT_NEAR(fb_value, 8.0, epsilon);
  EXPECT_NEAR(pid_contributions.at(0), 8.0, epsilon);
  EXPECT_NEAR(pid_contributions.at(1), 0.21825, epsilon);
  EXPECT_NEAR(pid_contributions.at(2), -0.15, epsilon);
}
