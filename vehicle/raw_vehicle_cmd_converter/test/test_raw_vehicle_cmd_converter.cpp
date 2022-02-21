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
#include "raw_vehicle_cmd_converter/steer_converter.hpp"

#include <cmath>

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
using raw_vehicle_cmd_converter::SteerConverter;

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

bool loadSteerMapData(SteerConverter & steer_map)
{
  return steer_map.setFFMap(map_path + "test_steer_map.csv");
}

TEST(ConverterTests, LoadValidPath)
{
  AccelMap accel_map;
  BrakeMap brake_map;
  SteerConverter steer_map;

  // for valid path
  EXPECT_TRUE(loadAccelMapData(accel_map));
  EXPECT_TRUE(loadBrakeMapData(brake_map));
  EXPECT_TRUE(loadSteerMapData(steer_map));

  // for invalid path
  EXPECT_FALSE(accel_map.readAccelMapFromCSV("invalid.csv"));
  EXPECT_FALSE(brake_map.readBrakeMapFromCSV("invalid.csv"));
  EXPECT_FALSE(steer_map.setFFMap("invalid.csv"));
}

TEST(ConverterTests, AccelMapCalculation)
{
  AccelMap accel_map;
  loadAccelMapData(accel_map);
  const auto calcThrottle = [&](double acc, double vel) {
    double output;
    accel_map.getThrottle(acc, vel, output);
    return output;
  };

  // case for min vel
  EXPECT_DOUBLE_EQ(calcThrottle(1.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(calcThrottle(1.5, 0.0), 0.25);
  EXPECT_DOUBLE_EQ(calcThrottle(3.0, 0.0), 1.0);

  // case for min throttle
  EXPECT_DOUBLE_EQ(calcThrottle(6.0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(calcThrottle(21.0, 20.0), 0.0);

  // case for direct access
  EXPECT_DOUBLE_EQ(calcThrottle(22.0, 10.0), 0.5);
  EXPECT_DOUBLE_EQ(calcThrottle(46.0, 20.0), 1.0);

  // case for interpolation
  EXPECT_DOUBLE_EQ(calcThrottle(9.0, 5.0), 0.25);
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

  // case for min vel
  EXPECT_DOUBLE_EQ(calcBrake(-1.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(calcBrake(-1.5, 0.0), 0.25);
  EXPECT_DOUBLE_EQ(calcBrake(-3.0, 0.0), 1.0);

  // case for min brake
  EXPECT_DOUBLE_EQ(calcBrake(-6.0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(calcBrake(-21.0, 20.0), 0.0);

  // case for direct access
  EXPECT_DOUBLE_EQ(calcBrake(-22.0, 10.0), 0.5);
  EXPECT_DOUBLE_EQ(calcBrake(-46.0, 20.0), 1.0);

  // case for interpolation
  EXPECT_DOUBLE_EQ(calcBrake(-9.0, 5.0), 0.25);
}

TEST(ConverterTests, SteerMapCalculation)
{
  SteerConverter steer_map;
  loadSteerMapData(steer_map);
  const auto calcSteer = [&](double steer_vel, double steer) {
    return steer_map.calcFFSteer(steer_vel, steer);
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
