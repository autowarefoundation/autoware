// Copyright 2023 Tier IV, Inc.
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

#include "../../src/radar_threshold_filter_node.hpp"

#include <radar_msgs/msg/radar_scan.hpp>

#include <gtest/gtest.h>

TEST(RadarThresholdFilter, isWithinThreshold)
{
  rclcpp::init(0, nullptr);
  using autoware::radar_threshold_filter::RadarThresholdFilterNode;
  using radar_msgs::msg::RadarReturn;

  const double amplitude_min = -10.0;
  const double amplitude_max = 100.0;
  const double range_min = 20.0;
  const double range_max = 300.0;
  const double azimuth_min = -1.2;
  const double azimuth_max = 1.2;
  const double z_min = -2.0;
  const double z_max = 5.0;
  // amplitude filter
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({
      {"node_params.is_amplitude_filter", true},
      {"node_params.amplitude_min", amplitude_min},
      {"node_params.amplitude_max", amplitude_max},
      {"node_params.is_range_filter", false},
      {"node_params.range_min", range_min},
      {"node_params.range_max", range_max},
      {"node_params.is_azimuth_filter", false},
      {"node_params.azimuth_min", azimuth_min},
      {"node_params.azimuth_max", azimuth_max},
      {"node_params.is_z_filter", false},
      {"node_params.z_min", z_min},
      {"node_params.z_max", z_max},
    });

    RadarThresholdFilterNode node(node_options);
    RadarReturn radar_return;

    // Now you can use radar_return and set its fields as needed
    radar_return.amplitude = -100.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
    radar_return.amplitude = 0.0;
    EXPECT_TRUE(node.isWithinThreshold(radar_return));
    radar_return.amplitude = 1000.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
  }

  // range filter
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({
      {"node_params.is_amplitude_filter", false},
      {"node_params.amplitude_min", amplitude_min},
      {"node_params.amplitude_max", amplitude_max},
      {"node_params.is_range_filter", true},
      {"node_params.range_min", range_min},
      {"node_params.range_max", range_max},
      {"node_params.is_azimuth_filter", false},
      {"node_params.azimuth_min", azimuth_min},
      {"node_params.azimuth_max", azimuth_max},
      {"node_params.is_z_filter", false},
      {"node_params.z_min", z_min},
      {"node_params.z_max", z_max},
    });

    RadarThresholdFilterNode node(node_options);
    RadarReturn radar_return;

    radar_return.range = 0.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
    radar_return.range = 100.0;
    EXPECT_TRUE(node.isWithinThreshold(radar_return));
    radar_return.range = 1000.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
  }

  // azimuth filter
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({
      {"node_params.is_amplitude_filter", false},
      {"node_params.amplitude_min", amplitude_min},
      {"node_params.amplitude_max", amplitude_max},
      {"node_params.is_range_filter", false},
      {"node_params.range_min", range_min},
      {"node_params.range_max", range_max},
      {"node_params.is_azimuth_filter", true},
      {"node_params.azimuth_min", azimuth_min},
      {"node_params.azimuth_max", azimuth_max},
      {"node_params.is_z_filter", false},
      {"node_params.z_min", z_min},
      {"node_params.z_max", z_max},
    });

    RadarThresholdFilterNode node(node_options);
    RadarReturn radar_return;
    radar_return.azimuth = -10.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
    radar_return.azimuth = 0.0;
    EXPECT_TRUE(node.isWithinThreshold(radar_return));
    radar_return.azimuth = 10.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
  }

  // z
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({
      {"node_params.is_amplitude_filter", false},
      {"node_params.amplitude_min", amplitude_min},
      {"node_params.amplitude_max", amplitude_max},
      {"node_params.is_range_filter", false},
      {"node_params.range_min", range_min},
      {"node_params.range_max", range_max},
      {"node_params.is_azimuth_filter", false},
      {"node_params.azimuth_min", azimuth_min},
      {"node_params.azimuth_max", azimuth_max},
      {"node_params.is_z_filter", true},
      {"node_params.z_min", z_min},
      {"node_params.z_max", z_max},
    });

    RadarThresholdFilterNode node(node_options);
    RadarReturn radar_return;
    radar_return.elevation = M_PI / 2;
    // range =  z / std::sin(radar_return.elevation)
    radar_return.range = -10.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
    radar_return.range = 0.0;
    EXPECT_TRUE(node.isWithinThreshold(radar_return));
    radar_return.range = 10.0;
    EXPECT_FALSE(node.isWithinThreshold(radar_return));
  }
}
