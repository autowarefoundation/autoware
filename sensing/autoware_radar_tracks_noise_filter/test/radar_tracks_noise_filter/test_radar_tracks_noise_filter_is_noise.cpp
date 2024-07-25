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

#include "../../src/radar_tracks_noise_filter_node.hpp"

#include <radar_msgs/msg/radar_scan.hpp>

#include <gtest/gtest.h>

std::shared_ptr<autoware::radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode> get_node(
  float velocity_y_threshold)
{
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({
    {"node_params.is_amplitude_filter", true},
    {"velocity_y_threshold", velocity_y_threshold},
  });

  auto node =
    std::make_shared<autoware::radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode>(
      node_options);
  return node;
}

radar_msgs::msg::RadarTrack getRadarTrack(float velocity_x, float velocity_y)
{
  radar_msgs::msg::RadarTrack radar_track;
  geometry_msgs::msg::Vector3 vector_msg;
  vector_msg.x = velocity_x;
  vector_msg.y = velocity_y;
  vector_msg.z = 0.0;
  radar_track.velocity = vector_msg;
  return radar_track;
}

TEST(RadarTracksNoiseFilter, isNoise)
{
  using autoware::radar_tracks_noise_filter::RadarTrackCrossingNoiseFilterNode;
  using radar_msgs::msg::RadarTrack;
  rclcpp::init(0, nullptr);
  {
    float velocity_node_threshold = 0.0;
    float y_velocity = 0.0;
    float x_velocity = 10.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velocity_node_threshold);
    RadarTrack radar_track = getRadarTrack(x_velocity, y_velocity);
    EXPECT_TRUE(node->isNoise(radar_track));
  }

  {
    float velocity_node_threshold = -1.0;
    float y_velocity = 3.0;
    float x_velocity = 10.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velocity_node_threshold);
    RadarTrack radar_track = getRadarTrack(x_velocity, y_velocity);
    EXPECT_TRUE(node->isNoise(radar_track));
  }

  {
    float velocity_node_threshold = -1.0;
    float y_velocity = 3.0;
    float x_velocity = 100.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velocity_node_threshold);
    RadarTrack radar_track = getRadarTrack(x_velocity, y_velocity);
    EXPECT_TRUE(node->isNoise(radar_track));
  }

  {
    float velocity_node_threshold = 3.0;
    float y_velocity = 1.0;
    float x_velocity = 10.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velocity_node_threshold);
    RadarTrack radar_track = getRadarTrack(x_velocity, y_velocity);
    EXPECT_FALSE(node->isNoise(radar_track));
  }

  {
    float velocity_node_threshold = 3.0;
    float y_velocity = 1.0;
    float x_velocity = -10.0;
    std::shared_ptr<RadarTrackCrossingNoiseFilterNode> node = get_node(velocity_node_threshold);
    RadarTrack radar_track = getRadarTrack(x_velocity, y_velocity);
    EXPECT_FALSE(node->isNoise(radar_track));
  }
}
