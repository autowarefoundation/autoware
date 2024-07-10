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

#ifndef RADAR_TRACKS_NOISE_FILTER_NODE_HPP_
#define RADAR_TRACKS_NOISE_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "radar_msgs/msg/radar_tracks.hpp"

#include <chrono>
#include <memory>
#include <vector>

namespace autoware::radar_tracks_noise_filter
{
using radar_msgs::msg::RadarTrack;
using radar_msgs::msg::RadarTracks;

class RadarTrackCrossingNoiseFilterNode : public rclcpp::Node
{
public:
  explicit RadarTrackCrossingNoiseFilterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double velocity_y_threshold{};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarTracks>::SharedPtr sub_tracks_{};

  // Callback
  void onTracks(const RadarTracks::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<RadarTracks>::SharedPtr pub_noise_tracks_{};
  rclcpp::Publisher<RadarTracks>::SharedPtr pub_filtered_tracks_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

public:
  // Core
  bool isNoise(const RadarTrack & radar_track);
};

}  // namespace autoware::radar_tracks_noise_filter

#endif  // RADAR_TRACKS_NOISE_FILTER_NODE_HPP_
