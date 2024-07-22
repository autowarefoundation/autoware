// Copyright 2022 TIER IV, Inc.
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

#ifndef RADAR_THRESHOLD_FILTER_NODE_HPP_
#define RADAR_THRESHOLD_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <radar_msgs/msg/radar_scan.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::radar_threshold_filter
{
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

class RadarThresholdFilterNode : public rclcpp::Node
{
public:
  explicit RadarThresholdFilterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    bool is_amplitude_filter{};
    double amplitude_min{};
    double amplitude_max{};
    bool is_range_filter{};
    double range_min{};
    double range_max{};
    bool is_azimuth_filter{};
    double azimuth_min{};
    double azimuth_max{};
    bool is_z_filter{};
    double z_min{};
    double z_max{};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarScan>::SharedPtr sub_radar_{};

  // Callback
  void onData(const RadarScan::ConstSharedPtr radar_msg);

  // Publisher
  rclcpp::Publisher<RadarScan>::SharedPtr pub_radar_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

public:
  // Function
  bool isWithinThreshold(const RadarReturn & radar_return);
};

}  // namespace autoware::radar_threshold_filter

#endif  // RADAR_THRESHOLD_FILTER_NODE_HPP_
