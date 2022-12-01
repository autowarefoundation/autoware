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

#ifndef RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_NODE_HPP_
#define RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_NODE_HPP_

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <radar_msgs/msg/radar_scan.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_static_pointcloud_filter
{
using nav_msgs::msg::Odometry;
using radar_msgs::msg::RadarReturn;
using radar_msgs::msg::RadarScan;

class RadarStaticPointcloudFilterNode : public rclcpp::Node
{
public:
  explicit RadarStaticPointcloudFilterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double doppler_velocity_sd{};
  };

private:
  // Subscriber
  message_filters::Subscriber<RadarScan> sub_radar_{};
  message_filters::Subscriber<Odometry> sub_odometry_{};
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<RadarScan, Odometry>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;

  // Callback
  void onData(const RadarScan::ConstSharedPtr radar_msg, const Odometry::ConstSharedPtr odom_msg);

  // Publisher
  rclcpp::Publisher<RadarScan>::SharedPtr pub_static_radar_{};
  rclcpp::Publisher<RadarScan>::SharedPtr pub_dynamic_radar_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Function
  bool isStaticPointcloud(
    const RadarReturn & radar_return, const Odometry::ConstSharedPtr & odom_msg,
    geometry_msgs::msg::TransformStamped::ConstSharedPtr transform);
};
}  // namespace radar_static_pointcloud_filter

#endif  // RADAR_STATIC_POINTCLOUD_FILTER__RADAR_STATIC_POINTCLOUD_FILTER_NODE_HPP_
