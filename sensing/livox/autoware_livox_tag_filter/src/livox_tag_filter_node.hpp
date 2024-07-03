// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef LIVOX_TAG_FILTER_NODE_HPP_
#define LIVOX_TAG_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <vector>

namespace autoware::livox_tag_filter
{
class LivoxTagFilterNode : public rclcpp::Node
{
public:
  explicit LivoxTagFilterNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  std::vector<std::int64_t> ignore_tags_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
};
}  // namespace autoware::livox_tag_filter

#endif  // LIVOX_TAG_FILTER_NODE_HPP_
