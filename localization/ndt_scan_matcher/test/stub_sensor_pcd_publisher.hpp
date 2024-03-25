// Copyright 2024 Autoware Foundation
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

#ifndef STUB_SENSOR_PCD_PUBLISHER_HPP_
#define STUB_SENSOR_PCD_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

class StubSensorPcdPublisher : public rclcpp::Node
{
public:
  StubSensorPcdPublisher() : Node("stub_sensor_pcd_publisher")
  {
    pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);
  }

  void publish_pcd(const sensor_msgs::msg::PointCloud2 & pcd) { pcd_publisher_->publish(pcd); }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
};

#endif  // STUB_SENSOR_PCD_PUBLISHER_HPP_
