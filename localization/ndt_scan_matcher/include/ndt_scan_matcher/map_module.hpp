// Copyright 2022 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__MAP_MODULE_HPP_
#define NDT_SCAN_MATCHER__MAP_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>

#include <memory>

class MapModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::NormalDistributionsTransform<PointSource, PointTarget>;

public:
  MapModule(
    rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
    std::shared_ptr<NormalDistributionsTransform> ndt_ptr,
    rclcpp::CallbackGroup::SharedPtr map_callback_group);

private:
  void callback_map_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
  std::shared_ptr<NormalDistributionsTransform> ndt_ptr_;
  std::mutex * ndt_ptr_mutex_;
};

#endif  // NDT_SCAN_MATCHER__MAP_MODULE_HPP_
