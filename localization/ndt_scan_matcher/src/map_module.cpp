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

#include "ndt_scan_matcher/map_module.hpp"

MapModule::MapModule(
  rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
  std::shared_ptr<NormalDistributionsTransform> ndt_ptr,
  rclcpp::CallbackGroup::SharedPtr map_callback_group)
: ndt_ptr_(ndt_ptr), ndt_ptr_mutex_(ndt_ptr_mutex)
{
  auto map_sub_opt = rclcpp::SubscriptionOptions();
  map_sub_opt.callback_group = map_callback_group;

  map_points_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapModule::callback_map_points, this, std::placeholders::_1), map_sub_opt);
}

void MapModule::callback_map_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  NormalDistributionsTransform new_ndt;
  new_ndt.setParams(ndt_ptr_->getParams());

  pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt.setInputTarget(map_points_ptr);
  // create Thread
  // detach
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  new_ndt.align(*output_cloud);

  // swap
  ndt_ptr_mutex_->lock();
  *ndt_ptr_ = new_ndt;
  ndt_ptr_mutex_->unlock();
}
