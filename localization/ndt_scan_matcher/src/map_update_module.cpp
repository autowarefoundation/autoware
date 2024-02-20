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

#include "ndt_scan_matcher/map_update_module.hpp"

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, std::mutex * ndt_ptr_mutex, NdtPtrType & ndt_ptr,
  HyperParameters::DynamicMapLoading param)
: ndt_ptr_(ndt_ptr),
  ndt_ptr_mutex_(ndt_ptr_mutex),
  logger_(node->get_logger()),
  clock_(node->get_clock()),
  param_(param)
{
  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");

  secondary_ndt_ptr_.reset(new NdtType);

  if (ndt_ptr_) {
    *secondary_ndt_ptr_ = *ndt_ptr_;
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, "Attempt to update a null NDT pointer.");
  }

  // Initially, a direct map update on ndt_ptr_ is needed.
  // ndt_ptr_'s mutex is locked until it is fully rebuilt.
  // From the second update, the update is done on secondary_ndt_ptr_,
  // and ndt_ptr_ is only locked when swapping its pointer with
  // secondary_ndt_ptr_.
  need_rebuild_ = true;
}

bool MapUpdateModule::should_update_map(const geometry_msgs::msg::Point & position)
{
  if (last_update_position_ == std::nullopt) {
    return false;
  }

  const double dx = position.x - last_update_position_.value().x;
  const double dy = position.y - last_update_position_.value().y;
  const double distance = std::hypot(dx, dy);
  if (distance + param_.lidar_radius > param_.map_radius) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, "Dynamic map loading is not keeping up.");
    // If the map does not keep up with the current position,
    // lock ndt_ptr_ entirely until it is fully rebuilt.
    need_rebuild_ = true;
  }

  return distance > param_.update_distance;
}

void MapUpdateModule::update_map(const geometry_msgs::msg::Point & position)
{
  // If the current position is super far from the previous loading position,
  // lock and rebuild ndt_ptr_
  if (need_rebuild_) {
    ndt_ptr_mutex_->lock();
    auto param = ndt_ptr_->getParams();

    ndt_ptr_.reset(new NdtType);

    ndt_ptr_->setParams(param);

    update_ndt(position, *ndt_ptr_);
    ndt_ptr_mutex_->unlock();
    need_rebuild_ = false;
  } else {
    // Load map to the secondary_ndt_ptr, which does not require a mutex lock
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    update_ndt(position, *secondary_ndt_ptr_);

    ndt_ptr_mutex_->lock();
    auto input_source = ndt_ptr_->getInputSource();
    ndt_ptr_ = secondary_ndt_ptr_;
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_mutex_->unlock();
  }

  secondary_ndt_ptr_.reset(new NdtType);
  *secondary_ndt_ptr_ = *ndt_ptr_;

  // Memorize the position of the last update
  last_update_position_ = position;

  // Publish the new ndt maps
  publish_partial_pcd_map();
}

void MapUpdateModule::update_ndt(const geometry_msgs::msg::Point & position, NdtType & ndt)
{
  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();

  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(param_.map_radius);
  request->cached_ids = ndt.getCurrentMapIDs();

  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(logger_, "Waiting for pcd loader service. Check the pointcloud_map_loader.");
  }

  // send a request to map_loader
  auto result{pcd_loader_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    RCLCPP_INFO(logger_, "waiting response");
    if (!rclcpp::ok()) {
      return;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  auto & maps_to_add = result.get()->new_pointcloud_with_ids;
  auto & map_ids_to_remove = result.get()->ids_to_remove;

  RCLCPP_INFO(
    logger_, "Update map (Add: %lu, Remove: %lu)", maps_to_add.size(), map_ids_to_remove.size());
  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    RCLCPP_INFO(logger_, "Skip map update");
    return;
  }

  const auto exe_start_time = std::chrono::system_clock::now();
  const size_t add_size = maps_to_add.size();
  // Perform heavy processing outside of the lock scope
  std::vector<pcl::shared_ptr<pcl::PointCloud<PointTarget>>> points_pcl(add_size);

  for (size_t i = 0; i < add_size; i++) {
    points_pcl[i] = pcl::make_shared<pcl::PointCloud<PointTarget>>();
    pcl::fromROSMsg(maps_to_add[i].pointcloud, *points_pcl[i]);
  }

  // Add pcd
  for (size_t i = 0; i < add_size; i++) {
    ndt.addTarget(points_pcl[i], maps_to_add[i].cell_id);
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    ndt.removeTarget(map_id_to_remove);
  }

  ndt.createVoxelKdtree();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<double>(duration_micro_sec) / 1000.0;
  RCLCPP_INFO(logger_, "Time duration for creating new ndt_ptr: %lf [ms]", exe_time);
}

void MapUpdateModule::publish_partial_pcd_map()
{
  pcl::PointCloud<PointTarget> map_pcl = ndt_ptr_->getVoxelPCD();
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  loaded_pcd_pub_->publish(map_msg);
}
