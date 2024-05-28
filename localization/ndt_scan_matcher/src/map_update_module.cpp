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
    std::stringstream message;
    message << "Error at MapUpdateModule::MapUpdateModule."
            << "`ndt_ptr_` is a null NDT pointer.";
    throw std::runtime_error(message.str());
  }

  // Initially, a direct map update on ndt_ptr_ is needed.
  // ndt_ptr_'s mutex is locked until it is fully rebuilt.
  // From the second update, the update is done on secondary_ndt_ptr_,
  // and ndt_ptr_ is only locked when swapping its pointer with
  // secondary_ndt_ptr_.
  need_rebuild_ = true;
}

void MapUpdateModule::callback_timer(
  const bool is_activated, const std::optional<geometry_msgs::msg::Point> & position,
  std::unique_ptr<DiagnosticsModule> & diagnostics_ptr)
{
  diagnostics_ptr->addKeyValue("timer_callback_time_stamp", clock_->now().nanoseconds());

  // check is_activated
  diagnostics_ptr->addKeyValue("is_activated", is_activated);
  if (!is_activated) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_ptr->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // check is_set_last_update_position
  const bool is_set_last_update_position = (position != std::nullopt);
  diagnostics_ptr->addKeyValue("is_set_last_update_position", is_set_last_update_position);
  if (!is_set_last_update_position) {
    std::stringstream message;
    message << "Cannot find the reference position for map update."
            << "Please check if the EKF odometry is provided to NDT.";
    diagnostics_ptr->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  if (should_update_map(position.value(), diagnostics_ptr)) {
    update_map(position.value(), diagnostics_ptr);
  }
}

bool MapUpdateModule::should_update_map(
  const geometry_msgs::msg::Point & position, std::unique_ptr<DiagnosticsModule> & diagnostics_ptr)
{
  if (last_update_position_ == std::nullopt) {
    need_rebuild_ = true;
    return true;
  }

  const double dx = position.x - last_update_position_.value().x;
  const double dy = position.y - last_update_position_.value().y;
  const double distance = std::hypot(dx, dy);

  // check distance_last_update_position_to_current_position
  diagnostics_ptr->addKeyValue("distance_last_update_position_to_current_position", distance);
  if (distance + param_.lidar_radius > param_.map_radius) {
    std::stringstream message;
    message << "Dynamic map loading is not keeping up.";
    diagnostics_ptr->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());

    // If the map does not keep up with the current position,
    // lock ndt_ptr_ entirely until it is fully rebuilt.
    need_rebuild_ = true;
  }

  return distance > param_.update_distance;
}

void MapUpdateModule::update_map(
  const geometry_msgs::msg::Point & position, std::unique_ptr<DiagnosticsModule> & diagnostics_ptr)
{
  diagnostics_ptr->addKeyValue("is_need_rebuild", need_rebuild_);

  // If the current position is super far from the previous loading position,
  // lock and rebuild ndt_ptr_
  if (need_rebuild_) {
    ndt_ptr_mutex_->lock();

    auto param = ndt_ptr_->getParams();
    auto input_source = ndt_ptr_->getInputSource();

    ndt_ptr_.reset(new NdtType);

    ndt_ptr_->setParams(param);
    if (input_source != nullptr) {
      ndt_ptr_->setInputSource(input_source);
    }

    const bool updated = update_ndt(position, *ndt_ptr_, diagnostics_ptr);

    // check is_updated_map
    diagnostics_ptr->addKeyValue("is_updated_map", updated);
    if (!updated) {
      std::stringstream message;
      message
        << "update_ndt failed. If this happens with initial position estimation, make sure that"
        << "(1) the initial position matches the pcd map and (2) the map_loader is working "
           "properly.";
      diagnostics_ptr->updateLevelAndMessage(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
      RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, message.str());
      last_update_position_ = position;
      ndt_ptr_mutex_->unlock();
      return;
    }

    ndt_ptr_mutex_->unlock();
    need_rebuild_ = false;

  } else {
    // Load map to the secondary_ndt_ptr, which does not require a mutex lock
    // Since the update of the secondary ndt ptr and the NDT align (done on
    // the main ndt_ptr_) overlap, the latency of updating/alignment reduces partly.
    // If the updating is done the main ndt_ptr_, either the update or the NDT
    // align will be blocked by the other.
    const bool updated = update_ndt(position, *secondary_ndt_ptr_, diagnostics_ptr);

    // check is_updated_map
    diagnostics_ptr->addKeyValue("is_updated_map", updated);
    if (!updated) {
      last_update_position_ = position;
      return;
    }

    ndt_ptr_mutex_->lock();
    auto dummy_ptr = ndt_ptr_;
    auto input_source = ndt_ptr_->getInputSource();
    ndt_ptr_ = secondary_ndt_ptr_;
    if (input_source != nullptr) {
      ndt_ptr_->setInputSource(input_source);
    }
    ndt_ptr_mutex_->unlock();

    dummy_ptr.reset();
  }

  secondary_ndt_ptr_.reset(new NdtType);
  *secondary_ndt_ptr_ = *ndt_ptr_;

  // Memorize the position of the last update
  last_update_position_ = position;

  // Publish the new ndt maps
  publish_partial_pcd_map();
}

bool MapUpdateModule::update_ndt(
  const geometry_msgs::msg::Point & position, NdtType & ndt,
  std::unique_ptr<DiagnosticsModule> & diagnostics_ptr)
{
  diagnostics_ptr->addKeyValue("maps_size_before", ndt.getCurrentMapIDs().size());

  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();

  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(param_.map_radius);
  request->cached_ids = ndt.getCurrentMapIDs();

  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    diagnostics_ptr->addKeyValue("is_succeed_call_pcd_loader", false);

    std::stringstream message;
    message << "Waiting for pcd loader service. Check the pointcloud_map_loader.";
    diagnostics_ptr->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  // send a request to map_loader
  auto result{pcd_loader_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    // check is_succeed_call_pcd_loader
    if (!rclcpp::ok()) {
      diagnostics_ptr->addKeyValue("is_succeed_call_pcd_loader", false);

      std::stringstream message;
      message << "pcd_loader service is not working.";
      diagnostics_ptr->updateLevelAndMessage(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;  // No update
    }
    status = result.wait_for(std::chrono::seconds(1));
  }
  diagnostics_ptr->addKeyValue("is_succeed_call_pcd_loader", true);

  auto & maps_to_add = result.get()->new_pointcloud_with_ids;
  auto & map_ids_to_remove = result.get()->ids_to_remove;

  diagnostics_ptr->addKeyValue("maps_to_add_size", maps_to_add.size());
  diagnostics_ptr->addKeyValue("maps_to_remove_size", map_ids_to_remove.size());

  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    return false;  // No update
  }

  const auto exe_start_time = std::chrono::system_clock::now();
  // Perform heavy processing outside of the lock scope

  // Add pcd
  for (auto & map : maps_to_add) {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointTarget>>();

    pcl::fromROSMsg(map.pointcloud, *cloud);
    ndt.addTarget(cloud, map.cell_id);
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
  diagnostics_ptr->addKeyValue("map_update_execution_time", exe_time);
  diagnostics_ptr->addKeyValue("maps_size_after", ndt.getCurrentMapIDs().size());
  diagnostics_ptr->addKeyValue("is_succeed_call_pcd_loader", true);
  return true;  // Updated
}

void MapUpdateModule::publish_partial_pcd_map()
{
  pcl::PointCloud<PointTarget> map_pcl = ndt_ptr_->getVoxelPCD();
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  loaded_pcd_pub_->publish(map_msg);
}
