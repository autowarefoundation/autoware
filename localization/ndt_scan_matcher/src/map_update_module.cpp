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

template <typename T, typename U>
double norm_xy(const T p1, const U p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

MapUpdateModule::MapUpdateModule(
  rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
  std::shared_ptr<NormalDistributionsTransform> ndt_ptr,
  std::shared_ptr<Tf2ListenerModule> tf2_listener_module, std::string map_frame,
  rclcpp::CallbackGroup::SharedPtr main_callback_group,
  std::shared_ptr<std::map<std::string, std::string>> state_ptr)
: ndt_ptr_(std::move(ndt_ptr)),
  ndt_ptr_mutex_(ndt_ptr_mutex),
  map_frame_(std::move(map_frame)),
  logger_(node->get_logger()),
  clock_(node->get_clock()),
  tf2_listener_module_(std::move(tf2_listener_module)),
  state_ptr_(std::move(state_ptr)),
  dynamic_map_loading_update_distance_(
    node->declare_parameter<double>("dynamic_map_loading_update_distance")),
  dynamic_map_loading_map_radius_(
    node->declare_parameter<double>("dynamic_map_loading_map_radius")),
  lidar_radius_(node->declare_parameter<double>("lidar_radius"))
{
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  map_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ekf_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "ekf_odom", 100, std::bind(&MapUpdateModule::callback_ekf_odom, this, std::placeholders::_1),
    main_sub_opt);

  loaded_pcd_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS{1}.transient_local());

  pcd_loader_client_ =
    node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>("pcd_loader_service");
  while (!pcd_loader_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(
      logger_,
      "Waiting for pcd loader service. Check if the enable_differential_load in "
      "pointcloud_map_loader is set `true`.");
  }

  double map_update_dt = 1.0;
  auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    node, clock_, period_ns, std::bind(&MapUpdateModule::map_update_timer_callback, this),
    map_callback_group_);
}

void MapUpdateModule::callback_ekf_odom(nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr)
{
  current_position_ = odom_ptr->pose.pose.position;

  if (last_update_position_ == std::nullopt) {
    return;
  }
  double distance = norm_xy(current_position_.value(), last_update_position_.value());
  if (distance + lidar_radius_ > dynamic_map_loading_map_radius_) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1, "Dynamic map loading is not keeping up.");
  }
}

void MapUpdateModule::map_update_timer_callback()
{
  if (current_position_ == std::nullopt) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, *clock_, 1,
      "Cannot find the reference position for map update. Please check if the EKF odometry is "
      "provided to NDT.");
    return;
  }
  if (last_update_position_ == std::nullopt) return;

  // continue only if we should update the map
  if (should_update_map(current_position_.value())) {
    RCLCPP_INFO(logger_, "Start updating NDT map (timer_callback)");
    update_map(current_position_.value());
    last_update_position_ = current_position_;
  }
}

bool MapUpdateModule::should_update_map(const geometry_msgs::msg::Point & position) const
{
  if (last_update_position_ == std::nullopt) return false;
  double distance = norm_xy(position, last_update_position_.value());
  return distance > dynamic_map_loading_update_distance_;
}

void MapUpdateModule::update_map(const geometry_msgs::msg::Point & position)
{
  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();
  request->area.center_x = static_cast<float>(position.x);
  request->area.center_y = static_cast<float>(position.y);
  request->area.radius = static_cast<float>(dynamic_map_loading_map_radius_);
  request->cached_ids = ndt_ptr_->getCurrentMapIDs();

  // // send a request to map_loader
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
  update_ndt(result.get()->new_pointcloud_with_ids, result.get()->ids_to_remove);
  last_update_position_ = position;
}

void MapUpdateModule::update_ndt(
  const std::vector<autoware_map_msgs::msg::PointCloudMapCellWithID> & maps_to_add,
  const std::vector<std::string> & map_ids_to_remove)
{
  RCLCPP_INFO(
    logger_, "Update map (Add: %lu, Remove: %lu)", maps_to_add.size(), map_ids_to_remove.size());
  if (maps_to_add.empty() && map_ids_to_remove.empty()) {
    RCLCPP_INFO(logger_, "Skip map update");
    return;
  }
  const auto exe_start_time = std::chrono::system_clock::now();

  NormalDistributionsTransform backup_ndt = *ndt_ptr_;

  // Add pcd
  for (const auto & map_to_add : maps_to_add) {
    pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
    pcl::fromROSMsg(map_to_add.pointcloud, *map_points_ptr);
    backup_ndt.addTarget(map_points_ptr, map_to_add.cell_id);
  }

  // Remove pcd
  for (const std::string & map_id_to_remove : map_ids_to_remove) {
    backup_ndt.removeTarget(map_id_to_remove);
  }

  backup_ndt.createVoxelKdtree();

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<double>(duration_micro_sec) / 1000.0;
  RCLCPP_INFO(logger_, "Time duration for creating new ndt_ptr: %lf [ms]", exe_time);

  // swap
  (*ndt_ptr_mutex_).lock();
  // ToDo (kminoda): Here negligible NDT copy occurs during the new map loading phase, which should
  // ideally be avoided. But I will leave this for now since I cannot come up with a solution other
  // than using pointer of pointer.
  *ndt_ptr_ = backup_ndt;
  (*ndt_ptr_mutex_).unlock();

  publish_partial_pcd_map();
}

void MapUpdateModule::publish_partial_pcd_map()
{
  pcl::PointCloud<PointTarget> map_pcl = ndt_ptr_->getVoxelPCD();

  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(map_pcl, map_msg);
  map_msg.header.frame_id = "map";

  loaded_pcd_pub_->publish(map_msg);
}
