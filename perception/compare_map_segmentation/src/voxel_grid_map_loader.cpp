// Copyright 2023 Autoware Foundation
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

#include "compare_map_segmentation/voxel_grid_map_loader.hpp"

VoxelGridMapLoader::VoxelGridMapLoader(
  rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex)
: logger_(node->get_logger()), voxel_leaf_size_(leaf_size)
{
  tf_map_input_frame_ = tf_map_input_frame;
  mutex_ptr_ = mutex;

  downsampled_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/downsampled_map/pointcloud", rclcpp::QoS{1}.transient_local());
}

bool VoxelGridMapLoader::is_close_points(
  const pcl::PointXYZ point, const pcl::PointXYZ target_point,
  const double distance_threshold) const
{
  if (distance3D(point, target_point) < distance_threshold * distance_threshold) {
    return true;
  }
  return false;
}

void VoxelGridMapLoader::publish_downsampled_map(
  const pcl::PointCloud<pcl::PointXYZ> & downsampled_pc)
{
  sensor_msgs::msg::PointCloud2 downsampled_map_msg;
  pcl::toROSMsg(downsampled_pc, downsampled_map_msg);
  downsampled_map_msg.header.frame_id = "map";
  downsampled_map_pub_->publish(downsampled_map_msg);
}

bool VoxelGridMapLoader::is_close_to_neighbor_voxels(
  const pcl::PointXYZ & point, const double distance_threshold, const PointCloudPtr & map,
  MultiVoxelGrid & voxel) const
{
  // check map downsampled pc
  if (map == NULL) {
    return false;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y, point.z), point, distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y - distance_threshold, point.z - distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y - distance_threshold, point.z), point, distance_threshold,
        map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y - distance_threshold, point.z + distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y, point.z - distance_threshold), point, distance_threshold,
        map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y, point.z + distance_threshold), point, distance_threshold,
        map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y + distance_threshold, point.z - distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y + distance_threshold, point.z), point, distance_threshold,
        map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x, point.y + distance_threshold, point.z + distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }

  if (is_in_voxel(
        pcl::PointXYZ(
          point.x - distance_threshold, point.y - distance_threshold, point.z - distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x - distance_threshold, point.y - distance_threshold, point.z), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(
          point.x - distance_threshold, point.y - distance_threshold, point.z + distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x - distance_threshold, point.y, point.z - distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x - distance_threshold, point.y, point.z), point, distance_threshold,
        map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x - distance_threshold, point.y, point.z + distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(
          point.x - distance_threshold, point.y + distance_threshold, point.z - distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x - distance_threshold, point.y + distance_threshold, point.z), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(
          point.x - distance_threshold, point.y + distance_threshold, point.z + distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }

  if (is_in_voxel(
        pcl::PointXYZ(
          point.x + distance_threshold, point.y - distance_threshold, point.z - distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x + distance_threshold, point.y - distance_threshold, point.z), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(
          point.x + distance_threshold, point.y - distance_threshold, point.z + distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x + distance_threshold, point.y, point.z - distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x + distance_threshold, point.y, point.z), point, distance_threshold,
        map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x + distance_threshold, point.y, point.z + distance_threshold), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(
          point.x + distance_threshold, point.y + distance_threshold, point.z - distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(point.x + distance_threshold, point.y + distance_threshold, point.z), point,
        distance_threshold, map, voxel)) {
    return true;
  }
  if (is_in_voxel(
        pcl::PointXYZ(
          point.x + distance_threshold, point.y + distance_threshold, point.z + distance_threshold),
        point, distance_threshold, map, voxel)) {
    return true;
  }

  return false;
}

bool VoxelGridMapLoader::is_in_voxel(
  const pcl::PointXYZ & src_point, const pcl::PointXYZ & target_point,
  const double distance_threshold, const PointCloudPtr & map, MultiVoxelGrid & voxel) const
{
  int voxel_index =
    voxel.getCentroidIndexAt(voxel.getGridCoordinates(src_point.x, src_point.y, src_point.z));
  if (voxel_index != -1) {  // not empty voxel
    const double dist_x = map->points.at(voxel_index).x - target_point.x;
    const double dist_y = map->points.at(voxel_index).y - target_point.y;
    const double dist_z = map->points.at(voxel_index).z - target_point.z;
    const double sqr_distance = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
    if (sqr_distance < distance_threshold * distance_threshold) {
      return true;
    }
  }
  return false;
}

//*************************** for Static Map loader Voxel Grid Filter *************

VoxelGridStaticMapLoader::VoxelGridStaticMapLoader(
  rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex)
: VoxelGridMapLoader(node, leaf_size, tf_map_input_frame, mutex)
{
  sub_map_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS{1}.transient_local(),
    std::bind(&VoxelGridStaticMapLoader::onMapCallback, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "VoxelGridStaticMapLoader initialized.\n");
}

void VoxelGridStaticMapLoader::onMapCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr map)
{
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg<pcl::PointXYZ>(*map, map_pcl);
  const auto map_pcl_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_pcl);
  *tf_map_input_frame_ = map_pcl_ptr->header.frame_id;
  (*mutex_ptr_).lock();
  voxel_map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid_.setInputCloud(map_pcl_ptr);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr_);
  (*mutex_ptr_).unlock();
  publish_downsampled_map(*voxel_map_ptr_);
}
bool VoxelGridStaticMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  if (is_close_to_neighbor_voxels(point, distance_threshold, voxel_map_ptr_, voxel_grid_)) {
    return true;
  }
  return false;
}
//*************** for Dynamic and Differential Map loader Voxel Grid Filter *************

VoxelGridDynamicMapLoader::VoxelGridDynamicMapLoader(
  rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex,
  rclcpp::CallbackGroup::SharedPtr main_callback_group)
: VoxelGridMapLoader(node, leaf_size, tf_map_input_frame, mutex)
{
  auto timer_interval_ms = node->declare_parameter<int>("timer_interval_ms");
  map_update_distance_threshold_ = node->declare_parameter<double>("map_update_distance_threshold");
  map_loader_radius_ = node->declare_parameter<double>("map_loader_radius");
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  sub_estimated_pose_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_with_covariance", rclcpp::QoS{1},
    std::bind(&VoxelGridDynamicMapLoader::onEstimatedPoseCallback, this, std::placeholders::_1),
    main_sub_opt);
  RCLCPP_INFO(logger_, "VoxelGridDynamicMapLoader initialized.\n");

  client_callback_group_ =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_update_client_ = node->create_client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>(
    "map_loader_service", rmw_qos_profile_services_default, client_callback_group_);

  while (!map_update_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(logger_, "service not available, waiting again ...");
  }

  timer_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_update_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(timer_interval_ms),
    std::bind(&VoxelGridDynamicMapLoader::timer_callback, this), timer_callback_group_);
}

void VoxelGridDynamicMapLoader::onEstimatedPoseCallback(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  current_position_ = msg->pose.pose.position;
}

bool VoxelGridDynamicMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, const double distance_threshold)
{
  if (current_voxel_grid_dict_.size() == 0) {
    return false;
  }
  std::vector<std::string> neighbor_map_cells_id;

  for (auto & kv : current_voxel_grid_dict_) {
    if (point.x < kv.second.min_b_x - distance_threshold) {
      continue;
    }
    if (point.y < kv.second.min_b_y - distance_threshold) {
      continue;
    }
    if (point.x > kv.second.max_b_x + distance_threshold) {
      continue;
    }
    if (point.y > kv.second.max_b_y + distance_threshold) {
      continue;
    }
    // the map cell is found
    neighbor_map_cells_id.push_back(kv.first);
    // check distance
    if (kv.second.map_cell_pc_ptr == NULL) {
      continue;
    }
    if (is_close_to_neighbor_voxels(
          point, distance_threshold, kv.second.map_cell_pc_ptr, kv.second.map_cell_voxel_grid)) {
      return true;
    }
  }
  return false;
}
void VoxelGridDynamicMapLoader::timer_callback()
{
  if (current_position_ == std::nullopt) {
    return;
  }
  if (last_updated_position_ == std::nullopt) {
    request_update_map(current_position_.value());
    last_updated_position_ = current_position_;
    return;
  }

  if (should_update_map()) {
    last_updated_position_ = current_position_;
    request_update_map((current_position_.value()));
    last_updated_position_ = current_position_;
  }
}

bool VoxelGridDynamicMapLoader::should_update_map() const
{
  if (
    distance2D(current_position_.value(), last_updated_position_.value()) >
    map_update_distance_threshold_) {
    return true;
  }
  return false;
}

void VoxelGridDynamicMapLoader::request_update_map(const geometry_msgs::msg::Point & position)
{
  auto request = std::make_shared<autoware_map_msgs::srv::GetDifferentialPointCloudMap::Request>();
  request->area.center = position;
  request->area.radius = map_loader_radius_;
  request->cached_ids = getCurrentMapIDs();

  auto result{map_update_client_->async_send_request(
    request,
    [](rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedFuture) {})};

  std::future_status status = result.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    RCLCPP_INFO(logger_, "Waiting for response...\n");
    if (!rclcpp::ok()) {
      return;
    }
    status = result.wait_for(std::chrono::seconds(1));
  }

  //
  if (status == std::future_status::ready) {
    if (
      result.get()->new_pointcloud_with_ids.size() > 0 || result.get()->ids_to_remove.size() > 0) {
      updateDifferentialMapCells(
        result.get()->new_pointcloud_with_ids, result.get()->ids_to_remove);
    }
  }
  publish_downsampled_map(getCurrentDownsampledMapPc());
}
