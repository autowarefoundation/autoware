// Copyright 2021 Tier IV, Inc.
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

#include "laserscan_based_occupancy_grid_map/laserscan_based_occupancy_grid_map_node.hpp"

#include "cost_value.hpp"
#include "utils/utils.hpp"

#include <pcl_ros/transforms.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <memory>
#include <string>

namespace occupancy_grid_map
{
using costmap_2d::OccupancyGridMap;
using costmap_2d::OccupancyGridMapBBFUpdater;
using geometry_msgs::msg::Pose;

LaserscanBasedOccupancyGridMapNode::LaserscanBasedOccupancyGridMapNode(
  const rclcpp::NodeOptions & node_options)
: Node("laserscan_based_occupancy_grid_map_node", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  /* params */
  map_frame_ = this->declare_parameter<std::string>("map_frame");
  base_link_frame_ = this->declare_parameter<std::string>("base_link_frame");
  gridmap_origin_frame_ = this->declare_parameter<std::string>("gridmap_origin_frame");
  scan_origin_frame_ = this->declare_parameter<std::string>("scan_origin_frame");
  use_height_filter_ = this->declare_parameter<bool>("height_filter.use_height_filter");
  min_height_ = this->declare_parameter<double>("height_filter.min_height");
  max_height_ = this->declare_parameter<double>("height_filter.max_height");
  enable_single_frame_mode_ = this->declare_parameter<bool>("enable_single_frame_mode");
  const double map_length = this->declare_parameter<double>("map_length");
  const double map_width = this->declare_parameter<double>("map_width");
  const double map_resolution = this->declare_parameter<double>("map_resolution");
  const bool input_obstacle_pointcloud = this->declare_parameter<bool>("input_obstacle_pointcloud");
  const bool input_obstacle_and_raw_pointcloud =
    this->declare_parameter<bool>("input_obstacle_and_raw_pointcloud");

  /* Subscriber and publisher */
  laserscan_sub_.subscribe(
    this, "~/input/laserscan", rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
  obstacle_pointcloud_sub_.subscribe(
    this, "~/input/obstacle_pointcloud",
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
  raw_pointcloud_sub_.subscribe(
    this, "~/input/raw_pointcloud", rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile());
  // add dummy callback to enable passthrough filter
  laserscan_sub_.registerCallback(
    std::bind(&LaserscanBasedOccupancyGridMapNode::onDummyPointCloud2, this, _1));
  if (input_obstacle_and_raw_pointcloud) {
    sync_ptr_ = std::make_shared<Sync>(
      SyncPolicy(5), laserscan_sub_, obstacle_pointcloud_sub_, raw_pointcloud_sub_);
  } else if (input_obstacle_pointcloud) {
    sync_ptr_ =
      std::make_shared<Sync>(SyncPolicy(3), laserscan_sub_, obstacle_pointcloud_sub_, passthrough_);
  } else {
    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(3), laserscan_sub_, passthrough_, passthrough_);
  }

  sync_ptr_->registerCallback(std::bind(
    &LaserscanBasedOccupancyGridMapNode::onLaserscanPointCloud2WithObstacleAndRaw, this, _1, _2,
    _3));
  occupancy_grid_map_pub_ = create_publisher<OccupancyGrid>("~/output/occupancy_grid_map", 1);

  const std::string updater_type = this->declare_parameter<std::string>("updater_type");
  if (updater_type == "binary_bayes_filter") {
    occupancy_grid_map_updater_ptr_ = std::make_shared<OccupancyGridMapBBFUpdater>(
      map_length / map_resolution, map_width / map_resolution, map_resolution);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "specified occupancy grid map updater type [%s] is not found, use binary_bayes_filter",
      updater_type.c_str());
    occupancy_grid_map_updater_ptr_ = std::make_shared<OccupancyGridMapBBFUpdater>(
      map_length / map_resolution, map_width / map_resolution, map_resolution);
  }
  occupancy_grid_map_updater_ptr_->initRosParam(*this);
}

PointCloud2::SharedPtr LaserscanBasedOccupancyGridMapNode::convertLaserscanToPointCLoud2(
  const LaserScan::ConstSharedPtr & input)
{
  // check over max range point
  const float max_range =
    static_cast<float>(occupancy_grid_map_updater_ptr_->getSizeInCellsX()) * 0.5f +
    occupancy_grid_map_updater_ptr_->getResolution();
  constexpr float epsilon = 0.001;
  LaserScan laserscan = *input;
  laserscan.range_max = max_range;
  for (auto & range : laserscan.ranges) {
    if (max_range < range || std::isinf(range)) {
      range = max_range - epsilon;
    }
  }

  // convert to pointcloud
  PointCloud2::SharedPtr pointcloud_ptr = std::make_shared<PointCloud2>();
  pointcloud_ptr->header = laserscan.header;
  laserscan2pointcloud_converter_.transformLaserScanToPointCloud(
    laserscan.header.frame_id, laserscan, *pointcloud_ptr, *tf2_);

  return pointcloud_ptr;
}

void LaserscanBasedOccupancyGridMapNode::onLaserscanPointCloud2WithObstacleAndRaw(
  const LaserScan::ConstSharedPtr & input_laserscan_msg,
  const PointCloud2::ConstSharedPtr & input_obstacle_msg,
  const PointCloud2::ConstSharedPtr & input_raw_msg)
{
  // Laserscan to pointcloud2
  PointCloud2::ConstSharedPtr laserscan_pc_ptr = convertLaserscanToPointCLoud2(input_laserscan_msg);

  // Apply height filter
  PointCloud2 cropped_obstacle_pc{};
  PointCloud2 cropped_raw_pc{};
  if (use_height_filter_) {
    if (!utils::cropPointcloudByHeight(
          *input_obstacle_msg, *tf2_, base_link_frame_, min_height_, max_height_,
          cropped_obstacle_pc)) {
      return;
    }
    if (!utils::cropPointcloudByHeight(
          *input_raw_msg, *tf2_, base_link_frame_, min_height_, max_height_, cropped_raw_pc)) {
      return;
    }
  }
  const PointCloud2 & filtered_obstacle_pc =
    use_height_filter_ ? cropped_obstacle_pc : *input_obstacle_msg;
  const PointCloud2 & filtered_raw_pc = use_height_filter_ ? cropped_raw_pc : *input_raw_msg;

  // Transform pointcloud and get frame pose
  PointCloud2 trans_laserscan_pc{};
  PointCloud2 trans_obstacle_pc{};
  PointCloud2 trans_raw_pc{};
  Pose gridmap_origin{};
  Pose scan_origin{};
  try {
    utils::transformPointcloud(*laserscan_pc_ptr, *tf2_, map_frame_, trans_laserscan_pc);
    utils::transformPointcloud(filtered_obstacle_pc, *tf2_, map_frame_, trans_obstacle_pc);
    utils::transformPointcloud(filtered_raw_pc, *tf2_, map_frame_, trans_raw_pc);
    gridmap_origin =
      utils::getPose(laserscan_pc_ptr->header.stamp, *tf2_, gridmap_origin_frame_, map_frame_);
    scan_origin =
      utils::getPose(laserscan_pc_ptr->header.stamp, *tf2_, scan_origin_frame_, map_frame_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(get_logger(), ex.what());
    return;
  }

  // Create single frame occupancy grid map
  OccupancyGridMap single_frame_occupancy_grid_map(
    occupancy_grid_map_updater_ptr_->getSizeInCellsX(),
    occupancy_grid_map_updater_ptr_->getSizeInCellsY(),
    occupancy_grid_map_updater_ptr_->getResolution());
  single_frame_occupancy_grid_map.updateOrigin(
    gridmap_origin.position.x - single_frame_occupancy_grid_map.getSizeInMetersX() / 2,
    gridmap_origin.position.y - single_frame_occupancy_grid_map.getSizeInMetersY() / 2);
  single_frame_occupancy_grid_map.updateFreespaceCells(trans_raw_pc);
  single_frame_occupancy_grid_map.raytrace2D(trans_laserscan_pc, scan_origin);
  single_frame_occupancy_grid_map.updateOccupiedCells(trans_obstacle_pc);

  if (enable_single_frame_mode_) {
    // publish
    occupancy_grid_map_pub_->publish(OccupancyGridMapToMsgPtr(
      map_frame_, laserscan_pc_ptr->header.stamp, gridmap_origin.position.z,
      single_frame_occupancy_grid_map));
  } else {
    // Update with bayes filter
    occupancy_grid_map_updater_ptr_->update(single_frame_occupancy_grid_map);

    // publish
    occupancy_grid_map_pub_->publish(OccupancyGridMapToMsgPtr(
      map_frame_, laserscan_pc_ptr->header.stamp, gridmap_origin.position.z,
      *occupancy_grid_map_updater_ptr_));
  }
}

OccupancyGrid::UniquePtr LaserscanBasedOccupancyGridMapNode::OccupancyGridMapToMsgPtr(
  const std::string & frame_id, const Time & stamp, const float & robot_pose_z,
  const Costmap2D & occupancy_grid_map)
{
  auto msg_ptr = std::make_unique<OccupancyGrid>();

  msg_ptr->header.frame_id = frame_id;
  msg_ptr->header.stamp = stamp;
  msg_ptr->info.resolution = occupancy_grid_map.getResolution();

  msg_ptr->info.width = occupancy_grid_map.getSizeInCellsX();
  msg_ptr->info.height = occupancy_grid_map.getSizeInCellsY();

  double wx{};
  double wy{};
  occupancy_grid_map.mapToWorld(0, 0, wx, wy);
  msg_ptr->info.origin.position.x = occupancy_grid_map.getOriginX();
  msg_ptr->info.origin.position.y = occupancy_grid_map.getOriginY();
  msg_ptr->info.origin.position.z = robot_pose_z;
  msg_ptr->info.origin.orientation.w = 1.0;

  msg_ptr->data.resize(msg_ptr->info.width * msg_ptr->info.height);

  unsigned char * data = occupancy_grid_map.getCharMap();
  for (unsigned int i = 0; i < msg_ptr->data.size(); ++i) {
    msg_ptr->data[i] = occupancy_cost_value::cost_translation_table[data[i]];
  }
  return msg_ptr;
}

}  // namespace occupancy_grid_map

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_map::LaserscanBasedOccupancyGridMapNode)
