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

#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

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

namespace
{
bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped = tf2.lookupTransform(
    target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

bool cropPointcloudByHeight(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, const float min_height, const float max_height,
  sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  // Transformed pointcloud on target frame
  sensor_msgs::msg::PointCloud2 trans_input_tmp;
  const bool is_target_frame = (input.header.frame_id == target_frame);
  if (!is_target_frame) {
    if (!transformPointcloud(input, tf2, target_frame, trans_input_tmp)) return false;
  }
  const sensor_msgs::msg::PointCloud2 & trans_input = is_target_frame ? input : trans_input_tmp;

  // Apply height filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(trans_input, "x"),
       iter_y(trans_input, "y"), iter_z(trans_input, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (min_height < *iter_z && *iter_z < max_height) {
      pcl_output->push_back(pcl::PointXYZ(*iter_x, *iter_y, *iter_z));
    }
  }

  // Convert to ros msg
  pcl::toROSMsg(*pcl_output, output);
  output.header = trans_input.header;
  return true;
}

geometry_msgs::msg::Pose getPose(
  const std_msgs::msg::Header & source_header, const tf2_ros::Buffer & tf2,
  const std::string & target_frame)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped = tf2.lookupTransform(
    target_frame, source_header.frame_id, source_header.stamp, rclcpp::Duration::from_seconds(0.5));
  pose = tier4_autoware_utils::transform2pose(tf_stamped.transform);
  return pose;
}
}  // namespace

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
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");
  use_height_filter_ = declare_parameter("use_height_filter", true);
  enable_single_frame_mode_ = declare_parameter("enable_single_frame_mode", false);
  const double map_length{declare_parameter("map_length", 100.0)};
  const double map_width{declare_parameter("map_width", 100.0)};
  const double map_resolution{declare_parameter("map_resolution", 0.5)};
  const bool input_obstacle_pointcloud{declare_parameter("input_obstacle_pointcloud", true)};
  const bool input_obstacle_and_raw_pointcloud{
    declare_parameter("input_obstacle_and_raw_pointcloud", true)};

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

  /* Occupancy grid */
  occupancy_grid_map_updater_ptr_ = std::make_shared<OccupancyGridMapBBFUpdater>(
    map_length / map_resolution, map_width / map_resolution, map_resolution);
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
    constexpr float min_height = -1.0, max_height = 2.0;
    if (!cropPointcloudByHeight(
          *input_obstacle_msg, *tf2_, base_link_frame_, min_height, max_height,
          cropped_obstacle_pc)) {
      return;
    }
    if (!cropPointcloudByHeight(
          *input_raw_msg, *tf2_, base_link_frame_, min_height, max_height, cropped_raw_pc)) {
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
  Pose pose{};
  try {
    transformPointcloud(*laserscan_pc_ptr, *tf2_, map_frame_, trans_laserscan_pc);
    transformPointcloud(filtered_obstacle_pc, *tf2_, map_frame_, trans_obstacle_pc);
    transformPointcloud(filtered_raw_pc, *tf2_, map_frame_, trans_raw_pc);
    pose = getPose(laserscan_pc_ptr->header, *tf2_, map_frame_);
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
    pose.position.x - single_frame_occupancy_grid_map.getSizeInMetersX() / 2,
    pose.position.y - single_frame_occupancy_grid_map.getSizeInMetersY() / 2);
  single_frame_occupancy_grid_map.updateFreespaceCells(trans_raw_pc);
  single_frame_occupancy_grid_map.raytrace2D(trans_laserscan_pc, pose);
  single_frame_occupancy_grid_map.updateOccupiedCells(trans_obstacle_pc);

  if (enable_single_frame_mode_) {
    // publish
    occupancy_grid_map_pub_->publish(OccupancyGridMapToMsgPtr(
      map_frame_, laserscan_pc_ptr->header.stamp, pose.position.z,
      single_frame_occupancy_grid_map));
  } else {
    // Update with bayes filter
    occupancy_grid_map_updater_ptr_->update(single_frame_occupancy_grid_map);

    // publish
    occupancy_grid_map_pub_->publish(OccupancyGridMapToMsgPtr(
      map_frame_, laserscan_pc_ptr->header.stamp, pose.position.z,
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
