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

#ifndef LASERSCAN_BASED_OCCUPANCY_GRID_MAP__LASERSCAN_BASED_OCCUPANCY_GRID_MAP_NODE_HPP_
#define LASERSCAN_BASED_OCCUPANCY_GRID_MAP__LASERSCAN_BASED_OCCUPANCY_GRID_MAP_NODE_HPP_

#include "laserscan_based_occupancy_grid_map/occupancy_grid_map.hpp"
#include "updater/occupancy_grid_map_binary_bayes_filter_updater.hpp"
#include "updater/occupancy_grid_map_updater_interface.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace occupancy_grid_map
{
using builtin_interfaces::msg::Time;
using costmap_2d::OccupancyGridMapUpdaterInterface;
using laser_geometry::LaserProjection;
using nav2_costmap_2d::Costmap2D;
using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;
using tf2_ros::Buffer;
using tf2_ros::TransformListener;

class LaserscanBasedOccupancyGridMapNode : public rclcpp::Node
{
public:
  explicit LaserscanBasedOccupancyGridMapNode(const rclcpp::NodeOptions & node_options);

private:
  PointCloud2::SharedPtr convertLaserscanToPointCLoud2(const LaserScan::ConstSharedPtr & input);
  void onLaserscanPointCloud2WithObstacleAndRaw(
    const LaserScan::ConstSharedPtr & input_laserscan_msg,
    const PointCloud2::ConstSharedPtr & input_obstacle_msg,
    const PointCloud2::ConstSharedPtr & input_raw_msg);
  OccupancyGrid::UniquePtr OccupancyGridMapToMsgPtr(
    const std::string & frame_id, const Time & stamp, const float & robot_pose_z,
    const Costmap2D & occupancy_grid_map);
  inline void onDummyPointCloud2(const LaserScan::ConstSharedPtr & input)
  {
    PointCloud2 dummy;
    sensor_msgs::PointCloud2Modifier modifier(dummy);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    dummy.header = input->header;
    passthrough_.add(std::make_shared<PointCloud2>(dummy));
  }

private:
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_map_pub_;
  message_filters::Subscriber<LaserScan> laserscan_sub_;
  message_filters::Subscriber<PointCloud2> obstacle_pointcloud_sub_;
  message_filters::Subscriber<PointCloud2> raw_pointcloud_sub_;
  message_filters::PassThrough<PointCloud2> passthrough_;

  std::shared_ptr<Buffer> tf2_{std::make_shared<Buffer>(get_clock())};
  std::shared_ptr<TransformListener> tf2_listener_{std::make_shared<TransformListener>(*tf2_)};

  using SyncPolicy = message_filters::sync_policies::ExactTime<LaserScan, PointCloud2, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_ptr_;

  LaserProjection laserscan2pointcloud_converter_;

  std::shared_ptr<OccupancyGridMapUpdaterInterface> occupancy_grid_map_updater_ptr_;

  // ROS Parameters
  std::string map_frame_;
  std::string base_link_frame_;
  bool use_height_filter_;
  bool enable_single_frame_mode_;
};

}  // namespace occupancy_grid_map

#endif  // LASERSCAN_BASED_OCCUPANCY_GRID_MAP__LASERSCAN_BASED_OCCUPANCY_GRID_MAP_NODE_HPP_
