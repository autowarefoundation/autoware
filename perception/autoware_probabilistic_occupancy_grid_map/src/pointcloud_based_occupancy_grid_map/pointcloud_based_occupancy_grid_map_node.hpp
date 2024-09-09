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

#ifndef POINTCLOUD_BASED_OCCUPANCY_GRID_MAP__POINTCLOUD_BASED_OCCUPANCY_GRID_MAP_NODE_HPP_
#define POINTCLOUD_BASED_OCCUPANCY_GRID_MAP__POINTCLOUD_BASED_OCCUPANCY_GRID_MAP_NODE_HPP_

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_base.hpp"
#include "autoware/probabilistic_occupancy_grid_map/updater/binary_bayes_filter_updater.hpp"
#include "autoware/probabilistic_occupancy_grid_map/updater/ogm_updater_interface.hpp"

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
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

namespace autoware::occupancy_grid_map
{
using builtin_interfaces::msg::Time;
using costmap_2d::OccupancyGridMapInterface;
using costmap_2d::OccupancyGridMapUpdaterInterface;
using laser_geometry::LaserProjection;
using nav2_costmap_2d::Costmap2D;
using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;
using tf2_ros::Buffer;
using tf2_ros::TransformListener;

class PointcloudBasedOccupancyGridMapNode : public rclcpp::Node
{
public:
  explicit PointcloudBasedOccupancyGridMapNode(const rclcpp::NodeOptions & node_options);

private:
  void onPointcloudWithObstacleAndRaw(
    const PointCloud2::ConstSharedPtr & input_obstacle_msg,
    const PointCloud2::ConstSharedPtr & input_raw_msg);
  OccupancyGrid::UniquePtr OccupancyGridMapToMsgPtr(
    const std::string & frame_id, const Time & stamp, const float & robot_pose_z,
    const Costmap2D & occupancy_grid_map);

private:
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_map_pub_;
  message_filters::Subscriber<PointCloud2> obstacle_pointcloud_sub_;
  message_filters::Subscriber<PointCloud2> raw_pointcloud_sub_;
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{};
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_ptr_{};

  std::shared_ptr<Buffer> tf2_{std::make_shared<Buffer>(get_clock())};
  std::shared_ptr<TransformListener> tf2_listener_{std::make_shared<TransformListener>(*tf2_)};

  using SyncPolicy = message_filters::sync_policies::ExactTime<PointCloud2, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_ptr_;

  std::unique_ptr<OccupancyGridMapInterface> occupancy_grid_map_ptr_;
  std::unique_ptr<OccupancyGridMapUpdaterInterface> occupancy_grid_map_updater_ptr_;

  // ROS Parameters
  std::string map_frame_;
  std::string base_link_frame_;
  std::string gridmap_origin_frame_;
  std::string scan_origin_frame_;
  bool use_height_filter_;
  double min_height_;
  double max_height_;
  bool enable_single_frame_mode_;
  bool filter_obstacle_pointcloud_by_raw_pointcloud_;

  // time keeper
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;
};

}  // namespace autoware::occupancy_grid_map

#endif  // POINTCLOUD_BASED_OCCUPANCY_GRID_MAP__POINTCLOUD_BASED_OCCUPANCY_GRID_MAP_NODE_HPP_
