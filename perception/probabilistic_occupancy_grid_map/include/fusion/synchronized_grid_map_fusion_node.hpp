// Copyright 2023 Tier IV, Inc.
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

#ifndef FUSION__SYNCHRONIZED_GRID_MAP_FUSION_NODE_HPP_
#define FUSION__SYNCHRONIZED_GRID_MAP_FUSION_NODE_HPP_

#include "fusion/single_frame_fusion_policy.hpp"
#include "pointcloud_based_occupancy_grid_map/occupancy_grid_map_fixed.hpp"
#include "updater/occupancy_grid_map_log_odds_bayes_filter_updater.hpp"
#include "updater/occupancy_grid_map_updater_interface.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace synchronized_grid_map_fusion
{

using costmap_2d::OccupancyGridMapFixedBlindSpot;
using costmap_2d::OccupancyGridMapLOBFUpdater;
using costmap_2d::OccupancyGridMapUpdaterInterface;
using geometry_msgs::msg::Pose;

class GridMapFusionNode : public rclcpp::Node
{
public:
  explicit GridMapFusionNode(const rclcpp::NodeOptions & node_options);

private:
  void onGridMap(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & occupancy_grid_msg,
    const std::string & input_topic);
  nav_msgs::msg::OccupancyGrid::UniquePtr OccupancyGridMapToMsgPtr(
    const std::string & frame_id, const builtin_interfaces::msg::Time & stamp,
    const float & robot_pose_z, const nav2_costmap_2d::Costmap2D & occupancy_grid_map);

  nav2_costmap_2d::Costmap2D OccupancyGridMsgToCostmap2D(
    const nav_msgs::msg::OccupancyGrid & occupancy_grid_map);
  OccupancyGridMapFixedBlindSpot OccupancyGridMsgToGridMap(
    const nav_msgs::msg::OccupancyGrid & occupancy_grid_map);
  OccupancyGridMapFixedBlindSpot SingleFrameOccupancyFusion(
    std::vector<OccupancyGridMapFixedBlindSpot> & occupancy_grid_maps,
    const builtin_interfaces::msg::Time latest_stamp, const std::vector<double> & weights);

  void updateGridMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & occupancy_grid_msg);

  void setPeriod(const int64_t new_period);
  void timer_callback();
  void publish();

private:
  // Publisher and Subscribers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr single_frame_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> grid_map_subs_;
  std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{};
  std::unique_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_ptr_{};

  // Topics manager
  std::size_t num_input_topics_{1};
  std::vector<std::string> input_topics_;
  std::vector<double> input_offset_sec_;
  std::vector<double> input_topic_weights_;
  std::map<std::string, double> input_topic_weights_map_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Timer to manage the timeout of the occupancy grid map
  rclcpp::TimerBase::SharedPtr timer_;
  double timeout_sec_{};
  double match_threshold_sec_{};

  // cache for fusion
  std::mutex mutex_;
  std::shared_ptr<OccupancyGridMapUpdaterInterface>
    occupancy_grid_map_updater_ptr_;  // contains fused grid map
  std::map<std::string, nav_msgs::msg::OccupancyGrid::ConstSharedPtr>
    gridmap_dict_;  // temporary cache for grid map message
  std::map<std::string, nav_msgs::msg::OccupancyGrid::ConstSharedPtr>
    gridmap_dict_tmp_;                        // second cache for grid map message
  std::map<std::string, double> offset_map_;  // time offset for each grid map

  // grid map parameters
  std::string map_frame_;
  std::string base_link_frame_;
  std::string gridmap_origin_frame_;
  float fusion_map_length_x_;
  float fusion_map_length_y_;
  float fusion_map_resolution_;
  fusion_policy::FusionMethod fusion_method_;
};

}  // namespace synchronized_grid_map_fusion

#endif  // FUSION__SYNCHRONIZED_GRID_MAP_FUSION_NODE_HPP_
