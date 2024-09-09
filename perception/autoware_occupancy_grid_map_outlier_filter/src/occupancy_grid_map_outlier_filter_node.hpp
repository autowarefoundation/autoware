// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef OCCUPANCY_GRID_MAP_OUTLIER_FILTER_NODE_HPP_
#define OCCUPANCY_GRID_MAP_OUTLIER_FILTER_NODE_HPP_

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/universe_utils/ros/published_time_publisher.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <pcl/common/impl/common.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware::occupancy_grid_map_outlier_filter
{
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;
using sensor_msgs::msg::PointCloud2;
using std_msgs::msg::Header;

class RadiusSearch2dFilter
{
public:
  explicit RadiusSearch2dFilter(rclcpp::Node & node);
  void filter(
    const PointCloud2 & input, const Pose & pose, PointCloud2 & output, PointCloud2 & outlier);
  void filter(
    const PointCloud2 & high_conf_input, const PointCloud2 & low_conf_input, const Pose & pose,
    PointCloud2 & output, PointCloud2 & outlier);

private:
  float search_radius_;
  float min_points_and_distance_ratio_;
  int min_points_;
  int max_points_;
  long unsigned int max_filter_points_nb_;
  pcl::search::Search<pcl::PointXY>::Ptr kd_tree_;
};

class OccupancyGridMapOutlierFilterComponent : public rclcpp::Node
{
public:
  explicit OccupancyGridMapOutlierFilterComponent(const rclcpp::NodeOptions & options);

private:
  void onOccupancyGridMapAndPointCloud2(
    const OccupancyGrid::ConstSharedPtr & input_occupancy_grid_map,
    const PointCloud2::ConstSharedPtr & input_pointcloud);
  void filterByOccupancyGridMap(
    const OccupancyGrid & occupancy_grid_map, const PointCloud2 & pointcloud,
    PointCloud2 & high_confidence, PointCloud2 & low_confidence, PointCloud2 & out_ogm);
  void splitPointCloudFrontBack(
    const PointCloud2::ConstSharedPtr & input_pc, PointCloud2 & front_pc, PointCloud2 & behind_pc);
  void initializerPointCloud2(const PointCloud2 & input, PointCloud2 & output);
  void finalizePointCloud2(const PointCloud2 & input, PointCloud2 & output);
  void concatPointCloud2(PointCloud2 & output, const PointCloud2 & input);

private:
  class Debugger
  {
  public:
    explicit Debugger(OccupancyGridMapOutlierFilterComponent & node);
    void publishOutlier(const PointCloud2 & input, const Header & header);
    void publishHighConfidence(const PointCloud2 & input, const Header & header);
    void publishLowConfidence(const PointCloud2 & input, const Header & header);

  private:
    void transformToBaseLink(
      const PointCloud2 & input, const Header & header, PointCloud2 & output);
    rclcpp::Publisher<PointCloud2>::SharedPtr outlier_pointcloud_pub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr low_confidence_pointcloud_pub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr high_confidence_pointcloud_pub_;
    const OccupancyGridMapOutlierFilterComponent & node_;
  };

private:
  // publishers and subscribers
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
  message_filters::Subscriber<OccupancyGrid> occupancy_grid_map_sub_;
  message_filters::Subscriber<PointCloud2> pointcloud_sub_;
  using SyncPolicy = message_filters::sync_policies::ExactTime<OccupancyGrid, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Sync> sync_ptr_;

  // tf
  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // 2d outlier filter
  std::shared_ptr<RadiusSearch2dFilter> radius_search_2d_filter_ptr_;

  // Debugger
  std::shared_ptr<Debugger> debugger_ptr_;
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  // ROS Parameters
  std::string map_frame_;
  std::string base_link_frame_;
  int cost_threshold_;

  // time keeper
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;
};
}  // namespace autoware::occupancy_grid_map_outlier_filter

#endif  // OCCUPANCY_GRID_MAP_OUTLIER_FILTER_NODE_HPP_
