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

#ifndef ELEVATION_MAP_LOADER__ELEVATION_MAP_LOADER_NODE_HPP_
#define ELEVATION_MAP_LOADER__ELEVATION_MAP_LOADER_NODE_HPP_

#include <filters/filter_chain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include "tier4_external_api_msgs/msg/map_hash.hpp"
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

class DataManager
{
public:
  DataManager() = default;
  bool isInitialized()
  {
    if (use_lane_filter_) {
      return static_cast<bool>(elevation_map_path_) && static_cast<bool>(map_pcl_ptr_) &&
             static_cast<bool>(lanelet_map_ptr_);
    } else {
      return static_cast<bool>(elevation_map_path_) && static_cast<bool>(map_pcl_ptr_);
    }
  }
  std::unique_ptr<std::filesystem::path> elevation_map_path_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pcl_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  bool use_lane_filter_ = false;
};

class ElevationMapLoaderNode : public rclcpp::Node
{
public:
  explicit ElevationMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_map_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::MapHash>::SharedPtr sub_map_hash_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_elevation_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevation_map_cloud_;
  void onPointcloudMap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_map);
  void onMapHash(const tier4_external_api_msgs::msg::MapHash::ConstSharedPtr map_hash);
  void onVectorMap(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr vector_map);

  void publish();
  void createElevationMap();
  void setVerbosityLevelToDebugIfFlagSet();
  void createElevationMapFromPointcloud();
  tier4_autoware_utils::LinearRing2d getConvexHull(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud);
  lanelet::ConstLanelets getIntersectedLanelets(
    const tier4_autoware_utils::LinearRing2d & convex_hull,
    const lanelet::ConstLanelets & road_lanelets_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr getLaneFilteredPointCloud(
    const lanelet::ConstLanelets & joint_lanelets,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  bool checkPointWithinLanelets(
    const pcl::PointXYZ & point, const lanelet::ConstLanelets & joint_lanelets);
  void inpaintElevationMap(const float radius);
  pcl::PointCloud<pcl::PointXYZ>::Ptr createPointcloudFromElevationMap();
  void saveElevationMap();
  float calculateDistancePointFromPlane(
    const pcl::PointXYZ & point, const lanelet::ConstLanelet & lanelet);

  grid_map::GridMap elevation_map_;
  std::string layer_name_;
  std::string map_frame_;
  std::string elevation_map_directory_;
  bool use_inpaint_;
  float inpaint_radius_;
  bool use_elevation_map_cloud_publisher_;
  pcl::shared_ptr<grid_map::GridMapPclLoader> grid_map_pcl_loader_;

  DataManager data_manager_;
  struct LaneFilter
  {
    float voxel_size_x_;
    float voxel_size_y_;
    float voxel_size_z_;
    float lane_margin_;
    float lane_height_diff_thresh_;
    lanelet::ConstLanelets road_lanelets_;
    bool use_lane_filter_;
  };
  LaneFilter lane_filter_;
};

#endif  // ELEVATION_MAP_LOADER__ELEVATION_MAP_LOADER_NODE_HPP_
