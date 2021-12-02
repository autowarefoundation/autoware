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

#ifndef MAP_LOADER__ELEVATION_MAP_LOADER_NODE_HPP_
#define MAP_LOADER__ELEVATION_MAP_LOADER_NODE_HPP_

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <filters/filter_chain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <string>
#include <vector>

class ElevationMapLoaderNode : public rclcpp::Node
{
public:
  explicit ElevationMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_map_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_elevation_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevation_map_cloud_;
  void onPointcloudMap(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_map);
  void onVectorMap(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr vector_map);
  void setVerbosityLevelToDebugIfFlagSet();
  void createElevationMapFromPointcloud();
  autoware_utils::LinearRing2d getConvexHull(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud);
  lanelet::ConstLanelets getIntersectedLanelets(
    const autoware_utils::LinearRing2d & convex_hull,
    const lanelet::ConstLanelets & road_lanelets_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr getLaneFilteredPointCloud(
    const lanelet::ConstLanelets & joint_lanelets,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  bool checkPointWithinLanelets(
    const pcl::PointXYZ & point, const lanelet::ConstLanelets & joint_lanelets);
  void publishElevationMap();
  void inpaintElevationMap(const float radius);
  pcl::PointCloud<pcl::PointXYZ>::Ptr createPointcloudFromElevationMap();
  void saveElevationMap();
  float calculateDistancePointFromPlane(
    const pcl::PointXYZ & point, const lanelet::ConstLanelet & lanelet);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pcl_ptr_;
  grid_map::GridMap elevation_map_;
  std::string layer_name_;
  std::filesystem::path elevation_map_path_;
  nlohmann::json hash_json_;
  std::string map_frame_;
  bool use_inpaint_;
  float inpaint_radius_;
  bool already_sub_vector_map_;
  bool already_sub_pointcloud_map_;
  bool use_elevation_map_file_;
  bool use_elevation_map_cloud_publisher_;
  pcl::shared_ptr<grid_map::GridMapPclLoader> grid_map_pcl_loader_;

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

#endif  // MAP_LOADER__ELEVATION_MAP_LOADER_NODE_HPP_
