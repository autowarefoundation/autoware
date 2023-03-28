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

#ifndef COMPARE_MAP_SEGMENTATION__VOXEL_GRID_MAP_LOADER_HPP_
#define COMPARE_MAP_SEGMENTATION__VOXEL_GRID_MAP_LOADER_HPP_

#include "compare_map_segmentation/multi_voxel_grid_map_update.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
// create map loader interface for static and dynamic map

template <typename T, typename U>
double distance3D(const T p1, const U p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return dx * dx + dy * dy + dz * dz;
}

template <typename T, typename U>
double distance2D(const T p1, const U p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

class VoxelGridMapLoader
{
protected:
  rclcpp::Logger logger_;
  std::mutex * mutex_ptr_;
  double voxel_leaf_size_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_map_pub_;

public:
  typedef compare_map_segmentation::MultiVoxelGrid<pcl::PointXYZ> MultiVoxelGrid;
  typedef typename pcl::Filter<pcl::PointXYZ>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  explicit VoxelGridMapLoader(
    rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex);
  virtual bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold) = 0;
  bool is_close_to_neighbor_voxels(
    const pcl::PointXYZ & point, const double distance_threshold, const PointCloudPtr & map,
    MultiVoxelGrid & voxel) const;
  bool is_in_voxel(
    const pcl::PointXYZ & src_point, const pcl::PointXYZ & target_point,
    const double distance_threshold, const PointCloudPtr & map, MultiVoxelGrid & voxel) const;

  void publish_downsampled_map(const pcl::PointCloud<pcl::PointXYZ> & downsampled_pc);
  bool is_close_points(
    const pcl::PointXYZ point, const pcl::PointXYZ target_point,
    const double distance_threshold) const;
  std::string * tf_map_input_frame_;
};

//*************************** for Static Map loader Voxel Grid Filter *************
class VoxelGridStaticMapLoader : public VoxelGridMapLoader
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
  MultiVoxelGrid voxel_grid_;
  PointCloudPtr voxel_map_ptr_;

public:
  explicit VoxelGridStaticMapLoader(
    rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex);
  void onMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr map);
  bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold);
};

// *************** for Dynamic and Differential Map loader Voxel Grid Filter *************
class VoxelGridDynamicMapLoader : public VoxelGridMapLoader
{
  struct MapGridVoxelInfo
  {
    MultiVoxelGrid map_cell_voxel_grid;
    PointCloudPtr map_cell_pc_ptr;
    float min_b_x, min_b_y, max_b_x, max_b_y;
  };

  typedef typename std::map<std::string, struct MapGridVoxelInfo> VoxelGridDict;

private:
  VoxelGridDict current_voxel_grid_dict_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_estimated_pose_;
  std::optional<geometry_msgs::msg::Point> current_position_ = std::nullopt;
  std::optional<geometry_msgs::msg::Point> last_updated_position_ = std::nullopt;
  rclcpp::TimerBase::SharedPtr map_update_timer_;
  double map_update_distance_threshold_;
  double map_loader_radius_;
  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr
    map_update_client_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

public:
  explicit VoxelGridDynamicMapLoader(
    rclcpp::Node * node, double leaf_size, std::string * tf_map_input_frame, std::mutex * mutex,
    rclcpp::CallbackGroup::SharedPtr main_callback_group);
  void onEstimatedPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose);

  void timer_callback();
  bool should_update_map() const;
  void request_update_map(const geometry_msgs::msg::Point & position);
  bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold);

  inline pcl::PointCloud<pcl::PointXYZ> getCurrentDownsampledMapPc() const
  {
    pcl::PointCloud<pcl::PointXYZ> output;
    for (const auto & kv : current_voxel_grid_dict_) {
      output = output + *(kv.second.map_cell_pc_ptr);
    }
    return output;
  }
  inline std::vector<std::string> getCurrentMapIDs() const
  {
    std::vector<std::string> current_map_ids{};
    for (auto & kv : current_voxel_grid_dict_) {
      current_map_ids.push_back(kv.first);
    }
    return current_map_ids;
  }
  inline void updateDifferentialMapCells(
    const std::vector<autoware_map_msgs::msg::PointCloudMapCellWithID> & map_cells_to_add,
    std::vector<std::string> map_cell_ids_to_remove)
  {
    for (const auto & map_cell_to_add : map_cells_to_add) {
      addMapCellAndFilter(map_cell_to_add);
    }
    for (size_t i = 0; i < map_cell_ids_to_remove.size(); ++i) {
      removeMapCell(map_cell_ids_to_remove.at(i));
    }
  }

  inline void removeMapCell(const std::string map_cell_id_to_remove)
  {
    (*mutex_ptr_).lock();
    current_voxel_grid_dict_.erase(map_cell_id_to_remove);
    (*mutex_ptr_).unlock();
  }

  inline void addMapCellAndFilter(
    const autoware_map_msgs::msg::PointCloudMapCellWithID & map_cell_to_add)
  {
    pcl::PointCloud<pcl::PointXYZ> map_cell_pc_tmp;
    pcl::fromROSMsg(map_cell_to_add.pointcloud, map_cell_pc_tmp);

    MultiVoxelGrid map_cell_voxel_grid_tmp;
    PointCloudPtr map_cell_downsampled_pc_ptr_tmp;

    auto map_cell_voxel_input_tmp_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_cell_pc_tmp);
    map_cell_voxel_grid_tmp.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    map_cell_downsampled_pc_ptr_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_cell_voxel_grid_tmp.setInputCloud(map_cell_voxel_input_tmp_ptr);
    map_cell_voxel_grid_tmp.setSaveLeafLayout(true);
    map_cell_voxel_grid_tmp.filter(*map_cell_downsampled_pc_ptr_tmp);

    MapGridVoxelInfo current_voxel_grid_list_item;
    // TODO(badai-nguyen): use map cell info from map cell, when map loader I/F is updated
    current_voxel_grid_list_item.min_b_x = map_cell_voxel_grid_tmp.get_min_p()[0];
    current_voxel_grid_list_item.min_b_y = map_cell_voxel_grid_tmp.get_min_p()[1];
    current_voxel_grid_list_item.max_b_x = map_cell_voxel_grid_tmp.get_max_p()[0];
    current_voxel_grid_list_item.max_b_y = map_cell_voxel_grid_tmp.get_max_p()[1];

    current_voxel_grid_list_item.map_cell_voxel_grid.set_voxel_grid(
      &(map_cell_voxel_grid_tmp.leaf_layout_), map_cell_voxel_grid_tmp.get_min_b(),
      map_cell_voxel_grid_tmp.get_max_b(), map_cell_voxel_grid_tmp.get_div_b(),
      map_cell_voxel_grid_tmp.get_divb_mul(), map_cell_voxel_grid_tmp.get_inverse_leaf_size());

    current_voxel_grid_list_item.map_cell_pc_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // for (size_t i = 0; i < map_cell_downsampled_pc_ptr_tmp->points.size(); ++i) {
    //   current_voxel_grid_list_item.map_cell_pc_ptr->points.push_back(
    //     map_cell_downsampled_pc_ptr_tmp->points.at(i));
    // }
    current_voxel_grid_list_item.map_cell_pc_ptr = std::move(map_cell_downsampled_pc_ptr_tmp);
    // add
    (*mutex_ptr_).lock();
    current_voxel_grid_dict_.insert({map_cell_to_add.cell_id, current_voxel_grid_list_item});
    (*mutex_ptr_).unlock();
  }
};

#endif  // COMPARE_MAP_SEGMENTATION__VOXEL_GRID_MAP_LOADER_HPP_
