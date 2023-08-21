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

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

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

template <typename PointT>
class VoxelGridEx : public pcl::VoxelGrid<PointT>
{
protected:
  using pcl::VoxelGrid<PointT>::save_leaf_layout_;
  using pcl::VoxelGrid<PointT>::min_b_;
  using pcl::VoxelGrid<PointT>::max_b_;
  using pcl::VoxelGrid<PointT>::divb_mul_;
  using pcl::VoxelGrid<PointT>::div_b_;
  using pcl::VoxelGrid<PointT>::inverse_leaf_size_;

  using PointCloud = typename pcl::Filter<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  using pcl::VoxelGrid<PointT>::leaf_layout_;

  inline void set_voxel_grid(
    std::vector<int> * leaf_layout, const Eigen::Vector4i & min_b, const Eigen::Vector4i & max_b,
    const Eigen::Vector4i & div_b, const Eigen::Vector4i & divb_mul,
    const Eigen::Array4f & inverse_leaf_size)
  {
    leaf_layout_ = std::move(*leaf_layout);
    min_b_ = min_b;
    max_b_ = max_b;
    div_b_ = div_b;
    divb_mul_ = divb_mul;
    inverse_leaf_size_ = inverse_leaf_size;
  }

  inline Eigen::Vector4i get_min_b() const { return min_b_; }
  inline Eigen::Vector4i get_divb_mul() const { return divb_mul_; }
  inline Eigen::Vector4i get_max_b() const { return max_b_; }
  inline Eigen::Vector4i get_div_b() const { return div_b_; }
  inline Eigen::Array4f get_inverse_leaf_size() const { return inverse_leaf_size_; }
  inline std::vector<int> getLeafLayout() { return (leaf_layout_); }
};

class VoxelGridMapLoader
{
protected:
  rclcpp::Logger logger_;
  std::mutex * mutex_ptr_;
  double voxel_leaf_size_;
  double voxel_leaf_size_z_;
  double downsize_ratio_z_axis_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_map_pub_;
  bool debug_ = false;

public:
  typedef VoxelGridEx<pcl::PointXYZ> VoxelGridPointXYZ;
  typedef typename pcl::Filter<pcl::PointXYZ>::PointCloud PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  explicit VoxelGridMapLoader(
    rclcpp::Node * node, double leaf_size, double downsize_ratio_z_axis,
    std::string * tf_map_input_frame, std::mutex * mutex);

  virtual bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold) = 0;
  bool is_close_to_neighbor_voxels(
    const pcl::PointXYZ & point, const double distance_threshold, VoxelGridPointXYZ & voxel,
    pcl::search::Search<pcl::PointXYZ>::Ptr tree) const;
  bool is_close_to_neighbor_voxels(
    const pcl::PointXYZ & point, const double distance_threshold, const PointCloudPtr & map,
    VoxelGridPointXYZ & voxel) const;
  bool is_in_voxel(
    const pcl::PointXYZ & src_point, const pcl::PointXYZ & target_point,
    const double distance_threshold, const PointCloudPtr & map, VoxelGridPointXYZ & voxel) const;

  void publish_downsampled_map(const pcl::PointCloud<pcl::PointXYZ> & downsampled_pc);
  bool is_close_points(
    const pcl::PointXYZ point, const pcl::PointXYZ target_point,
    const double distance_threshold) const;
  std::string * tf_map_input_frame_;
};

class VoxelGridStaticMapLoader : public VoxelGridMapLoader
{
protected:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
  VoxelGridPointXYZ voxel_grid_;
  PointCloudPtr voxel_map_ptr_;

public:
  explicit VoxelGridStaticMapLoader(
    rclcpp::Node * node, double leaf_size, double downsize_ratio_z_axis,
    std::string * tf_map_input_frame, std::mutex * mutex);
  virtual void onMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr map);
  virtual bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold);
};

class VoxelGridDynamicMapLoader : public VoxelGridMapLoader
{
protected:
  struct MapGridVoxelInfo
  {
    VoxelGridPointXYZ map_cell_voxel_grid;
    PointCloudPtr map_cell_pc_ptr;
    float min_b_x, min_b_y, max_b_x, max_b_y;
    pcl::search::Search<pcl::PointXYZ>::Ptr map_cell_kdtree;
  };

  typedef typename std::map<std::string, struct MapGridVoxelInfo> VoxelGridDict;

  /** \brief Map to hold loaded map grid id and it's voxel filter */
  VoxelGridDict current_voxel_grid_dict_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_kinematic_state_;

  std::optional<geometry_msgs::msg::Point> current_position_ = std::nullopt;
  std::optional<geometry_msgs::msg::Point> last_updated_position_ = std::nullopt;
  rclcpp::TimerBase::SharedPtr map_update_timer_;
  double map_update_distance_threshold_;
  double map_loader_radius_;
  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr
    map_update_client_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  /** Map grid size. It might be defined by using metadata */
  double map_grid_size_x_ = -1.0;
  double map_grid_size_y_ = -1.0;

  double origin_x_remainder_ = 0.0;
  double origin_y_remainder_ = 0.0;

  /** \brief Array to hold loaded map grid positions for fast map grid searching.
   */
  std::vector<std::shared_ptr<MapGridVoxelInfo>> current_voxel_grid_array_;

  /** \brief Array size in x axis */
  int map_grids_x_;
  /** \brief Array size in y axis */
  int map_grids_y_;

  /** \brief x-coordinate of map grid which should belong to array[0][0] */
  float origin_x_;
  /** \brief y-coordinate of map grid which should belong to array[0][0] */
  float origin_y_;

public:
  explicit VoxelGridDynamicMapLoader(
    rclcpp::Node * node, double leaf_size, double downsize_ratio_z_axis,
    std::string * tf_map_input_frame, std::mutex * mutex,
    rclcpp::CallbackGroup::SharedPtr main_callback_group);
  void onEstimatedPoseCallback(nav_msgs::msg::Odometry::ConstSharedPtr pose);

  void timer_callback();
  bool should_update_map() const;
  void request_update_map(const geometry_msgs::msg::Point & position);
  virtual bool is_close_to_map(const pcl::PointXYZ & point, const double distance_threshold);
  /** \brief Check if point close to map pointcloud in the */
  bool is_close_to_next_map_grid(
    const pcl::PointXYZ & point, const int current_map_grid_index, const double distance_threshold);

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

    updateVoxelGridArray();
  }

  /** Update loaded map grid array for fast searching*/
  virtual inline void updateVoxelGridArray()
  {
    origin_x_ = std::floor((current_position_.value().x - map_loader_radius_) / map_grid_size_x_) *
                  map_grid_size_x_ +
                origin_x_remainder_;
    origin_y_ = std::floor((current_position_.value().y - map_loader_radius_) / map_grid_size_y_) *
                  map_grid_size_y_ +
                origin_y_remainder_;

    map_grids_x_ = static_cast<int>(
      std::ceil((current_position_.value().x + map_loader_radius_ - origin_x_) / map_grid_size_x_));
    map_grids_y_ = static_cast<int>(
      std::ceil((current_position_.value().y + map_loader_radius_ - origin_y_) / map_grid_size_y_));

    if (map_grids_x_ * map_grids_y_ == 0) {
      return;
    }

    (*mutex_ptr_).lock();
    current_voxel_grid_array_.assign(
      map_grids_x_ * map_grid_size_y_, std::make_shared<MapGridVoxelInfo>());
    for (const auto & kv : current_voxel_grid_dict_) {
      int index = static_cast<int>(
        std::floor((kv.second.min_b_x - origin_x_) / map_grid_size_x_) +
        map_grids_x_ * std::floor((kv.second.min_b_y - origin_y_) / map_grid_size_y_));
      // TODO(1222-takeshi): check if index is valid
      if (index >= map_grids_x_ * map_grids_y_ || index < 0) {
        continue;
      }
      current_voxel_grid_array_.at(index) = std::make_shared<MapGridVoxelInfo>(kv.second);
    }
    (*mutex_ptr_).unlock();
  }

  inline void removeMapCell(const std::string map_cell_id_to_remove)
  {
    (*mutex_ptr_).lock();
    current_voxel_grid_dict_.erase(map_cell_id_to_remove);
    (*mutex_ptr_).unlock();
  }

  virtual inline void addMapCellAndFilter(
    const autoware_map_msgs::msg::PointCloudMapCellWithID & map_cell_to_add)
  {
    map_grid_size_x_ = map_cell_to_add.metadata.max_x - map_cell_to_add.metadata.min_x;
    map_grid_size_y_ = map_cell_to_add.metadata.max_y - map_cell_to_add.metadata.min_y;

    origin_x_remainder_ = std::remainder(map_cell_to_add.metadata.min_x, map_grid_size_x_);
    origin_y_remainder_ = std::remainder(map_cell_to_add.metadata.min_y, map_grid_size_y_);

    pcl::PointCloud<pcl::PointXYZ> map_cell_pc_tmp;
    pcl::fromROSMsg(map_cell_to_add.pointcloud, map_cell_pc_tmp);

    VoxelGridPointXYZ map_cell_voxel_grid_tmp;
    PointCloudPtr map_cell_downsampled_pc_ptr_tmp;

    auto map_cell_voxel_input_tmp_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_cell_pc_tmp);
    map_cell_voxel_grid_tmp.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_z_);
    map_cell_downsampled_pc_ptr_tmp.reset(new pcl::PointCloud<pcl::PointXYZ>);
    map_cell_voxel_grid_tmp.setInputCloud(map_cell_voxel_input_tmp_ptr);
    map_cell_voxel_grid_tmp.setSaveLeafLayout(true);
    map_cell_voxel_grid_tmp.filter(*map_cell_downsampled_pc_ptr_tmp);

    MapGridVoxelInfo current_voxel_grid_list_item;
    current_voxel_grid_list_item.min_b_x = map_cell_to_add.metadata.min_x;
    current_voxel_grid_list_item.min_b_y = map_cell_to_add.metadata.min_y;
    current_voxel_grid_list_item.max_b_x = map_cell_to_add.metadata.max_x;
    current_voxel_grid_list_item.max_b_y = map_cell_to_add.metadata.max_y;

    current_voxel_grid_list_item.map_cell_voxel_grid.set_voxel_grid(
      &(map_cell_voxel_grid_tmp.leaf_layout_), map_cell_voxel_grid_tmp.get_min_b(),
      map_cell_voxel_grid_tmp.get_max_b(), map_cell_voxel_grid_tmp.get_div_b(),
      map_cell_voxel_grid_tmp.get_divb_mul(), map_cell_voxel_grid_tmp.get_inverse_leaf_size());

    current_voxel_grid_list_item.map_cell_pc_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_voxel_grid_list_item.map_cell_pc_ptr = std::move(map_cell_downsampled_pc_ptr_tmp);
    // add
    (*mutex_ptr_).lock();
    current_voxel_grid_dict_.insert({map_cell_to_add.cell_id, current_voxel_grid_list_item});
    (*mutex_ptr_).unlock();
  }
};

#endif  // COMPARE_MAP_SEGMENTATION__VOXEL_GRID_MAP_LOADER_HPP_
