/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lidar_localizer/map_manager/map_manager.h"

#include <boost/filesystem.hpp>

template <class PointTarget>
MapManager<PointTarget>::MapManager()
    : separate_map_size_(100.0), save_map_leaf_size_(0.2),
      default_reserve_size_(10000000), is_thread_run_ok_(true) {
  map_.points.reserve(default_reserve_size_);
  runProcess();
}

template <class PointTarget> MapManager<PointTarget>::~MapManager() {
  is_thread_run_ok_ = false;
  process_thread_.join();
}

template <class PointTarget> void MapManager<PointTarget>::runProcess() {
  process_thread_ = std::thread([this]() {
    while (is_thread_run_ok_) {
      if (process_queue_.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      mtx_.lock();
      auto process = process_queue_.front();
      process_queue_.pop_front();
      mtx_.unlock();

      process();
    }
  });
}

template <class PointTarget>
void MapManager<PointTarget>::addPointCloudMapThread(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_raw_ptr) {
  auto process = std::bind(&MapManager<PointTarget>::addPointCloudMap, this,
                           points_raw_ptr);
  std::lock_guard<std::mutex> lock(mtx_);
  process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::downsampleMapThread() {
  auto process = std::bind(&MapManager<PointTarget>::downsampleMap, this);
  std::lock_guard<std::mutex> lock(mtx_);
  process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::saveSingleMapThread() {
  auto process = std::bind(&MapManager<PointTarget>::saveSingleMap, this);
  std::lock_guard<std::mutex> lock(mtx_);
  process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::saveSeparateMapThread() {
  auto process = std::bind(&MapManager<PointTarget>::saveSeparateMap, this);
  std::lock_guard<std::mutex> lock(mtx_);
  process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::loadAroundMapThread(const Pose &localizer_pose) {
  auto process =
      std::bind(&MapManager<PointTarget>::loadAroundMap, this, localizer_pose);
  std::lock_guard<std::mutex> lock(mtx_);
  process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::addPointCloudMap(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_raw_ptr) {

  const auto need_points_size =
      map_.points.size() + points_raw_ptr->points.size();
  if(map_.points.capacity() < need_points_size) {
    map_.points.reserve(need_points_size*2);
  }

  map_ += *points_raw_ptr;

  mtx_.lock();
  map_ptr_ = map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget> void MapManager<PointTarget>::downsampleMap() {
  if (map_ptr_ == nullptr) {
    std::cout << __func__ << "[ERROR] map_ptr_ is nullptr" << std::endl;
    return;
  }

  pcl::PointCloud<PointTarget> filterd_map;
  pcl::VoxelGrid<PointTarget> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(save_map_leaf_size_, save_map_leaf_size_,
                                save_map_leaf_size_);
  voxel_grid_filter.setInputCloud(map_ptr_);
  voxel_grid_filter.filter(map_);

  if (map_.points.capacity() < default_reserve_size_) {
    map_.points.reserve(default_reserve_size_);
  }

  mtx_.lock();
  map_ptr_ = map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget> void MapManager<PointTarget>::saveSingleMap() {
  if (map_ptr_ == nullptr) {
    std::cout << __func__ << "[ERROR] map_ptr_ is nullptr" << std::endl;
    return;
  }

  std::string path = directory_path_ + "/pointcloud_map.pcd";
  pcl::io::savePCDFileBinary(path, *map_ptr_);
}

template <class PointTarget> void MapManager<PointTarget>::saveSeparateMap() {
  if (map_ptr_ == nullptr) {
    std::cout << __func__ << "[ERROR] map_ptr_ is nullptr" << std::endl;
    return;
  }

  double min_x_m = calcMinX(map_ptr_);
  double max_x_m = calcMaxX(map_ptr_);
  double min_y_m = calcMinY(map_ptr_);
  double max_y_m = calcMaxY(map_ptr_);

  const int min_x = std::floor(min_x_m / separate_map_size_);
  const int max_x = std::floor(max_x_m / separate_map_size_);
  const int min_y = std::floor(min_y_m / separate_map_size_);
  const int max_y = std::floor(max_y_m / separate_map_size_);

  for (int i = min_x; i <= max_x; ++i) {
    for (int j = min_y; j <= max_y; ++j) {
      boost::shared_ptr<pcl::PointCloud<PointTarget>> map_region_tmp_ptr(
          new pcl::PointCloud<PointTarget>);
      passThroughPointCloud<PointTarget>(
          map_ptr_, map_region_tmp_ptr, i * separate_map_size_,
          j * separate_map_size_, separate_map_size_);

      if (map_region_tmp_ptr->width == 0) {
        continue;
      }

      std::string path = directory_path_ + "/pointcloud_map_" +
                         std::to_string(i * separate_map_size_) + "_" +
                         std::to_string(j * separate_map_size_) + ".pcd";
      boost::shared_ptr<pcl::PointCloud<PointTarget>> load_cloud(
          new pcl::PointCloud<PointTarget>);

      boost::system::error_code file_error;
      const bool exist_file =
          boost::filesystem::exists(path.c_str(), file_error);
      if (!exist_file || file_error) {
        std::cout << "no exist " << path << std::endl;
      } else if (pcl::io::loadPCDFile(path.c_str(), *load_cloud) == -1) {
        std::cout << "load failed " << path << std::endl;
      }

      map_region_tmp_ptr->height = 1;
      map_region_tmp_ptr->width += load_cloud->width;

      if (map_region_tmp_ptr->points.capacity() < map_region_tmp_ptr->width) {
        map_region_tmp_ptr->points.reserve(map_region_tmp_ptr->width);
      }

      for (const auto &point : load_cloud->points) {
        map_region_tmp_ptr->points.push_back(point);
      }

      pcl::io::savePCDFileBinary(path, *map_region_tmp_ptr);
    }
  }
}

template <class PointTarget>
void MapManager<PointTarget>::loadAroundMap(const Pose &localizer_pose) {

  std::vector<std::string> path_arrary;

  const int x = std::floor(localizer_pose.x / separate_map_size_);
  const int y = std::floor(localizer_pose.y / separate_map_size_);

  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      std::string path = directory_path_ + "/pointcloud_map_" +
                         std::to_string((i + x) * separate_map_size_) + "_" +
                         std::to_string((j + y) * separate_map_size_) + ".pcd";
      path_arrary.push_back(path);
    }
  }

  std::map<std::string, boost::shared_ptr<pcl::PointCloud<PointTarget>>>
      tmp_map_ptr;
  for (const auto &path : path_arrary) {
    pcl::PointCloud<PointTarget> pointcloud;

    boost::system::error_code file_error;
    const bool exist_file = boost::filesystem::exists(path.c_str(), file_error);
    if (!exist_file || file_error) {
      std::cout << "no exist " << path << std::endl;
      continue;
    }
    if (pcl::io::loadPCDFile(path.c_str(), pointcloud) == -1) {
      std::cout << "load failed " << path << std::endl;
      continue;
    }
    tmp_map_ptr.emplace(path, pointcloud.makeShared());
  }

  map_.points.clear();

  for (const auto &pointcloud_map : tmp_map_ptr) {
    const auto need_points_size =
        map_.points.size() + pointcloud_map.second->points.size();

    if(map_.points.capacity() < need_points_size) {
      map_.points.reserve(need_points_size*2);
    }
    map_ += *(pointcloud_map.second);
  }

  mtx_.lock();
  map_ptr_ = map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget>
void MapManager<PointTarget>::setMap(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {
  mtx_.lock();
  map_ = *map_ptr;
  if (map_.points.capacity() < default_reserve_size_) {
    map_.points.reserve(default_reserve_size_);
  }
  map_ptr_ = map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget>
boost::shared_ptr<pcl::PointCloud<PointTarget>>
MapManager<PointTarget>::getMap() {
  return map_ptr_;
}

template <class PointTarget>
void MapManager<PointTarget>::setSaveSeparateMapSize(
    const double separate_map_size) {
  separate_map_size_ = separate_map_size;
}

template <class PointTarget>
double MapManager<PointTarget>::getSaveSeparateMapSize() const {
  return separate_map_size_;
}

template <class PointTarget>
void MapManager<PointTarget>::setSaveMapLeafSize(
    const double save_map_leaf_size) {
  save_map_leaf_size_ = save_map_leaf_size;
}

template <class PointTarget>
void MapManager<PointTarget>::setFileDirectoryPath(
    const std::string &directory_path) {
  directory_path_ = directory_path;
}

template class MapManager<pcl::PointXYZ>;
template class MapManager<pcl::PointXYZI>;
