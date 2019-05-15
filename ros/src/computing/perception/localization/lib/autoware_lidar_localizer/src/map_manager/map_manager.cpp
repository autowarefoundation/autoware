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

#include "autoware_lidar_localizer/map_manager/map_manager.h"

#include <boost/filesystem.hpp>

template <class PointTarget>
MapManager<PointTarget>::MapManager()
    : separate_map_size_(100.0), save_map_leaf_size_(0.2),
      default_reserve_size_(10000000), merged_map_ptr_(nullptr),
      is_thread_run_ok_(true), save_added_map_(false) {
  merged_map_.points.reserve(default_reserve_size_);
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
void MapManager<PointTarget>::addPointCloudMapThread(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_raw_ptr) {
  auto process = std::bind(&MapManager<PointTarget>::addPointCloudMap, this,points_raw_ptr);
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
  auto process = std::bind(&MapManager<PointTarget>::loadAroundMap, this, localizer_pose);
  std::lock_guard<std::mutex> lock(mtx_);
  process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::addPointCloudMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_raw_ptr) {

  if(save_added_map_) {
      const auto need_points_size = added_map_.points.size() + points_raw_ptr->points.size();
      if(added_map_.points.capacity() < need_points_size) {
        added_map_.points.reserve(need_points_size*2);
      }
      added_map_ += *points_raw_ptr;
  }

  const auto need_points_size = merged_map_.points.size() + points_raw_ptr->points.size();
  if(merged_map_.points.capacity() < need_points_size) {
    merged_map_.points.reserve(need_points_size*2);
  }
  merged_map_ += *points_raw_ptr;

  mtx_.lock();
  merged_map_ptr_ = merged_map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget> void MapManager<PointTarget>::downsampleMap() {

  boost::shared_ptr<pcl::PointCloud<PointTarget>> map_tmp_ptr(new pcl::PointCloud<PointTarget>);
  donwsamplePointCloud(merged_map_ptr_, merged_map_.makeShared(), save_map_leaf_size_);

  if (merged_map_.points.capacity() < default_reserve_size_) {
    merged_map_.points.reserve(default_reserve_size_);
  }

  mtx_.lock();
  merged_map_ptr_ = merged_map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget> void MapManager<PointTarget>::saveSingleMap() {

  if(save_added_map_) {
      boost::filesystem::create_directories(boost::filesystem::path(directory_path_+"/added"));
      saveSingleMapImpl(directory_path_+"/added", merged_map_.makeShared());
  }

  boost::filesystem::create_directories(boost::filesystem::path(directory_path_+"/merged"));
  saveSingleMapImpl(directory_path_+"/merged", merged_map_.makeShared());
}

template <class PointTarget> void MapManager<PointTarget>::saveSingleMapImpl(const std::string &directory_path, const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {

    if(map_ptr == nullptr || map_ptr->points.empty()) {
        std::cout << __func__ << "[ERROR] map_ptr is nullptr or empty" << std::endl;
        return;
    }

    boost::shared_ptr<pcl::PointCloud<PointTarget>> map_tmp_ptr(new pcl::PointCloud<PointTarget>);
    donwsamplePointCloud(map_ptr, map_tmp_ptr, save_map_leaf_size_);

    std::string path = directory_path + "/pointcloud_map.pcd";
    pcl::io::savePCDFileBinary(path, *map_tmp_ptr);
}

template <class PointTarget> void MapManager<PointTarget>::saveSeparateMap() {

    if(save_added_map_) {
        boost::filesystem::create_directories(boost::filesystem::path(directory_path_+"/added"));
        saveSeparateMapImpl(directory_path_+"/added", added_map_.makeShared());
        added_map_.points.clear();
    }

    boost::filesystem::create_directories(boost::filesystem::path(directory_path_+"/merged"));
    saveSeparateMapImpl(directory_path_+"/merged", merged_map_.makeShared());
}

template <class PointTarget> void MapManager<PointTarget>::saveSeparateMapImpl(const std::string &directory_path, const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {

  if(map_ptr == nullptr || map_ptr->points.empty()) {
    std::cout << __func__ << "[ERROR] map_ptr is nullptr or empty" << std::endl;
    return;
  }

  double min_x_m = calcMinX(map_ptr);
  double max_x_m = calcMaxX(map_ptr);
  double min_y_m = calcMinY(map_ptr);
  double max_y_m = calcMaxY(map_ptr);

  const int min_x = std::floor(min_x_m / separate_map_size_);
  const int max_x = std::floor(max_x_m / separate_map_size_);
  const int min_y = std::floor(min_y_m / separate_map_size_);
  const int max_y = std::floor(max_y_m / separate_map_size_);

  for (int i = min_x; i <= max_x; ++i) {
    for (int j = min_y; j <= max_y; ++j) {
      boost::shared_ptr<pcl::PointCloud<PointTarget>> map_region_tmp_ptr(new pcl::PointCloud<PointTarget>);
      passThroughPointCloud<PointTarget>(map_ptr, map_region_tmp_ptr, i * separate_map_size_,
                                         j * separate_map_size_, separate_map_size_);

      if (map_region_tmp_ptr->points.empty()) {
        continue;
      }

      std::string path = directory_path + "/pointcloud_map_" +
                         std::to_string(i * separate_map_size_) + "_" +
                         std::to_string(j * separate_map_size_) + ".pcd";
      boost::shared_ptr<pcl::PointCloud<PointTarget>> load_cloud_ptr(new pcl::PointCloud<PointTarget>);

      boost::system::error_code file_error;
      const bool exist_file = boost::filesystem::exists(path.c_str(), file_error);
      if (!exist_file || file_error) {
        std::cout << "no exist " << path << std::endl;
      }
      else if (pcl::io::loadPCDFile(path.c_str(), *load_cloud_ptr) == -1) {
        std::cout << "load failed " << path << std::endl;
      }

      if(!load_cloud_ptr->points.empty()) {
          const auto need_points_size = map_region_tmp_ptr->points.size() + load_cloud_ptr->points.size();
          if(map_region_tmp_ptr->points.capacity() < need_points_size) {
            map_region_tmp_ptr->points.reserve(need_points_size);
          }
          *map_region_tmp_ptr += *load_cloud_ptr;
      }

      boost::shared_ptr<pcl::PointCloud<PointTarget>> map_region_filtered_tmp_ptr(new pcl::PointCloud<PointTarget>);
      donwsamplePointCloud(map_region_tmp_ptr, map_region_filtered_tmp_ptr, save_map_leaf_size_);

      pcl::io::savePCDFileBinary(path, *map_region_filtered_tmp_ptr);
    }
  }
}

template <class PointTarget>
void MapManager<PointTarget>::loadAroundMap(const Pose &localizer_pose) {

  merged_map_.points.clear();

  const int x = std::floor(localizer_pose.x / separate_map_size_);
  const int y = std::floor(localizer_pose.y / separate_map_size_);

  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      std::string path = directory_path_ + "/merged/pointcloud_map_" +
                         std::to_string((i + x) * separate_map_size_) + "_" +
                         std::to_string((j + y) * separate_map_size_) + ".pcd";
      loadMap(path);
    }
  }

  mtx_.lock();
  merged_map_ptr_ = merged_map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget>
void MapManager<PointTarget>::loadMap(const std::string &path) {

  boost::system::error_code file_error;
  const bool exist_file = boost::filesystem::exists(path.c_str(), file_error);
  if (!exist_file || file_error) {
    std::cout << "no exist " << path << std::endl;
    return;
  }

  boost::shared_ptr<pcl::PointCloud<PointTarget>> load_cloud_ptr(new pcl::PointCloud<PointTarget>);
  if (pcl::io::loadPCDFile(path.c_str(), *load_cloud_ptr) == -1) {
    std::cout << "load failed " << path << std::endl;
    return;
  }

  const auto need_points_size = merged_map_.points.size() + load_cloud_ptr->points.size();
  if(merged_map_.points.capacity() < need_points_size) {
    merged_map_.points.reserve(need_points_size*2);
  }
  merged_map_+= *load_cloud_ptr;
}

template <class PointTarget>
void MapManager<PointTarget>::setMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {
  mtx_.lock();
  merged_map_= *map_ptr;
  if (merged_map_.points.capacity() < default_reserve_size_) {
    merged_map_.points.reserve(default_reserve_size_);
  }
  merged_map_ptr_ = merged_map_.makeShared();
  mtx_.unlock();
}

template <class PointTarget>
boost::shared_ptr<pcl::PointCloud<PointTarget>> MapManager<PointTarget>::getMapPtr() const{
  return merged_map_ptr_;
}

template <class PointTarget>
void MapManager<PointTarget>::setSaveSeparateMapSize(const double separate_map_size) {
  separate_map_size_ = separate_map_size;
}

template <class PointTarget>
double MapManager<PointTarget>::getSaveSeparateMapSize() const {
  return separate_map_size_;
}

template <class PointTarget>
void MapManager<PointTarget>::setSaveMapLeafSize(const double save_map_leaf_size) {
  save_map_leaf_size_ = save_map_leaf_size;
}

template <class PointTarget>
double MapManager<PointTarget>::getSaveMapLeafSize() const {
  return save_map_leaf_size_;
}

template <class PointTarget>
void MapManager<PointTarget>::setSaveAddedMap(const bool save_added_map) {
  save_added_map_ = save_added_map;
}

template <class PointTarget>
bool MapManager<PointTarget>::getSaveAddedMap() const {
  return save_added_map_;
}

template <class PointTarget>
void MapManager<PointTarget>::setFileDirectoryPath(const std::string &directory_path) {
  directory_path_ = directory_path;
}

template class MapManager<pcl::PointXYZ>;
template class MapManager<pcl::PointXYZI>;
