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

#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "autoware_lidar_localizer/util/data_structs.h"
#include "autoware_lidar_localizer/util/util_functions.h"

template <class PointTarget> class MapManager {
public:
  MapManager();
  virtual ~MapManager();

  void addPointCloudMapThread(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_raw_ptr);
  void downsampleMapThread();
  void saveSingleMapThread();
  void saveSeparateMapThread();
  void loadAroundMapThread(const Pose &localizer_pose);

  void addPointCloudMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &points_raw_ptr);
  void downsampleMap();
  void saveSingleMap();
  void saveSingleMapImpl(const std::string &directory_path, const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr);
  void saveSeparateMap();
  void saveSeparateMapImpl(const std::string &directory_path, const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr);
  void loadAroundMap(const Pose &localizer_pose);
  void loadMap(const std::string &path);

  void setMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr);
  boost::shared_ptr<pcl::PointCloud<PointTarget>> getMapPtr() const;
  void setSaveSeparateMapSize(const double separate_map_size);
  double getSaveSeparateMapSize() const;
  void setSaveMapLeafSize(const double save_map_leaf_size);
  double getSaveMapLeafSize() const;
  void setSaveAddedMap(const bool save_added_map);
  bool getSaveAddedMap() const;
  void setFileDirectoryPath(const std::string &directory_path);

private:
  void runProcess();

  std::string directory_path_;
  double separate_map_size_;
  double save_map_leaf_size_;
  double default_reserve_size_;

  pcl::PointCloud<PointTarget> merged_map_;
  pcl::PointCloud<PointTarget> added_map_;
  boost::shared_ptr<pcl::PointCloud<PointTarget>> merged_map_ptr_;

  std::deque<std::function<void()>> process_queue_;
  std::thread process_thread_;
  std::mutex mtx_;
  bool is_thread_run_ok_;
  bool save_added_map_;
};

#endif
