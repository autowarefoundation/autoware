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

#ifndef LIDAR_LOCALIZER_H
#define LIDAR_LOCALIZER_H

#include <chrono>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "autoware_lidar_localizer/util/data_structs.h"
#include "autoware_lidar_localizer/util/util_functions.h"

template <class PointSource, class PointTarget> class LidarLocalizer {
  enum class ThreadStatus {
    Sleeping,
    Running,
    Finished,
  };

public:
  LidarLocalizer();
  virtual ~LidarLocalizer();
  void initPointsMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &pointcloud_ptr);
  void updatePointsMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &pointcloud_ptr);

  bool alignMap(const boost::shared_ptr<pcl::PointCloud<PointSource>> &pointcloud_ptr, const Pose &predict_pose);

  Pose getLocalizerPose() const;
  double getAlignTime() const;

protected:
  virtual void align(const Pose &predict_pose) = 0;
  virtual void setInputTarget(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) = 0;
  virtual void setInputSource(const boost::shared_ptr<pcl::PointCloud<PointSource>> &scan_ptr) = 0;
  virtual Pose getFinalPose() = 0;

  virtual void buildMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) = 0;
  virtual void swapInstance() = 0;

private:
  void buildMapThread(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr);
  bool swapMap();

  Pose localizer_pose_;

  bool is_init_map_;
  size_t map_point_size_;
  double align_time_;
  double fitness_score_;

  ThreadStatus thread_status_;
  std::chrono::system_clock::time_point thread_begin_time_;
};

#endif
