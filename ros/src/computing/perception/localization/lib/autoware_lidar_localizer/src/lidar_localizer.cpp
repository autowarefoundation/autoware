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

#include "autoware_lidar_localizer/lidar_localizer.h"

#include <iostream>
#include <thread>

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::LidarLocalizer()
    : is_init_map_(false), map_point_size_(0), align_time_(0),
      fitness_score_(0), thread_status_(ThreadStatus::Sleeping) {}

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::~LidarLocalizer() {}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::initPointsMap(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &pointcloud_ptr) {
  setInputTarget(pointcloud_ptr);
  map_point_size_ = pointcloud_ptr->points.size();
  is_init_map_ = true;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updatePointsMap(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &pointcloud_ptr) {
  if (pointcloud_ptr == nullptr) {
    std::cout << "[ERROR] pointcloud_ptr is nullptr" << std::endl;
    return;
  }

  if (map_point_size_ == 0) {
    initPointsMap(pointcloud_ptr);
  }
  else if (map_point_size_ != pointcloud_ptr->points.size()) {
    buildMapThread(pointcloud_ptr);
  }
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::buildMapThread(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {
  // donot create multiple threads
  if (thread_status_ != ThreadStatus::Sleeping) {
    return;
  }

  thread_begin_time_ = std::chrono::system_clock::now();
  thread_status_ = ThreadStatus::Running;

  std::thread build_map_thread([this, map_ptr]() {
    buildMap(map_ptr);
    thread_status_ = ThreadStatus::Finished;
  });
  build_map_thread.detach();

  map_point_size_ = map_ptr->points.size();
}

template <class PointSource, class PointTarget>
bool LidarLocalizer<PointSource, PointTarget>::swapMap() {
  // if it takes a lot of time to generate map, wait for thread
  auto thread_now_time = std::chrono::system_clock::now();
  auto thread_processing_time_msec =
      std::chrono::duration_cast<std::chrono::microseconds>(thread_now_time - thread_begin_time_).count() /1000.0;
  const double time_threshold_msec = 1000.0;
  if (thread_status_ == ThreadStatus::Running &&
      thread_processing_time_msec > time_threshold_msec) {
    std::cout << "*************************" << std::endl;
    std::cout << "waiting for finish thread" << std::endl;
    std::cout << "*************************" << std::endl;
    while (thread_status_ != ThreadStatus::Finished) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  else if (thread_status_ != ThreadStatus::Finished) {
    return false;
  }

  swapInstance();
  thread_status_ = ThreadStatus::Sleeping;
  return true;
}

template <class PointSource, class PointTarget>
bool LidarLocalizer<PointSource, PointTarget>::alignMap(
    const boost::shared_ptr<pcl::PointCloud<PointSource>> &pointcloud_ptr,
    const Pose &predict_pose) {
  if (!is_init_map_) {
    std::cout << "[WARN]received points. But map is not loaded" << std::endl;
    return false;
  }

  swapMap();
  setInputSource(pointcloud_ptr);

  const auto align_start = std::chrono::system_clock::now();
  align(predict_pose);
  const auto align_end = std::chrono::system_clock::now();
  align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

  localizer_pose_ = getFinalPose();

  return true;
}

template <class PointSource, class PointTarget>
Pose LidarLocalizer<PointSource, PointTarget>::getLocalizerPose() const {
  return localizer_pose_;
}

template <class PointSource, class PointTarget>
double LidarLocalizer<PointSource, PointTarget>::getAlignTime() const {
  return align_time_;
}

template class LidarLocalizer<pcl::PointXYZ, pcl::PointXYZ>;
template class LidarLocalizer<pcl::PointXYZI, pcl::PointXYZI>;
