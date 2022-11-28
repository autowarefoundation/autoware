// Copyright 2021-2022 AutoCore Ltd., TIER IV, Inc.
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

#ifndef LIDAR_CENTERPOINT_TVM__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
#define LIDAR_CENTERPOINT_TVM__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_

#include <lidar_centerpoint_tvm/visibility_control.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <list>
#include <string>
#include <utility>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{
class LIDAR_CENTERPOINT_TVM_LOCAL DensificationParam
{
public:
  DensificationParam(const std::string & world_frame_id, const uint32_t num_past_frames)
  : world_frame_id_(std::move(world_frame_id)),
    pointcloud_cache_size_(num_past_frames + /*current frame*/ 1)
  {
  }

  std::string world_frame_id() const { return world_frame_id_; }
  uint32_t pointcloud_cache_size() const { return pointcloud_cache_size_; }

private:
  std::string world_frame_id_;
  uint32_t pointcloud_cache_size_{1};
};

struct PointCloudWithTransform
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  Eigen::Affine3f affine_past2world;
};

class LIDAR_CENTERPOINT_TVM_LOCAL PointCloudDensification
{
public:
  explicit PointCloudDensification(const DensificationParam & param);

  bool enqueuePointCloud(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer);

  double getCurrentTimestamp() const { return current_timestamp_; }
  Eigen::Affine3f getAffineWorldToCurrent() const { return affine_world2current_; }
  std::list<PointCloudWithTransform>::iterator getPointCloudCacheIter()
  {
    return pointcloud_cache_.begin();
  }
  bool isCacheEnd(std::list<PointCloudWithTransform>::iterator iter)
  {
    return iter == pointcloud_cache_.end();
  }

private:
  void enqueue(const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Affine3f & affine);
  void dequeue();

  DensificationParam param_;
  double current_timestamp_{0.0};
  Eigen::Affine3f affine_world2current_;
  std::list<PointCloudWithTransform> pointcloud_cache_;
};

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_CENTERPOINT_TVM__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
