// Copyright 2021 Tier IV, Inc.
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

#include <pcl_ros/transforms.hpp>
#include <pointcloud_densification.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <string>
#include <utility>

namespace centerpoint
{
PointCloudDensification::PointCloudDensification(
  std::string base_frame_id, const unsigned int pointcloud_cache_size,
  rclcpp::Clock::SharedPtr clock)
: base_frame_id_(std::move(base_frame_id)),
  pointcloud_cache_size_(pointcloud_cache_size),
  tf_buffer_(clock)
{
}

sensor_msgs::msg::PointCloud2 PointCloudDensification::stackPointCloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg)
{
  const double input_timestamp = rclcpp::Time(input_pointcloud_msg.header.stamp).seconds();

  auto transform_base2frame =
    getTransformStamped(input_pointcloud_msg.header.frame_id, base_frame_id_);
  Eigen::Matrix4f matrix_base2current =
    tf2::transformToEigen(transform_base2frame.transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 output_pointcloud_msg;
  output_pointcloud_msg = input_pointcloud_msg;
  setTimeLag(output_pointcloud_msg, 0);

  // concat the current frame and past frames
  auto pc_msg_iter = pointcloud_cache_.begin();
  auto matrix_iter = matrix_past2base_cache_.begin();
  for (; pc_msg_iter != pointcloud_cache_.end(); pc_msg_iter++, matrix_iter++) {
    sensor_msgs::msg::PointCloud2 cached_pointcloud_msg = *pc_msg_iter;
    sensor_msgs::msg::PointCloud2 transformed_pointcloud_msg;

    Eigen::Affine3f affine_past2base;
    affine_past2base.matrix() = *matrix_iter;
    Eigen::Affine3f affine_base2current;
    affine_base2current.matrix() = matrix_base2current;

    pcl_ros::transformPointCloud(
      (affine_base2current * affine_past2base).matrix(), cached_pointcloud_msg,
      transformed_pointcloud_msg);
    double diff_timestamp =
      input_timestamp - rclcpp::Time(cached_pointcloud_msg.header.stamp).seconds();
    setTimeLag(transformed_pointcloud_msg, static_cast<float>(diff_timestamp));

    sensor_msgs::msg::PointCloud2 tmp_pointcloud_msg = output_pointcloud_msg;
    pcl::concatenatePointCloud(
      tmp_pointcloud_msg, transformed_pointcloud_msg, output_pointcloud_msg);
  }

  // add input pointcloud to cache
  auto transform_frame2base =
    getTransformStamped(base_frame_id_, input_pointcloud_msg.header.frame_id);
  Eigen::Matrix4f matrix_past2base =
    tf2::transformToEigen(transform_frame2base.transform).matrix().cast<float>();
  pointcloud_cache_.push_front(input_pointcloud_msg);
  matrix_past2base_cache_.push_front(matrix_past2base);
  if (pointcloud_cache_.size() > pointcloud_cache_size_) {
    pointcloud_cache_.pop_back();
    matrix_past2base_cache_.pop_back();
  }

  return output_pointcloud_msg;
}

geometry_msgs::msg::TransformStamped PointCloudDensification::getTransformStamped(
  const std::string & target_frame, const std::string & source_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(target_frame, source_frame, rclcpp::Time(0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(rclcpp::get_logger("PointCloudDensification"), "%s", ex.what());
    return transform;
  }
  return transform;
}

void PointCloudDensification::setTimeLag(
  sensor_msgs::msg::PointCloud2 & pointcloud_msg, const float diff_timestamp)
{
  // Note: pcl::fromROSMsg is very slow, so point values in ros message are changed directly.
  // Note: intensity isn't used in this CenterPoint implementation.

  const int intensity_idx = pcl::getFieldIndex(pointcloud_msg, "intensity");
  if (intensity_idx == -1) {
    return;
  }

  const int width = pointcloud_msg.width;
  const int point_step = pointcloud_msg.point_step;
  const int offset = pointcloud_msg.fields[intensity_idx].offset;
  for (int i = 0; i < width; i++) {
    memcpy(
      &pointcloud_msg.data[offset + point_step * i],
      reinterpret_cast<const void *>(&diff_timestamp), sizeof(float));
  }
}

}  // namespace centerpoint
