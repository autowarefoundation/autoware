// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__MANAGED_TRANSFORM_BUFFER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__MANAGED_TRANSFORM_BUFFER_HPP_

#include "autoware/universe_utils/ros/transform_listener.hpp"

#include <eigen3/Eigen/Core>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace std
{
template <>
struct hash<std::pair<std::string, std::string>>
{
  size_t operator()(const std::pair<std::string, std::string> & p) const
  {
    size_t h1 = std::hash<std::string>{}(p.first);
    size_t h2 = std::hash<std::string>{}(p.second);
    return h1 ^ (h2 << 1u);
  }
};
}  // namespace std

namespace autoware::universe_utils
{
using std::chrono_literals::operator""ms;
using Key = std::pair<std::string, std::string>;
struct PairEqual
{
  bool operator()(const Key & p1, const Key & p2) const
  {
    return p1.first == p2.first && p1.second == p2.second;
  }
};
using TFMap = std::unordered_map<Key, Eigen::Matrix4f, std::hash<Key>, PairEqual>;
constexpr std::array<std::string_view, 3> warn_frames = {"map", "odom", "world"};

class ManagedTransformBuffer
{
public:
  explicit ManagedTransformBuffer(rclcpp::Node * node, const bool & has_static_tf_only)
  : node_(node)
  {
    if (has_static_tf_only) {
      get_transform_ = [this](
                         const std::string & target_frame, const std::string & source_frame,
                         Eigen::Matrix4f & eigen_transform) {
        return getStaticTransform(target_frame, source_frame, eigen_transform);
      };
    } else {
      tf_listener_ = std::make_unique<autoware::universe_utils::TransformListener>(node);
      get_transform_ = [this](
                         const std::string & target_frame, const std::string & source_frame,
                         Eigen::Matrix4f & eigen_transform) {
        return getDynamicTransform(target_frame, source_frame, eigen_transform);
      };
    }
  }

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    Eigen::Matrix4f & eigen_transform)
  {
    return get_transform_(target_frame, source_frame, eigen_transform);
  }

  /**
   * Transforms a point cloud from one frame to another.
   *
   * @param target_frame The target frame to transform the point cloud to.
   * @param cloud_in The input point cloud to be transformed.
   * @param cloud_out The transformed point cloud.
   * @return True if the transformation is successful, false otherwise.
   */
  bool transformPointcloud(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
    sensor_msgs::msg::PointCloud2 & cloud_out)
  {
    if (
      pcl::getFieldIndex(cloud_in, "x") == -1 || pcl::getFieldIndex(cloud_in, "y") == -1 ||
      pcl::getFieldIndex(cloud_in, "z") == -1) {
      RCLCPP_ERROR(node_->get_logger(), "Input pointcloud does not have xyz fields");
      return false;
    }
    if (target_frame == cloud_in.header.frame_id) {
      cloud_out = cloud_in;
      return true;
    }
    Eigen::Matrix4f eigen_transform;
    if (!getTransform(target_frame, cloud_in.header.frame_id, eigen_transform)) {
      return false;
    }
    pcl_ros::transformPointCloud(eigen_transform, cloud_in, cloud_out);
    cloud_out.header.frame_id = target_frame;
    return true;
  }

private:
  /**
   * @brief Retrieves a transform between two static frames.
   *
   * This function attempts to retrieve a transform between the target frame and the source frame.
   * If success, the transform matrix is set to the output parameter and the function returns true.
   * Otherwise, transform matrix is set to identity and the function returns false. Transform
   * Listener is destroyed after function call.
   *
   * @param target_frame The target frame.
   * @param source_frame The source frame.
   * @param eigen_transform The output Eigen transform matrix. It is set to the identity if the
   * transform is not found.
   * @return True if the transform was successfully retrieved, false otherwise.
   */
  bool getStaticTransform(
    const std::string & target_frame, const std::string & source_frame,
    Eigen::Matrix4f & eigen_transform)
  {
    if (
      std::find(warn_frames.begin(), warn_frames.end(), target_frame) != warn_frames.end() ||
      std::find(warn_frames.begin(), warn_frames.end(), source_frame) != warn_frames.end()) {
      RCLCPP_WARN(
        node_->get_logger(), "Using %s -> %s transform. This may not be a static transform.",
        target_frame.c_str(), source_frame.c_str());
    }

    auto key = std::make_pair(target_frame, source_frame);
    auto key_inv = std::make_pair(source_frame, target_frame);

    // Check if the transform is already in the buffer
    auto it = buffer_.find(key);
    if (it != buffer_.end()) {
      eigen_transform = it->second;
      return true;
    }

    // Check if the inverse transform is already in the buffer
    auto it_inv = buffer_.find(key_inv);
    if (it_inv != buffer_.end()) {
      eigen_transform = it_inv->second.inverse();
      buffer_[key] = eigen_transform;
      return true;
    }

    // Check if transform is needed
    if (target_frame == source_frame) {
      eigen_transform = Eigen::Matrix4f::Identity();
      buffer_[key] = eigen_transform;
      return true;
    }

    // Get the transform from the TF tree
    tf_listener_ = std::make_unique<autoware::universe_utils::TransformListener>(node_);
    auto tf = tf_listener_->getTransform(
      target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(1000ms));
    tf_listener_.reset();
    RCLCPP_DEBUG(
      node_->get_logger(), "Trying to enqueue %s -> %s transform to static TFs buffer...",
      target_frame.c_str(), source_frame.c_str());
    if (tf == nullptr) {
      eigen_transform = Eigen::Matrix4f::Identity();
      return false;
    }
    pcl_ros::transformAsMatrix(*tf, eigen_transform);
    buffer_[key] = eigen_transform;
    return true;
  }

  /** @brief Retrieves a transform between two dynamic frames.
   *
   * This function attempts to retrieve a transform between the target frame and the source frame.
   * If successful, the transformation matrix is assigned to the output parameter, and the function
   * returns true. Otherwise, the transformation matrix is set to the identity and the function
   * returns false.
   *
   * @param target_frame The target frame.
   * @param source_frame The source frame.
   * @param eigen_transform The output Eigen transformation matrix. It is set to the identity if the
   * transform is not found.
   * @return True if the transform was successfully retrieved, false otherwise.
   */
  bool getDynamicTransform(
    const std::string & target_frame, const std::string & source_frame,
    Eigen::Matrix4f & eigen_transform)
  {
    auto tf = tf_listener_->getTransform(
      target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(1000ms));
    if (tf == nullptr) {
      eigen_transform = Eigen::Matrix4f::Identity();
      return false;
    }
    pcl_ros::transformAsMatrix(*tf, eigen_transform);
    return true;
  }

  TFMap buffer_;
  rclcpp::Node * const node_;
  std::unique_ptr<autoware::universe_utils::TransformListener> tf_listener_;
  std::function<bool(const std::string &, const std::string &, Eigen::Matrix4f &)> get_transform_;
};

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__MANAGED_TRANSFORM_BUFFER_HPP_
