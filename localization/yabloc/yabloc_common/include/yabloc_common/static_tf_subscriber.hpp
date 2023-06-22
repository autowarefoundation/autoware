// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_COMMON__STATIC_TF_SUBSCRIBER_HPP_
#define YABLOC_COMMON__STATIC_TF_SUBSCRIBER_HPP_

#include <Eigen/Geometry>
#include <sophus/geometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>

namespace yabloc::common
{
class StaticTfSubscriber
{
public:
  explicit StaticTfSubscriber(rclcpp::Clock::SharedPtr clock);

  std::optional<Sophus::SE3f> se3f(
    const std::string & frame_id, const std::string & parent_frame_id = "base_link");

  std::optional<Eigen::Affine3f> operator()(
    const std::string & frame_id, const std::string & parent_frame_id = "base_link");

private:
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace yabloc::common

#endif  // YABLOC_COMMON__STATIC_TF_SUBSCRIBER_HPP_
