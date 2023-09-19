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

#ifndef YABLOC_POSE_INITIALIZER__CAMERA__PROJECTOR_MODULE_HPP_
#define YABLOC_POSE_INITIALIZER__CAMERA__PROJECTOR_MODULE_HPP_

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yabloc_common/camera_info_subscriber.hpp>
#include <yabloc_common/static_tf_subscriber.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace yabloc::initializer
{
class ProjectorModule
{
public:
  using ProjectFunc = std::function<std::optional<Eigen::Vector3f>(const cv::Point2i &)>;
  explicit ProjectorModule(rclcpp::Node * node);

  bool define_project_func();

  cv::Mat project_image(const cv::Mat & mask_image);

private:
  common::CameraInfoSubscriber info_;
  common::StaticTfSubscriber tf_subscriber_;
  rclcpp::Logger logger_;
  ProjectFunc project_func_ = nullptr;
};
}  // namespace yabloc::initializer

#endif  // YABLOC_POSE_INITIALIZER__CAMERA__PROJECTOR_MODULE_HPP_
