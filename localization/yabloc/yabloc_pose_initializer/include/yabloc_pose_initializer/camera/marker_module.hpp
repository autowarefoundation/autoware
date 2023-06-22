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

#ifndef YABLOC_POSE_INITIALIZER__CAMERA__MARKER_MODULE_HPP_
#define YABLOC_POSE_INITIALIZER__CAMERA__MARKER_MODULE_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace yabloc::initializer
{
class MarkerModule
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  explicit MarkerModule(rclcpp::Node * node);

  void publish_marker(
    const std::vector<float> & scores, const std::vector<float> & angles,
    const Eigen::Vector3f & position);

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
};
}  // namespace yabloc::initializer

#endif  // YABLOC_POSE_INITIALIZER__CAMERA__MARKER_MODULE_HPP_
