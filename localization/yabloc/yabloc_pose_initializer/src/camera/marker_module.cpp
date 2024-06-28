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

#include "yabloc_pose_initializer/camera/marker_module.hpp"

#include <yabloc_common/color.hpp>

namespace yabloc::initializer
{

MarkerModule::MarkerModule(rclcpp::Node * node)
{
  pub_marker_ = node->create_publisher<MarkerArray>("~/debug/init_candidates", 10);
}

void MarkerModule::publish_marker(
  const std::vector<float> & scores, const std::vector<float> & angles,
  const Eigen::Vector3f & position)
{
  const int n = static_cast<int>(scores.size());
  auto minmax = std::minmax_element(scores.begin(), scores.end());
  auto normalize = [minmax](int score) -> float {
    return static_cast<float>(static_cast<float>(score) - *minmax.first) /
           std::max(1e-4f, static_cast<float>(*minmax.second - *minmax.first));
  };

  MarkerArray array;
  for (int i = 0; i < n; i++) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.type = Marker::ARROW;
    marker.id = i;
    marker.ns = "arrow";
    marker.color = static_cast<std_msgs::msg::ColorRGBA>(
      common::color_scale::rainbow(normalize(static_cast<int>(scores.at(i)))));
    marker.color.a = 0.5;

    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();

    const float rad = angles.at(i);
    marker.pose.orientation.w = std::cos(rad / 2.f);
    marker.pose.orientation.z = std::sin(rad / 2.f);

    marker.scale.x = 2.0;  // arrow length
    marker.scale.y = 0.2;  // arrow width
    marker.scale.z = 0.3;  // arrow height

    array.markers.push_back(marker);
  }
  pub_marker_->publish(array);
}

}  // namespace yabloc::initializer
