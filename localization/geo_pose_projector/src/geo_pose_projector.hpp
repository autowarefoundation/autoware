// Copyright 2023 Autoware Foundation
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

#ifndef GEO_POSE_PROJECTOR_HPP_
#define GEO_POSE_PROJECTOR_HPP_

#include <component_interface_specs/map.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geographic_msgs/msg/geo_pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <optional>
#include <string>

class GeoPoseProjector : public rclcpp::Node
{
private:
  using GeoPoseWithCovariance = geographic_msgs::msg::GeoPoseWithCovarianceStamped;
  using PoseWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;
  using MapProjectorInfo = map_interface::MapProjectorInfo;

public:
  GeoPoseProjector();

private:
  void on_geo_pose(const GeoPoseWithCovariance::ConstSharedPtr msg);

  component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr sub_map_projector_info_;
  rclcpp::Subscription<GeoPoseWithCovariance>::SharedPtr geo_pose_sub_;
  rclcpp::Publisher<PoseWithCovariance>::SharedPtr pose_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<MapProjectorInfo::Message> projector_info_ = std::nullopt;

  const bool publish_tf_;

  std::string parent_frame_;
  std::string child_frame_;
};

#endif  // GEO_POSE_PROJECTOR_HPP_
