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

#ifndef YABLOC_POSE_INITIALIZER__CAMERA__CAMERA_POSE_INITIALIZER_HPP_
#define YABLOC_POSE_INITIALIZER__CAMERA__CAMERA_POSE_INITIALIZER_HPP_

#include "yabloc_pose_initializer/camera/lane_image.hpp"
#include "yabloc_pose_initializer/camera/marker_module.hpp"
#include "yabloc_pose_initializer/camera/projector_module.hpp"
#include "yabloc_pose_initializer/camera/semantic_segmentation.hpp"

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>

#include <memory>

namespace yabloc
{
class CameraPoseInitializer : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using RequestPoseAlignment = tier4_localization_msgs::srv::PoseWithCovarianceStamped;

  CameraPoseInitializer();

private:
  const int angle_resolution_;
  std::unique_ptr<LaneImage> lane_image_{nullptr};
  std::unique_ptr<initializer::MarkerModule> marker_module_{nullptr};
  std::unique_ptr<initializer::ProjectorModule> projector_module_{nullptr};

  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;

  rclcpp::Service<RequestPoseAlignment>::SharedPtr align_server_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  std::optional<Image::ConstSharedPtr> latest_image_msg_{std::nullopt};
  lanelet::ConstLanelets const_lanelets_;

  std::unique_ptr<SemanticSegmentation> semantic_segmentation_{nullptr};

  void on_map(const HADMapBin & msg);
  void on_service(
    const RequestPoseAlignment::Request::SharedPtr,
    RequestPoseAlignment::Response::SharedPtr request);
  PoseCovStamped create_rectified_initial_pose(
    const Eigen::Vector3f & pos, double yaw_angle_rad, const PoseCovStamped & src_msg);

  std::optional<double> estimate_pose(
    const Eigen::Vector3f & position, const double yaw_angle_rad, const double yaw_std_rad);
};
}  // namespace yabloc

#endif  // YABLOC_POSE_INITIALIZER__CAMERA__CAMERA_POSE_INITIALIZER_HPP_
