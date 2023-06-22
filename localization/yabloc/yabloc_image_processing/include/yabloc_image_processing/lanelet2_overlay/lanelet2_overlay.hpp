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

#ifndef YABLOC_IMAGE_PROCESSING__LANELET2_OVERLAY__LANELET2_OVERLAY_HPP_
#define YABLOC_IMAGE_PROCESSING__LANELET2_OVERLAY__LANELET2_OVERLAY_HPP_

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yabloc_common/ground_plane.hpp>
#include <yabloc_common/static_tf_subscriber.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace yabloc::lanelet2_overlay
{
class Lanelet2Overlay : public rclcpp::Node
{
public:
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Marker = visualization_msgs::msg::Marker;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using LineSegments = pcl::PointCloud<pcl::PointNormal>;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;
  using Float32Array = std_msgs::msg::Float32MultiArray;

  Lanelet2Overlay();

private:
  common::StaticTfSubscriber tf_subscriber_;
  common::GroundPlane ground_plane_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_line_segments_cloud_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_sign_board_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_ground_plane_;

  rclcpp::Publisher<Marker>::SharedPtr pub_vis_;

  std::optional<CameraInfo> info_{std::nullopt};
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};
  LineSegments ll2_cloud_, sign_board_;
  boost::circular_buffer<PoseStamped> pose_buffer_;

  void on_info(const CameraInfo & msg);
  void on_image(const Image & msg);
  void on_line_segments(const PointCloud2 & msg);

  void draw_overlay(
    const cv::Mat & image, const std::optional<Pose> & pose, const rclcpp::Time & stamp);
  void draw_overlay_line_segments(
    cv::Mat & image, const Pose & pose, const LineSegments & line_segments);

  void make_vis_marker(const LineSegments & ls, const Pose & pose, const rclcpp::Time & stamp);
};
}  // namespace yabloc::lanelet2_overlay

#endif  // YABLOC_IMAGE_PROCESSING__LANELET2_OVERLAY__LANELET2_OVERLAY_HPP_
