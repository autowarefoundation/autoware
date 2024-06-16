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

#include "yabloc_image_processing/lanelet2_overlay/lanelet2_overlay.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/extract_line_segments.hpp>
#include <yabloc_common/pose_conversions.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::lanelet2_overlay
{
Lanelet2Overlay::Lanelet2Overlay(const rclcpp::NodeOptions & options)
: Node("lanelet2_overlay", options), tf_subscriber_(get_clock()), pose_buffer_{40}
{
  using std::placeholders::_1;

  // Subscriber
  auto cb_info = std::bind(&Lanelet2Overlay::on_info, this, _1);
  auto cb_image = std::bind(&Lanelet2Overlay::on_image, this, _1);
  auto cb_line_segments = std::bind(&Lanelet2Overlay::on_line_segments, this, _1);
  auto cb_pose = [this](const PoseStamped & msg) -> void { pose_buffer_.push_back(msg); };
  auto cb_ground = [this](const Float32Array & msg) -> void { ground_plane_.set(msg); };

  sub_ground_plane_ = create_subscription<Float32Array>("~/input/ground", 10, cb_ground);
  sub_image_ = create_subscription<Image>("~/input/image_raw", 10, cb_image);
  sub_pose_ = create_subscription<PoseStamped>("~/input/pose", 10, cb_pose);
  sub_line_segments_cloud_ =
    create_subscription<PointCloud2>("~/input/projected_line_segments_cloud", 10, cb_line_segments);
  sub_info_ = create_subscription<CameraInfo>("~/input/camera_info", 10, cb_info);
  sub_sign_board_ = create_subscription<PointCloud2>(
    "~/input/ll2_sign_board", 10,
    [this](const PointCloud2 & msg) -> void { pcl::fromROSMsg(msg, sign_board_); });
  sub_ll2_ = create_subscription<PointCloud2>(
    "~/input/ll2_road_marking", 10,
    [this](const PointCloud2 & msg) -> void { pcl::fromROSMsg(msg, ll2_cloud_); });

  // Publisher
  pub_vis_ = create_publisher<Marker>("~/debug/projected_marker", 10);
  pub_image_ = create_publisher<sensor_msgs::msg::Image>("~/debug/lanelet2_overlay_image", 10);
}

void Lanelet2Overlay::on_info(const CameraInfo & msg)
{
  info_ = msg;
  camera_extrinsic_ = tf_subscriber_(info_->header.frame_id, "base_link");
}

void Lanelet2Overlay::on_image(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = common::decompress_to_cv_mat(msg);
  const rclcpp::Time stamp = msg.header.stamp;

  // Search synchronized pose
  float min_abs_dt = std::numeric_limits<float>::max();
  std::optional<Pose> synched_pose{std::nullopt};
  for (const auto & pose : pose_buffer_) {
    auto dt = (rclcpp::Time(pose.header.stamp) - stamp);
    auto abs_dt = std::abs(dt.seconds());
    if (abs_dt < min_abs_dt) {
      min_abs_dt = static_cast<float>(abs_dt);
      synched_pose = pose.pose;
    }
  }
  if (min_abs_dt > 0.1) synched_pose = std::nullopt;
  RCLCPP_INFO_STREAM(get_logger(), "dt: " << min_abs_dt << " image:" << stamp.nanoseconds());

  draw_overlay(image, synched_pose, stamp);
}

void Lanelet2Overlay::on_line_segments(const PointCloud2 & msg)
{
  const rclcpp::Time stamp = msg.header.stamp;

  // Search synchronized pose
  float min_dt = std::numeric_limits<float>::max();
  geometry_msgs::msg::PoseStamped synched_pose;
  for (const auto & pose : pose_buffer_) {
    auto dt = (rclcpp::Time(pose.header.stamp) - stamp);
    auto abs_dt = std::abs(dt.seconds());
    if (abs_dt < min_dt) {
      min_dt = static_cast<float>(abs_dt);
      synched_pose = pose;
    }
  }
  if (min_dt > 0.1) return;
  auto latest_pose_stamp = rclcpp::Time(pose_buffer_.back().header.stamp);

  LineSegments line_segments_cloud;
  pcl::fromROSMsg(msg, line_segments_cloud);
  make_vis_marker(line_segments_cloud, synched_pose.pose, stamp);
}

void Lanelet2Overlay::draw_overlay(
  const cv::Mat & image, const std::optional<Pose> & pose, const rclcpp::Time & stamp)
{
  if (ll2_cloud_.empty()) return;

  cv::Mat overlaid_image = cv::Mat::zeros(image.size(), CV_8UC3);

  using common::extract_near_line_segments;
  if (pose) {
    draw_overlay_line_segments(
      overlaid_image, *pose,
      extract_near_line_segments(ll2_cloud_, common::pose_to_se3(*pose), 60));
    draw_overlay_line_segments(
      overlaid_image, *pose,
      extract_near_line_segments(sign_board_, common::pose_to_se3(*pose), 60));
  }

  cv::Mat show_image;
  cv::addWeighted(image, 0.8, overlaid_image, 0.8, 1, show_image);
  common::publish_image(*pub_image_, show_image, stamp);
}

void Lanelet2Overlay::draw_overlay_line_segments(
  cv::Mat & image, const Pose & pose, const LineSegments & near_segments)
{
  if (!camera_extrinsic_.has_value()) return;
  if (!info_.has_value()) return;

  Eigen::Matrix3f k =
    Eigen::Map<Eigen::Matrix<double, 3, 3> >(info_->k.data()).cast<float>().transpose();
  Eigen::Affine3f t = camera_extrinsic_.value();

  Eigen::Affine3f transform = ground_plane_.align_with_slope(common::pose_to_affine(pose));

  auto project_line_segment =
    [k, t, transform](
      const Eigen::Vector3f & p1,
      const Eigen::Vector3f & p2) -> std::tuple<bool, cv::Point2i, cv::Point2i> {
    Eigen::Vector3f from_camera1 = k * t.inverse() * transform.inverse() * p1;
    Eigen::Vector3f from_camera2 = k * t.inverse() * transform.inverse() * p2;
    constexpr float epsilon = 0.1f;
    bool p1_is_visible = from_camera1.z() > epsilon;
    bool p2_is_visible = from_camera2.z() > epsilon;
    if ((!p1_is_visible) && (!p2_is_visible)) return {false, cv::Point2i{}, cv::Point2i{}};

    Eigen::Vector3f uv1;
    Eigen::Vector3f uv2;
    if (p1_is_visible) uv1 = from_camera1 / from_camera1.z();
    if (p2_is_visible) uv2 = from_camera2 / from_camera2.z();

    if ((p1_is_visible) && (p2_is_visible))
      return {
        true, cv::Point2i(static_cast<int>(uv1.x()), static_cast<int>(uv1.y())),
        cv::Point2i(static_cast<int>(uv2.x()), static_cast<int>(uv2.y()))};

    Eigen::Vector3f tangent = from_camera2 - from_camera1;
    float mu = (epsilon - from_camera1.z()) / (tangent.z());
    if (!p1_is_visible) {
      from_camera1 = from_camera1 + mu * tangent;
      uv1 = from_camera1 / from_camera1.z();
    }
    if (!p2_is_visible) {
      from_camera2 = from_camera1 + mu * tangent;
      uv2 = from_camera2 / from_camera2.z();
    }
    return {
      true, cv::Point2i(static_cast<int>(uv1.x()), static_cast<int>(uv1.y())),
      cv::Point2i(static_cast<int>(uv2.x()), static_cast<int>(uv2.y()))};
  };

  for (const pcl::PointNormal & pn : near_segments) {
    auto [success, u1, u2] = project_line_segment(pn.getVector3fMap(), pn.getNormalVector3fMap());
    if (success) cv::line(image, u1, u2, cv::Scalar(0, 255, 255), 2);
  }
}

void Lanelet2Overlay::make_vis_marker(
  const LineSegments & ls, const Pose & pose, const rclcpp::Time & stamp)
{
  Marker marker;
  marker.type = Marker::LINE_LIST;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.pose = pose;
  marker.scale.x = 0.1;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.7f;

  for (const auto pn : ls) {
    geometry_msgs::msg::Point p1;
    geometry_msgs::msg::Point p2;
    p1.x = pn.x;
    p1.y = pn.y;
    p1.z = pn.z;
    p2.x = pn.normal_x;
    p2.y = pn.normal_y;
    p2.z = pn.normal_z;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  pub_vis_->publish(marker);
}

}  // namespace yabloc::lanelet2_overlay

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yabloc::lanelet2_overlay::Lanelet2Overlay)
