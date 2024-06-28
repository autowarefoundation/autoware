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

#include "yabloc_pose_initializer/camera/projector_module.hpp"

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>

namespace yabloc::initializer
{
ProjectorModule::ProjectorModule(rclcpp::Node * node)
: info_(node), tf_subscriber_(node->get_clock()), logger_(node->get_logger())
{
}

cv::Point2i to_cv_point(const Eigen::Vector3f & v)
{
  const float image_size = 800;
  const float max_range = 30;

  cv::Point pt;
  pt.x = static_cast<int>(-v.y() / max_range * image_size * 0.5f + image_size / 2.f);
  pt.y = static_cast<int>(-v.x() / max_range * image_size * 0.5f + image_size / 2.f);
  return pt;
}

cv::Mat ProjectorModule::project_image(const cv::Mat & mask_image)
{
  // project semantics on plane
  std::vector<cv::Mat> masks;
  cv::split(mask_image, masks);
  std::vector<cv::Scalar> colors = {
    cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255)};

  cv::Mat projected_image =
    cv::Mat::zeros(cv::Size(800, 800), CV_8UC3);  // NOLINT
                                                  // suppress hicpp-signed-bitwise
                                                  // from OpenCV
  for (int i = 0; i < 3; i++) {
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(masks[i], contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    std::vector<std::vector<cv::Point> > projected_contours;
    for (const auto & contour : contours) {
      std::vector<cv::Point> projected;
      for (const auto & c : contour) {
        auto opt = project_func_(c);
        if (!opt.has_value()) continue;

        cv::Point2i pt = to_cv_point(opt.value());
        projected.push_back(pt);
      }
      if (projected.size() > 2) {
        projected_contours.push_back(projected);
      }
    }
    cv::drawContours(projected_image, projected_contours, -1, colors[i], -1);
  }
  return projected_image;
}

bool ProjectorModule::define_project_func()
{
  if (project_func_) return true;

  if (info_.is_camera_info_nullopt()) {
    RCLCPP_WARN_STREAM(logger_, "camera info is not ready");
    return false;
  }
  Eigen::Matrix3f intrinsic_inv = info_.intrinsic().inverse();

  std::optional<Eigen::Affine3f> camera_extrinsic =
    tf_subscriber_(info_.get_frame_id(), "base_link");
  if (!camera_extrinsic.has_value()) {
    RCLCPP_WARN_STREAM(logger_, "camera tf_static is not ready");
    return false;
  }

  const Eigen::Vector3f t = camera_extrinsic->translation();
  const Eigen::Quaternionf q(camera_extrinsic->rotation());

  // TODO(KYabuuchi) This will take into account ground tilt and camera vibration someday.
  project_func_ = [intrinsic_inv, q, t](const cv::Point & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f u3(static_cast<float>(u.x), static_cast<float>(u.y), 1);
    Eigen::Vector3f u_bearing = (q * intrinsic_inv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;
    return v;
  };
  return true;
}
}  // namespace yabloc::initializer
