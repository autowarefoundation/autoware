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

#include "yabloc_particle_filter/camera_corrector/camera_particle_corrector.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/color.hpp>
#include <yabloc_common/pose_conversions.hpp>
#include <yabloc_common/pub_sub.hpp>

namespace yabloc::modularized_particle_filter
{
cv::Point2f cv2pt(const Eigen::Vector3f v)
{
  const float METRIC_PER_PIXEL = 0.05;
  const float IMAGE_RADIUS = 400;
  return {-v.y() / METRIC_PER_PIXEL + IMAGE_RADIUS, -v.x() / METRIC_PER_PIXEL + 2 * IMAGE_RADIUS};
}

float normalized_atan2(const Eigen::Vector3f & t, float deg)
{
  float diff = std::atan2(t.y(), t.x()) - deg * M_PI / 180;
  diff = std::fmod(diff, M_PI);

  if (diff < 0) diff = -diff;

  if (diff < M_PI_2) {
    return 1 - diff / M_PI_2;
  } else if (diff < M_PI) {
    return diff / M_PI_2 - 1;
  } else {
    throw std::runtime_error("invalid cos");
  }
}

std::pair<CameraParticleCorrector::LineSegments, CameraParticleCorrector::LineSegments>
CameraParticleCorrector::filt(const LineSegments & iffy_lines)
{
  LineSegments good, bad;
  if (!latest_pose_.has_value()) {
    throw std::runtime_error("latest_pose_ is nullopt");
  }

  const Sophus::SE3f pose = common::pose_to_se3(latest_pose_.value().pose);

  // pcl::PointCloud<pcl::PointXYZRGB> rgb_cloud;
  for (const auto & line : iffy_lines) {
    const Eigen::Vector3f p1 = line.getVector3fMap();
    const Eigen::Vector3f p2 = line.getNormalVector3fMap();
    const float length = (p1 - p2).norm();
    const Eigen::Vector3f tangent = (p1 - p2).normalized();

    float score = 0;
    int count = 0;
    for (float distance = 0; distance < length; distance += 0.1f) {
      Eigen::Vector3f px = pose * (p2 + tangent * distance);
      CostMapValue v3 = cost_map_.at(px.topRows(2));
      float cos2 = normalized_atan2(pose.so3() * tangent, v3.angle);
      score += (cos2 * v3.intensity);
      count++;

      // pcl::PointXYZRGB rgb;
      // rgb.getVector3fMap() = px;
      // rgb.rgba = common::color_scale::blue_red(cos2 * v3[0] / 255.0f);
      // rgb_cloud.push_back(rgb);
    }

    if (score / count > 0.5f) {
      good.push_back(line);
    } else {
      bad.push_back(line);
    }
  }
  // common::publish_cloud(*pub_scored_cloud_, rgb_cloud, get_clock()->now());

  return {good, bad};
}
}  // namespace yabloc::modularized_particle_filter
