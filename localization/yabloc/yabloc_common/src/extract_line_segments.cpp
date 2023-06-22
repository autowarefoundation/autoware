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

#include "yabloc_common/extract_line_segments.hpp"

namespace yabloc::common
{
pcl::PointCloud<pcl::PointNormal> extract_near_line_segments(
  const pcl::PointCloud<pcl::PointNormal> & line_segments, const Sophus::SE3f & transform,
  const float max_range)
{
  constexpr double sqrt_two = std::sqrt(2);
  const Eigen::Vector3f pose_vector = transform.translation();

  // All line segments contained in a square with max_range on one side must be taken out,
  // so pick up those that are closer than the **diagonals** of the square.
  auto check_intersection = [max_range, pose_vector](const pcl::PointNormal & pn) -> bool {
    const Eigen::Vector3f from = pn.getVector3fMap() - pose_vector;
    const Eigen::Vector3f to = pn.getNormalVector3fMap() - pose_vector;

    Eigen::Vector3f tangent = to - from;
    if (tangent.squaredNorm() < 1e-3f) {
      return from.norm() < sqrt_two * max_range;
    }

    float inner = from.dot(tangent);
    float mu = std::clamp(inner / tangent.squaredNorm(), -1.0f, 0.0f);
    Eigen::Vector3f nearest = from - tangent * mu;
    return nearest.norm() < sqrt_two * max_range;
  };

  pcl::PointCloud<pcl::PointNormal> dst;
  for (const pcl::PointNormal & pn : line_segments) {
    if (check_intersection(pn)) {
      dst.push_back(pn);
    }
  }
  return dst;
}

}  // namespace yabloc::common
