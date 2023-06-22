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

#include "yabloc_common/transform_line_segments.hpp"

namespace yabloc::common
{
pcl::PointCloud<pcl::PointXYZLNormal> transform_line_segments(
  const pcl::PointCloud<pcl::PointXYZLNormal> & src, const Sophus::SE3f & transform)
{
  pcl::PointCloud<pcl::PointXYZLNormal> dst;
  for (const pcl::PointXYZLNormal & line : src) {
    Eigen::Vector3f p1 = line.getVector3fMap();
    Eigen::Vector3f p2 = line.getNormalVector3fMap();

    pcl::PointXYZLNormal transformed;
    transformed.getVector3fMap() = transform * p1;
    transformed.getNormalVector3fMap() = transform * p2;
    transformed.label = line.label;
    dst.push_back(transformed);
  }
  return dst;
}

pcl::PointCloud<pcl::PointNormal> transform_line_segments(
  const pcl::PointCloud<pcl::PointNormal> & src, const Sophus::SE3f & transform)
{
  pcl::PointCloud<pcl::PointNormal> dst;
  for (const pcl::PointNormal & line : src) {
    Eigen::Vector3f p1 = line.getVector3fMap();
    Eigen::Vector3f p2 = line.getNormalVector3fMap();

    pcl::PointNormal transformed;
    transformed.getVector3fMap() = transform * p1;
    transformed.getNormalVector3fMap() = transform * p2;
    dst.push_back(transformed);
  }
  return dst;
}
}  // namespace yabloc::common
