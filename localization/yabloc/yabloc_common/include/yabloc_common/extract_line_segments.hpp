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

#ifndef YABLOC_COMMON__EXTRACT_LINE_SEGMENTS_HPP_
#define YABLOC_COMMON__EXTRACT_LINE_SEGMENTS_HPP_

#include <sophus/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace yabloc::common
{
pcl::PointCloud<pcl::PointNormal> extract_near_line_segments(
  const pcl::PointCloud<pcl::PointNormal> & line_segments, const Sophus::SE3f & transform,
  const float max_range = 40);

}  // namespace yabloc::common

#endif  // YABLOC_COMMON__EXTRACT_LINE_SEGMENTS_HPP_
