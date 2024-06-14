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

#ifndef YABLOC_COMMON__GROUND_SERVER__UTIL_HPP_
#define YABLOC_COMMON__GROUND_SERVER__UTIL_HPP_

#include <Eigen/Core>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <unordered_set>
#include <vector>

namespace yabloc::ground_server
{
void inline upsample_line_string(
  const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::Vector3f f(
    static_cast<float>(from.x()), static_cast<float>(from.y()), static_cast<float>(from.z()));
  Eigen::Vector3f t(
    static_cast<float>(to.x()), static_cast<float>(to.y()), static_cast<float>(to.z()));
  float length = (t - f).norm();
  Eigen::Vector3f d = (t - f).normalized();
  for (float l = 0; l < length; l += 0.5f) {
    pcl::PointXYZ xyz;
    xyz.getVector3fMap() = (f + l * d);
    cloud->push_back(xyz);
  }
};

std::vector<int> inline merge_indices(
  const std::vector<int> & indices1, const std::vector<int> & indices2)
{
  std::unordered_set<int> set;
  for (int i : indices1) set.insert(i);
  for (int i : indices2) set.insert(i);

  std::vector<int> indices;
  indices.assign(set.begin(), set.end());
  return indices;
}

}  // namespace yabloc::ground_server

#endif  // YABLOC_COMMON__GROUND_SERVER__UTIL_HPP_
