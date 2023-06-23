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

#pragma once

#include <Eigen/Eigen>

namespace pointcloud_preprocessor
{

/**
 * This holds the coordinate transformation information of the point cloud.
 * Usage example:
 *   \code
 *   if (transform_info.need_transform) {
 *       point = transform_info.eigen_transform * point;
 *   }
 *   \endcode
 */
struct TransformInfo
{
  TransformInfo()
  {
    eigen_transform = Eigen::Matrix4f::Identity(4, 4);
    need_transform = false;
  }

  Eigen::Matrix4f eigen_transform;
  bool need_transform;
};

}  // namespace pointcloud_preprocessor
