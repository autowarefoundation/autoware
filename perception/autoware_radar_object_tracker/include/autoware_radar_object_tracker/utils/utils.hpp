// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef AUTOWARE_RADAR_OBJECT_TRACKER__UTILS__UTILS_HPP_
#define AUTOWARE_RADAR_OBJECT_TRACKER__UTILS__UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "autoware_perception_msgs/msg/detected_object.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>
namespace autoware::radar_object_tracker::utils
{
// matrix concatenate
Eigen::MatrixXd stackMatricesVertically(const std::vector<Eigen::MatrixXd> & matrices);
Eigen::MatrixXd stackMatricesDiagonally(const std::vector<Eigen::MatrixXd> & matrices);

}  // namespace autoware::radar_object_tracker::utils

#endif  // AUTOWARE_RADAR_OBJECT_TRACKER__UTILS__UTILS_HPP_
