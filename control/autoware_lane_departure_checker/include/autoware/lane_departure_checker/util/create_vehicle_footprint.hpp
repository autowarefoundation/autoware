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
// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef AUTOWARE__LANE_DEPARTURE_CHECKER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_
#define AUTOWARE__LANE_DEPARTURE_CHECKER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_

#include <Eigen/Dense>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <tf2/utils.h>

struct FootprintMargin
{
  double lon;
  double lat;
};

inline FootprintMargin calcFootprintMargin(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto Cov_in_map = covariance.covariance;
  Eigen::Matrix2d Cov_xy_map;
  Cov_xy_map << Cov_in_map[0 * 6 + 0], Cov_in_map[0 * 6 + 1], Cov_in_map[1 * 6 + 0],
    Cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d R_map2vehicle;
  R_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d Cov_xy_vehicle = R_map2vehicle * Cov_xy_map * R_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in Cov_xy_vehicle(0,0), Cov_xy_vehicle(1,1) respectively.
  return FootprintMargin{Cov_xy_vehicle(0, 0) * scale, Cov_xy_vehicle(1, 1) * scale};
}

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_
