// Copyright 2026 Autoware Foundation
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

#ifndef COVARIANCE_CONVERSION_HPP_
#define COVARIANCE_CONVERSION_HPP_

#include <array>
#include <cstddef>
#include <utility>

namespace autoware::geo_pose_projector
{
using Covariance6x6 = std::array<double, 36>;

/**
 * @brief Convert a GeoPoseWithCovariance covariance into the map-frame (x, y, z, roll, pitch, yaw)
 *        ordering used by geometry_msgs/PoseWithCovariance.
 *
 * geographic_msgs/GeoPoseWithCovariance orders its 6x6 covariance as
 * (Lat, Lon, Alt, rot_E, rot_N, rot_U), i.e. the first two position axes are (North, East),
 * while the map frame is (x = East, y = North, z = Up). The orientation axes are already ENU.
 *
 * The conversion is therefore the permutation P = (1, 0, 2, 3, 4, 5) applied to both rows and
 * columns: out(i, j) = in(P[i], P[j]). Swapping only the two diagonal variances leaves the
 * cross-covariances of Lat/Lon with Alt and with the orientation terms in the wrong slots.
 *
 * @note The rotation between true North and the projected grid North (meridian convergence) is
 *       not taken into account.
 */
inline Covariance6x6 convert_lat_lon_alt_covariance_to_map(const Covariance6x6 & geo_covariance)
{
  constexpr std::size_t dim = 6;
  constexpr std::array<std::size_t, dim> permutation{1, 0, 2, 3, 4, 5};

  Covariance6x6 map_covariance{};
  for (std::size_t i = 0; i < dim; ++i) {
    for (std::size_t j = 0; j < dim; ++j) {
      map_covariance[i * dim + j] = geo_covariance[permutation[i] * dim + permutation[j]];
    }
  }
  return map_covariance;
}
}  // namespace autoware::geo_pose_projector

#endif  // COVARIANCE_CONVERSION_HPP_
