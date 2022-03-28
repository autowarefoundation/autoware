// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE_POINT_TYPES__TYPES_HPP_
#define AUTOWARE_POINT_TYPES__TYPES_HPP_

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <pcl/point_types.h>

#include <cmath>
#include <tuple>

namespace autoware_point_types
{
template <class T>
bool float_eq(const T a, const T b, const T eps = 10e-6)
{
  return std::fabs(a - b) < eps;
}

struct PointXYZI
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  friend bool operator==(const PointXYZI & p1, const PointXYZI & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) && float_eq<float>(p1.intensity, p2.intensity);
  }
};

enum ReturnType : uint8_t {
  INVALID = 0,
  SINGLE_STRONGEST,
  SINGLE_LAST,
  DUAL_STRONGEST_FIRST,
  DUAL_STRONGEST_LAST,
  DUAL_WEAK_FIRST,
  DUAL_WEAK_LAST,
  DUAL_ONLY,
};

struct PointXYZIRADRT
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  uint16_t ring{0U};
  float azimuth{0.0F};
  float distance{0.0F};
  uint8_t return_type{0U};
  double time_stamp{0.0};
  friend bool operator==(const PointXYZIRADRT & p1, const PointXYZIRADRT & p2) noexcept
  {
    return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
           float_eq<float>(p1.z, p2.z) && float_eq<float>(p1.intensity, p2.intensity) &&
           p1.ring == p2.ring && float_eq<float>(p1.azimuth, p2.azimuth) &&
           float_eq<float>(p1.distance, p2.distance) && p1.return_type == p2.return_type &&
           float_eq<float>(p1.time_stamp, p2.time_stamp);
  }
};

enum class PointIndex { X, Y, Z, Intensity, Ring, Azimuth, Distance, ReturnType, TimeStamp };

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(azimuth);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(distance);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(return_type);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(time_stamp);

using PointXYZIRADRTGenerator = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, point_cloud_msg_wrapper::field_intensity_generator,
  point_cloud_msg_wrapper::field_ring_generator, field_azimuth_generator, field_distance_generator,
  field_return_type_generator, field_time_stamp_generator>;

}  // namespace autoware_point_types

POINT_CLOUD_REGISTER_POINT_STRUCT(
  autoware_point_types::PointXYZIRADRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, azimuth, azimuth)(float, distance, distance)(std::uint8_t, return_type, return_type)(
    double, time_stamp, time_stamp))

#endif  // AUTOWARE_POINT_TYPES__TYPES_HPP_
