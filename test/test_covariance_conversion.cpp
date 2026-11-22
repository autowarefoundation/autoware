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

#include "covariance_conversion.hpp"

#include <gtest/gtest.h>

#include <cstddef>

using autoware::geo_pose_projector::convert_lat_lon_alt_covariance_to_map;
using autoware::geo_pose_projector::Covariance6x6;

namespace
{
constexpr std::size_t dim = 6;
constexpr std::size_t lat = 0;
constexpr std::size_t lon = 1;
constexpr std::size_t alt = 2;
constexpr std::size_t x = 0;
constexpr std::size_t y = 1;
constexpr std::size_t z = 2;

double at(const Covariance6x6 & cov, const std::size_t row, const std::size_t col)
{
  return cov[row * dim + col];
}

void set_symmetric(
  Covariance6x6 & cov, const std::size_t row, const std::size_t col, const double value)
{
  cov[row * dim + col] = value;
  cov[col * dim + row] = value;
}
}  // namespace

TEST(ConvertLatLonAltCovarianceToMap, SwapsLatLonVariances)
{
  Covariance6x6 geo{};
  geo[lat * dim + lat] = 4.0;  // North variance
  geo[lon * dim + lon] = 9.0;  // East variance
  geo[alt * dim + alt] = 16.0;

  const auto map = convert_lat_lon_alt_covariance_to_map(geo);

  EXPECT_DOUBLE_EQ(at(map, x, x), 9.0);   // East
  EXPECT_DOUBLE_EQ(at(map, y, y), 4.0);   // North
  EXPECT_DOUBLE_EQ(at(map, z, z), 16.0);  // Up
}

TEST(ConvertLatLonAltCovarianceToMap, MovesCrossCovariancesWithTheirAxes)
{
  Covariance6x6 geo{};
  set_symmetric(geo, lat, lon, 0.5);
  set_symmetric(geo, lat, alt, 1.0);  // North-Up
  set_symmetric(geo, lon, alt, 2.0);  // East-Up
  set_symmetric(geo, lat, 5, 3.0);    // North-yaw
  set_symmetric(geo, lon, 5, 7.0);    // East-yaw

  const auto map = convert_lat_lon_alt_covariance_to_map(geo);

  EXPECT_DOUBLE_EQ(at(map, x, y), 0.5);
  EXPECT_DOUBLE_EQ(at(map, x, z), 2.0);  // East-Up
  EXPECT_DOUBLE_EQ(at(map, z, x), 2.0);
  EXPECT_DOUBLE_EQ(at(map, y, z), 1.0);  // North-Up
  EXPECT_DOUBLE_EQ(at(map, z, y), 1.0);
  EXPECT_DOUBLE_EQ(at(map, x, 5), 7.0);  // East-yaw
  EXPECT_DOUBLE_EQ(at(map, y, 5), 3.0);  // North-yaw
}

TEST(ConvertLatLonAltCovarianceToMap, KeepsAltitudeAndOrientationBlockUnchanged)
{
  Covariance6x6 geo{};
  for (std::size_t i = 0; i < geo.size(); ++i) {
    geo[i] = static_cast<double>(i);
  }

  const auto map = convert_lat_lon_alt_covariance_to_map(geo);

  for (std::size_t i = alt; i < dim; ++i) {
    for (std::size_t j = alt; j < dim; ++j) {
      EXPECT_DOUBLE_EQ(at(map, i, j), at(geo, i, j)) << "i=" << i << " j=" << j;
    }
  }
}

TEST(ConvertLatLonAltCovarianceToMap, PreservesSymmetryAndIsAnInvolution)
{
  Covariance6x6 geo{};
  double value = 1.0;
  for (std::size_t i = 0; i < dim; ++i) {
    for (std::size_t j = i; j < dim; ++j) {
      set_symmetric(geo, i, j, value);
      value += 1.0;
    }
  }

  const auto map = convert_lat_lon_alt_covariance_to_map(geo);
  for (std::size_t i = 0; i < dim; ++i) {
    for (std::size_t j = 0; j < dim; ++j) {
      EXPECT_DOUBLE_EQ(at(map, i, j), at(map, j, i));
    }
  }

  EXPECT_EQ(convert_lat_lon_alt_covariance_to_map(map), geo);
}
