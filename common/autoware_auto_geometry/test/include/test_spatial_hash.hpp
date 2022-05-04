// Copyright 2019 the Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TEST_SPATIAL_HASH_HPP_
#define TEST_SPATIAL_HASH_HPP_

#include "geometry/spatial_hash.hpp"

#include <geometry_msgs/msg/point32.hpp>

#include <limits>
#include <vector>

using autoware::common::geometry::spatial_hash::Config2d;
using autoware::common::geometry::spatial_hash::Config3d;
using autoware::common::geometry::spatial_hash::SpatialHash;
using autoware::common::geometry::spatial_hash::SpatialHash2d;
using autoware::common::geometry::spatial_hash::SpatialHash3d;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

template <typename PointT>
class TypedSpatialHashTest : public ::testing::Test
{
public:
  TypedSpatialHashTest() : ref(), EPS(0.001F)
  {
    ref.x = 0.0F;
    ref.y = 0.0F;
    ref.z = 0.0F;
  }

protected:
  template <typename Cfg>
  void add_points(
    SpatialHash<PointT, Cfg> & hash, const uint32_t points_per_ring, const uint32_t num_rings,
    const float32_t dr, const float32_t dx = 0.0F, const float32_t dy = 0.0F)
  {
    const float32_t dth = 2.0F * 3.14159F / points_per_ring;

    // insert
    float32_t r = dr;
    for (uint32_t rdx = 0U; rdx < num_rings; ++rdx) {
      float32_t th = 0.0F;
      for (uint32_t pdx = 0U; pdx < points_per_ring; ++pdx) {
        PointT pt;
        pt.x = r * cosf(th) + dx;
        pt.y = r * sinf(th) + dy;
        pt.z = 0.0F;
        hash.insert(pt);
        th += dth;
      }
      r += dr;
    }
  }
  PointT ref;
  const float32_t EPS;
};  // SpatialHash
// test struct

// Instantiate tests for given types, add more types here as they are used
using PointTypesSpatialHash = ::testing::Types<geometry_msgs::msg::Point32>;
// cppcheck-suppress syntaxError
TYPED_TEST_SUITE(TypedSpatialHashTest, PointTypesSpatialHash, );
/// NOTE: This is the older version due to 1.8.0 of GTest. v1.8.1 uses TYPED_TEST_SUITE

///////////////////////////////////////////////////////////////
// TODO(christopher.ho) helper functions to simplify this stuff
/// all points in one bin
TYPED_TEST(TypedSpatialHashTest, OneBin)
{
  using PointT = TypeParam;
  const float32_t dr = 1.0F;
  Config2d cfg{-10.0F, 10.0F, -10.0F, 10.0F, 3.0F, 1024U};
  SpatialHash2d<PointT> hash{cfg};

  // build concentric rings around origin
  const uint32_t PTS_PER_RING = 10U;
  const uint32_t NUM_RINGS = 1U;
  this->add_points(hash, PTS_PER_RING, NUM_RINGS, dr);

  // loop through all points
  float r = dr - this->EPS;
  for (uint32_t rdx = 0U; rdx < NUM_RINGS + 1U; ++rdx) {
    const uint32_t n_pts = rdx * PTS_PER_RING;
    const auto & neighbors = hash.near(this->ref, r);
    uint32_t points_seen = 0U;
    for (const auto & itd : neighbors) {
      const PointT & pt = itd;
      const float dist = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
      ASSERT_LT(dist, r);
      ASSERT_FLOAT_EQ(dist, itd.get_distance());
      ++points_seen;
    }
    ASSERT_EQ(points_seen, n_pts);
    r += dr;
    // Make sure statistics are consistent
    EXPECT_EQ(hash.bins_hit(), 9U * (1U + rdx));
    EXPECT_EQ(hash.neighbors_found(), rdx * PTS_PER_RING);
  }
  // check iterators etc.
  uint32_t count = 0U;
  for (auto iter = hash.cbegin(); iter != hash.cend(); ++iter) {
    // TODO(c.ho) check uniqueness of stuff
    ++count;
  }
  EXPECT_EQ(PTS_PER_RING, count);
  hash.clear();
  EXPECT_EQ(hash.size(), 0U);
  EXPECT_TRUE(hash.empty());
  count = 0U;
  for (auto it : hash) {
    // TODO(c.ho) check uniqueness of stuff
    (void)it;
    ++count;
  }
  EXPECT_EQ(count, 0U);
}
/// test out of bounds points
TYPED_TEST(TypedSpatialHashTest, Oob)
{
  using PointT = TypeParam;
  const float32_t dr = 20.0F;
  Config2d cfg{-2.0F, 2.0F, -2.0F, 2.0F, dr + this->EPS, 1024U};
  SpatialHash2d<PointT> hash{cfg};

  // build concentric rings around origin
  const uint32_t PTS_PER_RING = 12U;
  this->add_points(hash, PTS_PER_RING, 1U, dr);

  // loop through all points
  float32_t r = dr + this->EPS;
  const uint32_t n_pts = PTS_PER_RING;
  const auto & nbrs = hash.near(this->ref, r);
  uint32_t points_seen = 0U;
  for (const auto itd : nbrs) {
    const PointT & pt = itd;
    const float32_t dist = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
    ASSERT_LT(dist, r);
    ASSERT_GT(dist, 10.0F * sqrtf(2.0F));
    ASSERT_FLOAT_EQ(dist, itd.get_distance());
    ++points_seen;
  }

  ASSERT_EQ(points_seen, n_pts);
}
// 3d test case
TYPED_TEST(TypedSpatialHashTest, 3d)
{
  using PointT = TypeParam;
  Config3d cfg{-30.0F, 30.0F, -30.0F, 30.0F, -30.0F, 30.0F, 30.0F, 1024U};
  SpatialHash3d<PointT> hash{cfg};
  EXPECT_TRUE(hash.empty());

  // build concentric rings around origin
  const uint32_t points_per_ring = 32U;
  const uint32_t num_rings = 5U;
  const float32_t dth = 2.0F * 3.14159F / points_per_ring;
  std::vector<PointT> pts{};

  // insert
  const float32_t r = 10.0F;
  float32_t phi = 0.0f;
  for (uint32_t rdx = 0U; rdx < num_rings; ++rdx) {
    float32_t th = 0.0F;
    for (uint32_t pdx = 0U; pdx < points_per_ring; ++pdx) {
      PointT pt;
      pt.x = r * cosf(th) * cosf(phi);
      pt.y = r * sinf(th) * cosf(phi);
      pt.z = r * sinf(phi);
      pts.push_back(pt);
      th += dth;
    }
    hash.insert(pts.begin(), pts.end());
    pts.clear();
    phi += 1.0f;
  }
  EXPECT_FALSE(hash.empty());
  EXPECT_EQ(hash.size(), num_rings * points_per_ring);

  // loop through all points
  const uint32_t n_pts = num_rings * points_per_ring;
  const auto & neighbors = hash.near(this->ref, r + this->EPS);
  uint32_t points_seen = 0U;
  for (const auto & itd : neighbors) {
    const PointT & pt = itd;
    const float32_t dist = sqrtf((pt.x * pt.x) + (pt.y * pt.y) + (pt.z * pt.z));
    ASSERT_LT(dist, r + this->EPS);
    ASSERT_FLOAT_EQ(dist, itd.get_distance());
    ++points_seen;
  }

  ASSERT_EQ(points_seen, n_pts);

  // check iterators etc.
  uint32_t count = 0U;
  for (auto iter = hash.cbegin(); iter != hash.cend(); ++iter) {
    // TODO(c.ho) check uniqueness of stuff
    ++count;
  }
  EXPECT_EQ(n_pts, count);
  hash.clear();
  EXPECT_EQ(hash.size(), 0U);
  EXPECT_TRUE(hash.empty());
  count = 0U;
  for (auto it : hash) {
    // TODO(c.ho) check uniqueness of stuff
    (void)it;
    ++count;
  }
  EXPECT_EQ(count, 0U);
}

/// edge cases
TEST(SpatialHashConfig, BadCases)
{
  // negative side length
  EXPECT_THROW(Config2d({-30.0F, 30.0F, -30.0F, 30.0F, -1.0F, 1024U}), std::domain_error);
  // min_x >= max_x
  EXPECT_THROW(Config2d({31.0F, 30.0F, -30.0F, 30.0F, 1.0F, 1024U}), std::domain_error);
  // min_y >= max_y
  EXPECT_THROW(Config2d({-30.0F, 30.0F, 30.1F, 30.0F, 1.0F, 1024U}), std::domain_error);
  // min_z >= max_z
  EXPECT_THROW(
    Config3d({-30.0F, 30.0F, -30.0F, 30.0F, 31.0F, 30.0F, 1.0F, 1024U}), std::domain_error);
  // floating point limit
  constexpr float32_t max_float = std::numeric_limits<float32_t>::max();
  EXPECT_THROW(Config2d({-max_float, max_float, -30.0F, 30.0F, 1.0F, 1024U}), std::domain_error);
  EXPECT_THROW(
    Config3d({-30.0F, 30.0F, -max_float, max_float, -30.0F, 30.0F, 1.0F, 1024U}),
    std::domain_error);
  EXPECT_THROW(
    Config3d({-30.0F, 30.0F, -30.0F, 30.0F, -max_float, max_float, 1.0F, 1024U}),
    std::domain_error);
  // y would overflow
  // constexpr float32_t big_float =
  //   static_cast<float32_t>(std::numeric_limits<uint64_t>::max() / 4UL);
  // EXPECT_THROW(Config({-big_float, big_float, -big_float, big_float, 0.001F, 1024U}),
  //   std::domain_error);
  // z would overflow
  // EXPECT_THROW(
  //   Config3d({-30.0F, 30.0F, -99999.0F, 99999.0F, -99999.0F, 99999.0F, 0.001F, 1024U}),
  //   std::domain_error);
  // TODO(c.ho) re-enable test when we can actually check unsigned integer multiplication overflow
}
#endif  // TEST_SPATIAL_HASH_HPP_
