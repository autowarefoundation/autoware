// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
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

#include "autoware_auto_geometry/convex_hull.hpp"
#include "autoware_auto_geometry/hull_pockets.hpp"

#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <iterator>
#include <list>
#include <utility>
#include <vector>

using autoware::common::geometry::point_adapter::x_;
using autoware::common::geometry::point_adapter::y_;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

template <typename PointT>
class TypedHullPocketsTest : public ::testing::Test
{
protected:
  PointT make(const float32_t x, const float32_t y, const float32_t z)
  {
    PointT ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
  }
};  // class TypedHullPocketsTest

using PointTypes = ::testing::Types<geometry_msgs::msg::Point32>;
// cppcheck-suppress syntaxError
TYPED_TEST_SUITE(TypedHullPocketsTest, PointTypes, );
/// NOTE: This is the older version due to 1.8.0 of GTest. v1.8.1 uses TYPED_TEST_SUITE

// Inner test function, shared among all the tests
template <typename Iter>
typename std::vector<std::vector<typename std::iterator_traits<Iter>::value_type>>
compute_hull_and_pockets(const Iter polygon_start, const Iter polygon_end)
{
  auto convex_hull_list =
    std::list<typename std::iterator_traits<Iter>::value_type>{polygon_start, polygon_end};
  const auto cvx_hull_end = autoware::common::geometry::convex_hull(convex_hull_list);

  const typename decltype(convex_hull_list)::const_iterator list_beginning =
    convex_hull_list.begin();
  const auto pockets = autoware::common::geometry::hull_pockets(
    polygon_start, polygon_end, list_beginning, cvx_hull_end);

  return pockets;
}

// Test for a triangle - any triangle should really not result in any pockets.
TYPED_TEST(TypedHullPocketsTest, Triangle)
{
  const auto polygon = std::vector<decltype(this->make(0, 0, 0))>{
    this->make(0, 0, 0), this->make(2, 0, 0), this->make(1, 1, 0)};

  const auto pockets = compute_hull_and_pockets(polygon.begin(), polygon.end());
  ASSERT_EQ(pockets.size(), 0u);
}

// Test for the use case:
// +--------------------+
// |                    |
// |                    |
// |                    |
// |                    |
// |                    |
// |                    |
// +--------------------+
// This should not result in any pockets.
TYPED_TEST(TypedHullPocketsTest, Box)
{
  const auto polygon = std::vector<decltype(this->make(0, 0, 0))>{
    this->make(0, 0, 0), this->make(2, 0, 0), this->make(2, 1, 0), this->make(0, 1, 0)};

  const auto pockets = compute_hull_and_pockets(polygon.begin(), polygon.end());
  ASSERT_EQ(pockets.size(), 0u);
}

// Test for the use case:
//    +-----+            +-----+
//   /      |            |      |
//  /       |            |       |
// +        |            |        +
// |        |            |        |
// |        |            |        |
// |        --------------        |
// |                              |
// |                              |
// |                              |
// |                              |
// +------------------------------+
// This should come up with a single box on the top left.
TYPED_TEST(TypedHullPocketsTest, UShape)
{
  const auto polygon = std::vector<decltype(this->make(0, 0, 0))>{
    this->make(0, 0, 0), this->make(5, 0, 0), this->make(5, 4.5, 0), this->make(4, 5, 0),
    this->make(4, 2, 0), this->make(2, 2, 0), this->make(2, 5, 0),   this->make(0, 4.5, 0),
  };

  const auto pockets = compute_hull_and_pockets(polygon.begin(), polygon.end());

  ASSERT_EQ(pockets.size(), 1u);
  ASSERT_EQ(pockets[0].size(), 4u);
  ASSERT_FLOAT_EQ(x_(pockets[0][0]), 4.0);
  ASSERT_FLOAT_EQ(y_(pockets[0][0]), 5.0);
  ASSERT_FLOAT_EQ(x_(pockets[0][1]), 4.0);
  ASSERT_FLOAT_EQ(y_(pockets[0][1]), 2.0);
}

// Test for the use case:
//                    +------+
//                    |      |
//                    |      |
//                    |      |
// +------------------+      +------+
// |                                |
// |                                |
// |                                |
// +--------------------------------+
//
// This should come up with two pockets, a triangle on the top left and one on the
// top right.
TYPED_TEST(TypedHullPocketsTest, TypicalGap)
{
  const auto polygon = std::vector<decltype(this->make(0, 0, 0))>{
    this->make(0, 0, 0), this->make(10, 0, 0), this->make(10, 2, 0), this->make(8, 2, 0),
    this->make(8, 4, 0), this->make(6, 4, 0),  this->make(6, 2, 0),  this->make(0, 2, 0),
  };

  const auto pockets = compute_hull_and_pockets(polygon.begin(), polygon.end());

  ASSERT_EQ(pockets.size(), 2u);
  ASSERT_EQ(pockets[0].size(), 3u);
  ASSERT_EQ(pockets[1].size(), 3u);
  // TODO(s.me) check for correct pocket positioning
}

// Test for the use case:
//
//   +-----------------+
//    |                |
//     |               |
//      +              |
//     /               |
//    /                |
//   +-----------------+
//
// This should come up with one pocket, in particular a pocket that contains
// the segment of the final to the first point.
TYPED_TEST(TypedHullPocketsTest, EndsInPocket)
{
  const auto polygon = std::vector<decltype(this->make(0, 0, 0))>{
    this->make(0, 0, 0), this->make(2, 0, 0),   this->make(2, 2, 0),
    this->make(0, 2, 0), this->make(0.1, 1, 0),
  };

  const auto pockets = compute_hull_and_pockets(polygon.begin(), polygon.end());

  ASSERT_EQ(pockets.size(), 1u);
  ASSERT_EQ(pockets[0].size(), 3u);
  // TODO(s.me) check for correct pocket positioning
}
