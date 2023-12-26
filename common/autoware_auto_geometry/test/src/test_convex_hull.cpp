// Copyright 2017-2019 the Autoware Foundation
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

#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <list>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

template <typename PointT>
class TypedConvexHullTest : public ::testing::Test
{
protected:
  std::list<PointT> list;

  typename std::list<PointT>::const_iterator convex_hull()
  {
    const auto ret = autoware::common::geometry::convex_hull<PointT>(list);
    return ret;
  }

  void check_hull(
    const typename std::list<PointT>::const_iterator last, const std::vector<PointT> & expect,
    const bool8_t strict = true)
  {
    uint32_t items = 0U;
    for (auto & pt : expect) {
      bool8_t found = false;
      auto it = list.begin();
      while (it != last) {
        constexpr float32_t TOL = 1.0E-6F;
        if (
          fabsf(pt.x - it->x) <= TOL && fabsf(pt.y - it->y) <= TOL &&
          (fabsf(pt.z - it->z) <= TOL || !strict))  // TODO(@estive): z if only strict
        {
          found = true;
          break;
        }
        ++it;
      }
      ASSERT_TRUE(found) << items;
      ++items;
    }
    if (strict) {
      ASSERT_EQ(items, expect.size());
    }
  }

  PointT make(const float32_t x, const float32_t y, const float32_t z)
  {
    PointT ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    return ret;
  }
};  // class convex_hull_test

// Instantiate tests for given types, add more types here as they are used
using PointTypes = ::testing::Types<geometry_msgs::msg::Point32>;
// cppcheck-suppress syntaxError
TYPED_TEST_SUITE(TypedConvexHullTest, PointTypes, );
/// NOTE: This is the older version due to 1.8.0 of GTest. v1.8.1 uses TYPED_TEST_SUITE

//////////////////////////////////////////

/*
  3
    2
1
*/
TYPED_TEST(TypedConvexHullTest, Triangle)
{
  std::vector<TypeParam> expect({this->make(1, 0, 0), this->make(3, 1, 0), this->make(2, 2, 0)});
  this->list.insert(this->list.begin(), expect.begin(), expect.end());

  const auto last = this->convex_hull();

  this->check_hull(last, expect);
  ASSERT_EQ(this->list.size(), 3U);
  // check order
  auto it = this->list.begin();
  ASSERT_FLOAT_EQ(it->x, 1);
  ++it;  // node 1
  ASSERT_FLOAT_EQ(it->x, 3);
  ++it;  // node 2
  ASSERT_FLOAT_EQ(it->x, 2);
  ++it;  // node 3
  ASSERT_EQ(it, last);
}
/*
  2       1

4
        3
*/
// test that things get reordered to ccw
TYPED_TEST(TypedConvexHullTest, Quadrilateral)
{
  std::vector<TypeParam> expect(
    {this->make(-1, -1, 1), this->make(-5, -1, 2), this->make(-2, -6, 3), this->make(-6, -5, 4)});
  this->list.insert(this->list.begin(), expect.begin(), expect.end());
  const auto last = this->convex_hull();

  this->check_hull(last, expect);
  ASSERT_EQ(this->list.size(), 4U);

  // check for order
  auto it = this->list.begin();
  ASSERT_FLOAT_EQ(it->x, -6);
  ++it;  // node 4
  ASSERT_FLOAT_EQ(it->x, -2);
  ++it;  // node 3
  ASSERT_FLOAT_EQ(it->x, -1);
  ++it;  // node 1
  ASSERT_FLOAT_EQ(it->x, -5);
  ++it;  // node 2
  ASSERT_EQ(it, last);
}

// test that things get reordered to ccw
TYPED_TEST(TypedConvexHullTest, QuadHull)
{
  std::vector<TypeParam> data(
    {this->make(1, 1, 1), this->make(5, 1, 2), this->make(2, 6, 3), this->make(3, 3, 4),
     this->make(6, 5, 5)});
  std::vector<TypeParam> expect{{data[0], data[1], data[2], data[4]}};
  this->list.insert(this->list.begin(), data.begin(), data.end());

  const auto last = this->convex_hull();

  this->check_hull(last, expect);
  ASSERT_EQ(std::distance(this->list.cbegin(), last), 4U);

  // check for order
  auto it = this->list.begin();
  ASSERT_FLOAT_EQ(it->x, 1);
  ++it;  // node 1
  ASSERT_FLOAT_EQ(it->x, 5);
  ++it;  // node 2
  ASSERT_FLOAT_EQ(it->x, 6);
  ++it;  // node 4
  ASSERT_FLOAT_EQ(it->x, 2);
  ++it;  // node 3
  ASSERT_EQ(it, last);
}

// a ring plus a bunch of random stuff in the middle
TYPED_TEST(TypedConvexHullTest, Hull)
{
  const uint32_t HULL_SIZE = 13U;
  const uint32_t FUZZ_SIZE = 50U;
  const float32_t dth = 1.133729384F;  // some weird irrational(ish) number
  const float32_t r_hull = 20.0F;
  const float32_t r_fuzz = 10.0F;
  ASSERT_LT(r_fuzz, r_hull);

  std::vector<TypeParam> hull;

  uint32_t hull_pts = 0U;
  float32_t th = 0.0F;
  // hull part 1
  for (uint32_t idx = 0U; idx < HULL_SIZE / 3U; ++idx) {
    const auto pt = this->make(r_hull * cosf(th), r_hull * sinf(th), th);
    hull.push_back(pt);
    this->list.push_back(pt);
    th = fmodf(th + dth, 2.0F * 3.14159F);
    ++hull_pts;
  }

  // fuzz part 1
  uint32_t fuzz_pts = 0U;
  for (uint32_t idx = 0U; idx < FUZZ_SIZE / 2U; ++idx) {
    const auto pt = this->make(r_fuzz * cosf(th), r_fuzz * sinf(th), th);
    this->list.push_back(pt);
    th = fmodf(th + dth, 2.0F * 3.14159F);
    ++fuzz_pts;
  }

  // hull part 2
  for (uint32_t idx = 0U; idx < HULL_SIZE / 3U; ++idx) {
    const auto pt = this->make(r_hull * cosf(th), r_hull * sinf(th), th);
    hull.push_back(pt);
    this->list.push_back(pt);
    th = fmodf(th + dth, 2.0F * 3.14159F);
    ++hull_pts;
  }

  // fuzz part 2
  for (uint32_t idx = 0U; idx < FUZZ_SIZE - fuzz_pts; ++idx) {
    const auto pt = this->make(r_fuzz * cosf(th), r_fuzz * sinf(th), th);
    this->list.push_back(pt);
    th = fmodf(th + dth, 2.0F * 3.14159F);
  }

  // hull part 3
  for (uint32_t idx = 0U; idx < HULL_SIZE - hull_pts; ++idx) {
    const auto pt = this->make(r_hull * cosf(th), r_hull * sinf(th), th);
    hull.push_back(pt);
    this->list.push_back(pt);
    th = fmodf(th + dth, 2.0F * 3.14159F);
  }

  const auto last = this->convex_hull();

  this->check_hull(last, hull);
  ASSERT_EQ(std::distance(this->list.cbegin(), last), HULL_SIZE);
}

TYPED_TEST(TypedConvexHullTest, Collinear)
{
  std::vector<TypeParam> data(
    {this->make(0, 0, 1), this->make(1, 0, 2), this->make(2, 0, 3), this->make(0, 2, 4),
     this->make(1, 2, 8), this->make(2, 2, 7), this->make(1, 0, 6), this->make(1, 2, 5),
     this->make(1, 1, 0)});
  const std::vector<TypeParam> expect{{data[0], data[2], data[3], data[5]}};
  this->list.insert(this->list.begin(), data.begin(), data.end());

  const auto last = this->convex_hull();

  this->check_hull(last, expect);
  ASSERT_EQ(std::distance(this->list.cbegin(), last), 4U);

  // check for order
  auto it = this->list.begin();
  ASSERT_FLOAT_EQ(it->z, 1);
  ++it;  // node 1
  ASSERT_FLOAT_EQ(it->z, 3);
  ++it;  // node 1
  ASSERT_FLOAT_EQ(it->z, 7);
  ++it;  // node 2
  ASSERT_FLOAT_EQ(it->z, 4);
  ++it;  // node 3
  ASSERT_EQ(it, last);
}

// degenerate cases
TYPED_TEST(TypedConvexHullTest, OverlappingPoints)
{
  std::vector<TypeParam> data(
    {this->make(3, -1, 1), this->make(4, -2, 2), this->make(5, -7, 3), this->make(4, -2, 4),
     this->make(5, -7, 8), this->make(3, -1, 7), this->make(5, -7, 6), this->make(4, -2, 5),
     this->make(3, -1, 0)});
  const std::vector<TypeParam> expect{{data[0], data[1], data[2]}};
  this->list.insert(this->list.begin(), data.begin(), data.end());

  const auto last = this->convex_hull();

  ASSERT_EQ(std::distance(this->list.cbegin(), last), 3U);
  this->check_hull(last, expect, false);
}

TYPED_TEST(TypedConvexHullTest, Line)
{
  std::vector<TypeParam> data(
    {this->make(-3, 3, 1), this->make(-2, 2, 2), this->make(-1, 1, 3), this->make(-8, 8, 4),
     this->make(-6, 6, 8), this->make(-4, 4, 7), this->make(-10, 10, 6), this->make(-12, 12, 5),
     this->make(-11, 11, 0)});
  const std::vector<TypeParam> expect{{data[2], data[7]}};
  this->list.insert(this->list.begin(), data.begin(), data.end());

  const auto last = this->convex_hull();

  ASSERT_EQ(std::distance(this->list.cbegin(), last), 2U);
  this->check_hull(last, expect, false);

  // check for order: this part is a little loose
  auto it = this->list.begin();
  ASSERT_FLOAT_EQ(it->z, 5);
  ++it;  // node 8
  ASSERT_FLOAT_EQ(it->z, 3);
  ++it;  // node 3
  ASSERT_EQ(it, last);
}

/*
1
        4

      3
  2
*/
TYPED_TEST(TypedConvexHullTest, LowerHull)
{
  const std::vector<TypeParam> data({
    this->make(1, 3, 1),
    this->make(2, -2, 2),
    this->make(3, -1, 3),
    this->make(4, 1, 4),
  });
  this->list.insert(this->list.begin(), data.begin(), data.end());

  const auto last = this->convex_hull();

  ASSERT_EQ(std::distance(this->list.cbegin(), last), 4U);
  this->check_hull(last, data);

  // check for order: this part is a little loose
  auto it = this->list.begin();
  ASSERT_FLOAT_EQ(it->z, 1);
  ++it;
  ASSERT_FLOAT_EQ(it->z, 2);
  ++it;
  ASSERT_FLOAT_EQ(it->z, 3);
  ++it;
  ASSERT_FLOAT_EQ(it->z, 4);
  ++it;
  ASSERT_EQ(it, last);
}

// Ensure the leftmost item is properly shuffled
/*
         5
1 6
  2        4
        3
*/
TYPED_TEST(TypedConvexHullTest, Root)
{
  const std::vector<TypeParam> data({
    this->make(0, 0, 1),
    this->make(1, -1, 2),
    this->make(3, -2, 3),
    this->make(4, 0, 4),
    this->make(3, 1, 5),
    this->make(1, 0, 6),
  });
  const std::vector<TypeParam> expect{{data[0], data[1], data[2], data[3], data[4]}};
  this->list.insert(this->list.begin(), data.begin(), data.end());

  const auto last = this->convex_hull();

  ASSERT_EQ(std::distance(this->list.cbegin(), last), 5);
  this->check_hull(last, expect);

  auto it = this->list.begin();
  ASSERT_FLOAT_EQ(it->z, 1);
  ++it;
  ASSERT_FLOAT_EQ(it->z, 2);
  ++it;
  ASSERT_FLOAT_EQ(it->z, 3);
  ++it;
  ASSERT_FLOAT_EQ(it->z, 4);
  ++it;
  ASSERT_FLOAT_EQ(it->z, 5);
  ++it;
  ASSERT_EQ(it, last);
  EXPECT_NE(last, this->list.cend());
  EXPECT_EQ(last->z, 6);
}

// TODO(c.ho) random input, fuzzing, stress tests
