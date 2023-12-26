// Copyright 2021 Apex.AI, Inc.
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

#include "autoware_auto_geometry/common_2d.hpp"

#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <list>
#include <utility>
#include <vector>

template <typename DataStructure>
class AreaTest : public ::testing::Test
{
protected:
  DataStructure data_{};
  using Point = typename DataStructure::value_type;
  using Real = decltype(autoware::common::geometry::point_adapter::x_(std::declval<Point>()));

  auto area() { return autoware::common::geometry::area_checked_2d(data_.begin(), data_.end()); }

  void add_point(Real x, Real y)
  {
    namespace pa = autoware::common::geometry::point_adapter;
    Point p{};
    pa::xr_(p) = x;
    pa::yr_(p) = y;
    (void)data_.insert(data_.end(), p);
  }
};

// Data structures to test...
template <typename... Points>
using TestTypes_ = ::testing::Types<std::vector<Points>..., std::list<Points>...>;
// ... and point types to test
using TestTypes = TestTypes_<geometry_msgs::msg::Point32>;
// cppcheck-suppress syntaxError
TYPED_TEST_SUITE(AreaTest, TestTypes, );

// The empty set has zero area
TYPED_TEST(AreaTest, DegenerateZero)
{
  EXPECT_FLOAT_EQ(0.0, this->area());
}

// An individual point has zero area
TYPED_TEST(AreaTest, DegenerateOne)
{
  this->add_point(0.0, 0.0);
  EXPECT_FLOAT_EQ(0.0, this->area());
}

// An line segment has zero area
TYPED_TEST(AreaTest, DegenerateTwo)
{
  this->add_point(1.0, -1.0);
  this->add_point(-3.0, 2.0);
  EXPECT_FLOAT_EQ(0.0, this->area());
}

// Simple triangle
TYPED_TEST(AreaTest, Triangle)
{
  this->add_point(1.0, 0.0);
  this->add_point(3.0, 0.0);           // 2.0 wide
  this->add_point(2.0, 2.0);           // 2.0 tall
  EXPECT_FLOAT_EQ(2.0, this->area());  // A = (1/2) * b * h
}

// Rectangle is easy to do computational
TYPED_TEST(AreaTest, Rectangle)
{
  this->add_point(-5.0, -5.0);
  this->add_point(-2.0, -5.0);  // L = 3
  this->add_point(-2.0, -1.0);  // H = 4
  this->add_point(-5.0, -1.0);
  EXPECT_FLOAT_EQ(12.0, this->area());  // A = b * h
}

// Parallelogram is slightly less trivial than a rectangle
TYPED_TEST(AreaTest, Parallelogram)
{
  this->add_point(-5.0, 1.0);
  this->add_point(-2.0, 1.0);  // L = 3
  this->add_point(-1.0, 3.0);  // H = 2
  this->add_point(-4.0, 3.0);
  EXPECT_FLOAT_EQ(6.0, this->area());  // A = b * h
}

// Octagon is analytical and reasonably easy to build
TYPED_TEST(AreaTest, Octagon)
{
  const auto sq2 = std::sqrt(2.0);
  const auto a = 1.0;
  const auto a2 = a / 2.0;
  const auto b = (a + sq2) / 2.0;
  this->add_point(-a2, -b);
  this->add_point(a2, -b);
  this->add_point(b, -a2);
  this->add_point(b, a2);
  this->add_point(a2, b);
  this->add_point(-a2, b);
  this->add_point(-b, a2);
  this->add_point(-b, -a2);
  const auto expect = (2.0 * (1.0 + sq2)) * (a * a);
  EXPECT_FLOAT_EQ(expect, this->area());  // A = b * h
}

// Bad case
TYPED_TEST(AreaTest, NotCcw)
{
  this->add_point(0.0, 0.0);
  this->add_point(1.0, 1.0);
  this->add_point(1.0, 0.0);
  this->add_point(2.0, 1.0);
  EXPECT_THROW(this->area(), std::domain_error);
}
