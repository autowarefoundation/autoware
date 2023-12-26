// Copyright 2021 The Autoware Foundation
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

#include <autoware_auto_perception_msgs/msg/point_clusters.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

#include <utility>
#include <vector>

using autoware::common::geometry::point_adapter::xr_;
using autoware::common::geometry::point_adapter::yr_;

// Helper function for adding new points
template <typename T>
T make_points(const float x, const float y)
{
  T ret;
  xr_(ret) = x;
  yr_(ret) = y;
  return ret;
}

// PointTypes to be tested
using PointTypes =
  ::testing::Types<geometry_msgs::msg::Point32, autoware::common::types::PointXYZIF>;

// Wrapper function for stubbing output of
// autoware::common::geometry::check_point_position_to_line_2d
template <typename T>
int point_position_checker(const T & p1, const T & p2, const T & q)
{
  auto result = autoware::common::geometry::check_point_position_to_line_2d(p1, p2, q);
  if (result > 0.0F) {
    return 1;
  } else if (result < 0.0F) {
    return -1;
  }
  return result;
}

// Templated struct defining the function parameters for
// autoware::common::geometry::check_point_position_to_line_2d and input_output vector for
// assertion of return values from the function
template <typename T>
struct PointPositionToLine : public ::testing::Test
{
  struct Parameters
  {
    T p1;
    T p2;
    T q;
  };
  static std::vector<std::pair<Parameters, int>> input_output;
};

TYPED_TEST_SUITE_P(PointPositionToLine);

template <typename T>
std::vector<std::pair<typename PointPositionToLine<T>::Parameters, int>>
  PointPositionToLine<T>::input_output{
    {{make_points<T>(0.0F, 0.0F), make_points<T>(-1.0F, 1.0F), make_points<T>(1.0F, 5.0F)}, -1},
    {{make_points<T>(0.0F, 0.0F), make_points<T>(-1.0F, 1.0F), make_points<T>(-1.0F, 0.5F)}, 1},
    // Check point on the line
    {{make_points<T>(0.0F, 0.0F), make_points<T>(-1.0F, 1.0F), make_points<T>(-2.0F, 2.0F)}, 0},
  };

TYPED_TEST_P(PointPositionToLine, PointPositionToLineTest)
{
  for (size_t i = 0; i < PointPositionToLine<TypeParam>::input_output.size(); ++i) {
    const auto & input = PointPositionToLine<TypeParam>::input_output[i].first;
    EXPECT_EQ(
      point_position_checker(input.p1, input.p2, input.q),
      PointPositionToLine<TypeParam>::input_output[i].second)
      << "Index " << i;
  }
}

REGISTER_TYPED_TEST_SUITE_P(PointPositionToLine, PointPositionToLineTest);
// cppcheck-suppress syntaxError
INSTANTIATE_TYPED_TEST_SUITE_P(Test, PointPositionToLine, PointTypes, );

/////////////////////////////////////////////////////////////////////////////////////

// Templated struct defining the function parameters for
// autoware::common::geometry::is_point_inside_polygon_2d and input_output vector for
// assertion of return values from the function
template <typename T>
struct InsidePolygon : public ::testing::Test
{
  struct Parameters
  {
    std::vector<T> polygon;
    T q;
  };
  static std::vector<std::pair<Parameters, bool>> input_output;
};

TYPED_TEST_SUITE_P(InsidePolygon);

template <typename T>
std::vector<std::pair<typename InsidePolygon<T>::Parameters, bool>> InsidePolygon<T>::input_output{
  // point inside the rectangle
  {{{make_points<T>(0.0F, 0.0F), make_points<T>(1.0F, 1.0F), make_points<T>(0.5F, 1.5F),
     make_points<T>(-0.5F, 0.5F)},
    make_points<T>(0.F, 0.5F)},
   true},
  // point below the rectangle
  {{{make_points<T>(0.0F, 0.0F), make_points<T>(1.0F, 1.0F), make_points<T>(0.5F, 1.5F),
     make_points<T>(-0.5F, 0.5F)},
    make_points<T>(0.5F, 0.F)},
   false},
  // point above the rectangle
  {{{make_points<T>(0.0F, 0.0F), make_points<T>(1.0F, 1.0F), make_points<T>(0.5F, 1.5F),
     make_points<T>(-0.5F, 0.5F)},
    make_points<T>(0.5F, 1.75F)},
   false},
  // point on the rectangle
  {{{make_points<T>(0.0F, 0.0F), make_points<T>(1.0F, 1.0F), make_points<T>(0.5F, 1.5F),
     make_points<T>(-0.5F, 0.5F)},
    make_points<T>(0.5F, 0.5F)},
   true},
};

TYPED_TEST_P(InsidePolygon, InsidePolygonTest)
{
  for (size_t i = 0; i < InsidePolygon<TypeParam>::input_output.size(); ++i) {
    const auto & input = InsidePolygon<TypeParam>::input_output[i].first;
    EXPECT_EQ(
      autoware::common::geometry::is_point_inside_polygon_2d(
        input.polygon.begin(), input.polygon.end(), input.q),
      InsidePolygon<TypeParam>::input_output[i].second)
      << "Index " << i;
  }
}

REGISTER_TYPED_TEST_SUITE_P(InsidePolygon, InsidePolygonTest);
// cppcheck-suppress syntaxError
INSTANTIATE_TYPED_TEST_SUITE_P(Test, InsidePolygon, PointTypes, );

TEST(ordered_check, basic)
{
  // CW
  std::vector<autoware_auto_perception_msgs::msg::PointXYZIF> points_list = {
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(8.0, 4.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(9.0, 1.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(3.0, 2.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(2.0, 5.0)};
  EXPECT_TRUE(autoware::common::geometry::all_ordered(points_list.begin(), points_list.end()));

  // CCW
  points_list = {
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(2.0, 5.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(3.0, 2.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(9.0, 1.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(8.0, 4.0)};
  EXPECT_TRUE(autoware::common::geometry::all_ordered(points_list.begin(), points_list.end()));

  // Unordered
  points_list = {
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(2.0, 5.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(3.0, 2.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(8.0, 4.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(9.0, 1.0)};
  EXPECT_FALSE(autoware::common::geometry::all_ordered(points_list.begin(), points_list.end()));

  // Unordered
  points_list = {
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(0.0, 0.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(1.0, 1.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(1.0, 0.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(2.0, 1.0)};
  EXPECT_FALSE(autoware::common::geometry::all_ordered(points_list.begin(), points_list.end()));

  // Collinearity
  points_list = {
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(2.0, 2.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(4.0, 4.0),
    make_points<autoware_auto_perception_msgs::msg::PointXYZIF>(6.0, 6.0)};
  EXPECT_TRUE(autoware::common::geometry::all_ordered(points_list.begin(), points_list.end()));
}
