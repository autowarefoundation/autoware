// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/forward_projection.hpp"
#include "obstacle_velocity_limiter/types.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/io/wkt/write.hpp>

#include <geometry_msgs/msg/point.h>
#include <gtest/gtest.h>

#include <algorithm>

constexpr auto EPS = 1e-15;
constexpr auto EPS_APPROX = 1e-3;

TEST(TestForwardProjection, forwardSimulatedSegment)
{
  using obstacle_velocity_limiter::forwardSimulatedSegment;
  using obstacle_velocity_limiter::ProjectionParameters;
  using obstacle_velocity_limiter::segment_t;

  geometry_msgs::msg::Point point;
  point.x = 0.0;
  point.y = 0.0;
  ProjectionParameters params;
  params.model = ProjectionParameters::PARTICLE;

  const auto check_vector = [&](const auto expected_vector_length) {
    params.heading = 0.0;
    auto vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), expected_vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), 0.0);
    params.heading = M_PI_2;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), expected_vector_length);
    params.heading = M_PI_4;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_DOUBLE_EQ(vector.second.x(), std::sqrt(0.5) * expected_vector_length);
    EXPECT_DOUBLE_EQ(vector.second.y(), std::sqrt(0.5) * expected_vector_length);
    params.heading = -M_PI_2;
    vector = forwardSimulatedSegment(point, params);
    EXPECT_DOUBLE_EQ(vector.first.x(), point.x);
    EXPECT_DOUBLE_EQ(vector.first.y(), point.y);
    EXPECT_NEAR(vector.second.x(), 0.0, 1e-9);
    EXPECT_DOUBLE_EQ(vector.second.y(), -expected_vector_length);
  };

  // 0 velocity: whatever the duration the vector length is always = to extra_dist
  params.velocity = 0.0;

  params.duration = 0.0;
  params.extra_length = 0.0;
  check_vector(params.extra_length);

  params.duration = 5.0;
  params.extra_length = 2.0;
  check_vector(params.extra_length);

  params.duration = -5.0;
  params.extra_length = 3.5;
  check_vector(params.extra_length);

  // set non-zero velocities
  params.velocity = 1.0;

  params.duration = 1.0;
  params.extra_length = 0.0;
  check_vector(1.0 + params.extra_length);

  params.duration = 5.0;
  params.extra_length = 2.0;
  check_vector(5.0 + params.extra_length);

  params.duration = -5.0;
  params.extra_length = 3.5;
  check_vector(-5.0 + params.extra_length);
}

const auto point_in_polygon = [](const auto x, const auto y, const auto & polygon) {
  return std::find_if(polygon.outer().begin(), polygon.outer().end(), [=](const auto & pt) {
           return pt.x() == x && pt.y() == y;
         }) != polygon.outer().end();
};

TEST(TestForwardProjection, generateFootprint)
{
  using obstacle_velocity_limiter::generateFootprint;
  using obstacle_velocity_limiter::linestring_t;
  using obstacle_velocity_limiter::segment_t;

  auto footprint = generateFootprint(linestring_t{{0.0, 0.0}, {1.0, 0.0}}, 1.0);
  EXPECT_TRUE(point_in_polygon(0.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));
  footprint = generateFootprint(segment_t{{0.0, 0.0}, {1.0, 0.0}}, 1.0);
  EXPECT_TRUE(point_in_polygon(0.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.0, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));

  footprint = generateFootprint(linestring_t{{0.0, 0.0}, {0.0, -1.0}}, 0.5);
  EXPECT_TRUE(point_in_polygon(0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, -1.0, footprint));
  footprint = generateFootprint(segment_t{{0.0, 0.0}, {0.0, -1.0}}, 0.5);
  EXPECT_TRUE(point_in_polygon(0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, -1.0, footprint));

  footprint = generateFootprint(linestring_t{{-2.5, 5.0}, {2.5, 0.0}}, std::sqrt(2));
  EXPECT_TRUE(point_in_polygon(3.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-3.5, 4.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.5, 6.0, footprint));
  footprint = generateFootprint(segment_t{{-2.5, 5.0}, {2.5, 0.0}}, std::sqrt(2));
  EXPECT_TRUE(point_in_polygon(3.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.5, -1.0, footprint));
  EXPECT_TRUE(point_in_polygon(-3.5, 4.0, footprint));
  EXPECT_TRUE(point_in_polygon(-1.5, 6.0, footprint));
}

TEST(TestForwardProjection, generateFootprintMultiLinestrings)
{
  using obstacle_velocity_limiter::generateFootprint;
  using obstacle_velocity_limiter::linestring_t;
  using obstacle_velocity_limiter::multilinestring_t;

  auto footprint = generateFootprint(
    multilinestring_t{
      linestring_t{{0.0, 0.0}, {0.0, 1.0}}, linestring_t{{0.0, 0.0}, {0.8, 0.8}},
      linestring_t{{0.0, 0.0}, {1.0, 0.0}}},
    0.5);
  std::cout << boost::geometry::wkt(footprint) << std::endl;
  /*
  EXPECT_TRUE(point_in_polygon(-0.5, 0.0, footprint));
  EXPECT_TRUE(point_in_polygon(-0.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(0.5, 1.0, footprint));
  EXPECT_TRUE(point_in_polygon(1.0, -1.0, footprint));
  */
}
