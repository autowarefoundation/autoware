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

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace bg = boost::geometry;

using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Point3d;

TEST(boost_geometry, boost_geometry_distance)
{
  {
    const Point2d p1(1.0, 2.0);
    const Point2d p2(2.0, 4.0);
    EXPECT_DOUBLE_EQ(bg::distance(p1, p2), std::sqrt(5));
  }

  {
    const Point3d p1(1.0, 2.0, 3.0);
    const Point3d p2(2.0, 4.0, 6.0);
    EXPECT_DOUBLE_EQ(bg::distance(p1, p2), std::sqrt(14));
  }
}

TEST(boost_geometry, to_3d)
{
  const Point2d p_2d(1.0, 2.0);
  const Point3d p_3d(1.0, 2.0, 3.0);
  EXPECT_TRUE(p_2d.to_3d(3.0) == p_3d);
}

TEST(boost_geometry, to_2d)
{
  const Point2d p_2d(1.0, 2.0);
  const Point3d p_3d(1.0, 2.0, 3.0);
  EXPECT_TRUE(p_3d.to_2d() == p_2d);
}

TEST(boost_geometry, toMsg)
{
  using tier4_autoware_utils::toMsg;

  {
    const Point3d p(1.0, 2.0, 3.0);
    const geometry_msgs::msg::Point p_msg = toMsg(p);

    EXPECT_DOUBLE_EQ(p_msg.x, 1.0);
    EXPECT_DOUBLE_EQ(p_msg.y, 2.0);
    EXPECT_DOUBLE_EQ(p_msg.z, 3.0);
  }
}

TEST(boost_geometry, fromMsg)
{
  using tier4_autoware_utils::fromMsg;

  geometry_msgs::msg::Point p_msg;
  p_msg.x = 1.0;
  p_msg.y = 2.0;
  p_msg.z = 3.0;

  const Point3d p = fromMsg(p_msg);

  EXPECT_DOUBLE_EQ(p.x(), 1.0);
  EXPECT_DOUBLE_EQ(p.y(), 2.0);
  EXPECT_DOUBLE_EQ(p.z(), 3.0);
}
