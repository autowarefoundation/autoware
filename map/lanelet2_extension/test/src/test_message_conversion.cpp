// Copyright 2015-2019 Autoware Foundation
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
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"

#include <gtest/gtest.h>
#include <math.h>

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::utils::getId;
using lanelet::utils::conversion::toGeomMsgPt;

class TestSuite : public ::testing::Test
{
public:
  TestSuite() : single_lanelet_map_ptr(new lanelet::LaneletMap())
  {
    Point3d p1, p2, p3, p4, p5, p6, p7;
    LineString3d traffic_light_base, traffic_light_bulbs, stop_line;

    p1 = Point3d(getId(), 0., 0., 0.);
    p2 = Point3d(getId(), 0., 1., 0.);

    p3 = Point3d(getId(), 1., 0., 0.);
    p4 = Point3d(getId(), 1., 1., 0.);

    LineString3d ls_left(getId(), {p1, p2});   // NOLINT
    LineString3d ls_right(getId(), {p3, p4});  // NOLINT

    Lanelet lanelet(getId(), ls_left, ls_right);

    single_lanelet_map_ptr->add(lanelet);
  }
  ~TestSuite() {}
  lanelet::LaneletMapPtr single_lanelet_map_ptr;

private:
};

TEST_F(TestSuite, BinMsgConversion)
{
  autoware_auto_mapping_msgs::msg::HADMapBin bin_msg;
  lanelet::LaneletMapPtr regenerated_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::toBinMsg(single_lanelet_map_ptr, &bin_msg);

  ASSERT_NE(0U, bin_msg.data.size()) << "converted bin message does not have any data";

  lanelet::utils::conversion::fromBinMsg(bin_msg, regenerated_map);

  auto original_lanelet = lanelet::utils::query::laneletLayer(single_lanelet_map_ptr);
  auto regenerated_lanelet = lanelet::utils::query::laneletLayer(regenerated_map);

  ASSERT_EQ(original_lanelet.front().id(), regenerated_lanelet.front().id())
    << "regenerated map has different id";
}

TEST_F(TestSuite, ToGeomMsgPt)
{
  Point3d lanelet_pt(getId(), -0.1, 0.2, 3.0);

  geometry_msgs::msg::Point32 geom_pt32;
  geom_pt32.x = -0.1;
  geom_pt32.y = 0.2;
  geom_pt32.z = 3.0;

  geometry_msgs::msg::Point geom_pt;
  toGeomMsgPt(geom_pt32, &geom_pt);
  ASSERT_FLOAT_EQ(geom_pt32.x, geom_pt.x)
    << " converted value is different from original geometry_msgs::msg::Point";
  ASSERT_FLOAT_EQ(geom_pt32.y, geom_pt.y)
    << " converted value is different from original geometry_msgs::msg::Point";
  ASSERT_FLOAT_EQ(geom_pt32.z, geom_pt.z)
    << " converted value is different from original geometry_msgs::msg::Point";

  geom_pt = toGeomMsgPt(geom_pt32);
  ASSERT_FLOAT_EQ(geom_pt32.x, geom_pt.x)
    << " converted value is different from original geometry_msgs::msg::Point";
  ASSERT_FLOAT_EQ(geom_pt32.y, geom_pt.y)
    << " converted value is different from original geometry_msgs::msg::Point";
  ASSERT_FLOAT_EQ(geom_pt32.z, geom_pt.z)
    << " converted value is different from original geometry_msgs::msg::Point";

  toGeomMsgPt(lanelet_pt.basicPoint(), &geom_pt);
  ASSERT_DOUBLE_EQ(lanelet_pt.basicPoint().x(), geom_pt.x)
    << " converted value is different from original "
       "lanelet::basicPoint";
  ASSERT_DOUBLE_EQ(lanelet_pt.basicPoint().y(), geom_pt.y)
    << " converted value is different from original "
       "lanelet::basicPoint";
  ASSERT_DOUBLE_EQ(lanelet_pt.basicPoint().z(), geom_pt.z)
    << " converted value is different from original "
       "lanelet::basicPoint";

  geom_pt = toGeomMsgPt(lanelet_pt.basicPoint());
  ASSERT_DOUBLE_EQ(lanelet_pt.basicPoint().x(), geom_pt.x)
    << " converted value is different from original "
       "lanelet::basicPoint";
  ASSERT_DOUBLE_EQ(lanelet_pt.basicPoint().y(), geom_pt.y)
    << " converted value is different from original "
       "lanelet::basicPoint";
  ASSERT_DOUBLE_EQ(lanelet_pt.basicPoint().z(), geom_pt.z)
    << " converted value is different from original "
       "lanelet::basicPoint";

  toGeomMsgPt(lanelet_pt, &geom_pt);
  ASSERT_DOUBLE_EQ(lanelet_pt.x(), geom_pt.x)
    << " converted value is different from original lanelet::Point3d";
  ASSERT_DOUBLE_EQ(lanelet_pt.y(), geom_pt.y)
    << " converted value is different from original lanelet::Point3d";
  ASSERT_DOUBLE_EQ(lanelet_pt.z(), geom_pt.z)
    << " converted value is different from original lanelet::Point3d";

  geom_pt = toGeomMsgPt(lanelet_pt);
  ASSERT_DOUBLE_EQ(lanelet_pt.x(), geom_pt.x)
    << " converted value is different from original lanelet::Point3d";
  ASSERT_DOUBLE_EQ(lanelet_pt.y(), geom_pt.y)
    << " converted value is different from original lanelet::Point3d";
  ASSERT_DOUBLE_EQ(lanelet_pt.z(), geom_pt.z)
    << " converted value is different from original lanelet::Point3d";

  lanelet::ConstPoint2d point_2d = lanelet::utils::to2D(lanelet_pt);

  toGeomMsgPt(point_2d, &geom_pt);
  ASSERT_DOUBLE_EQ(point_2d.x(), geom_pt.x)
    << " converted value is different from original lanelet::Point2d";
  ASSERT_DOUBLE_EQ(point_2d.y(), geom_pt.y)
    << " converted value is different from original lanelet::Point2d";
  ASSERT_DOUBLE_EQ(0.0, geom_pt.z)
    << " converted value is different from original lanelet::Point2d";

  geom_pt = toGeomMsgPt(point_2d);
  ASSERT_DOUBLE_EQ(point_2d.x(), geom_pt.x)
    << " converted value is different from original lanelet::Point2d";
  ASSERT_DOUBLE_EQ(point_2d.y(), geom_pt.y)
    << " converted value is different from original lanelet::Point2d";
  ASSERT_DOUBLE_EQ(0.0, geom_pt.z)
    << " converted value is different from original lanelet::Point2d";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
