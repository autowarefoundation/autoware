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

#include "lanelet2_extension/utility/utilities.hpp"

#include <gtest/gtest.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <map>

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::utils::getId;

class TestSuite : public ::testing::Test
{
public:
  TestSuite() : sample_map_ptr(new lanelet::LaneletMap())
  {  // NOLINT
    // create sample lanelets
    Point3d p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;

    p1 = Point3d(getId(), 0., 0., 0.);
    p2 = Point3d(getId(), 0., 1., 0.);
    p3 = Point3d(getId(), 1., 0., 0.);
    p4 = Point3d(getId(), 1., 1., 0.);

    LineString3d ls_left(getId(), {p1, p2});   // NOLINT
    LineString3d ls_right(getId(), {p3, p4});  // NOLINT

    p5 = Point3d(getId(), 0., 2., 0.);
    p6 = Point3d(getId(), 1., 2., 0.);

    LineString3d ls_left2(getId(), {p2, p5});   // NOLINT
    LineString3d ls_right2(getId(), {p4, p6});  // NOLINT

    p7 = Point3d(getId(), 0., 3., 0.);
    p8 = Point3d(getId(), 1., 3., 0.);

    LineString3d ls_left3(getId(), {p5, p7});   // NOLINT
    LineString3d ls_right3(getId(), {p6, p8});  // NOLINT

    p9 = Point3d(getId(), 0., 1., 0.);
    p10 = Point3d(getId(), 1., 1., 0.);

    LineString3d ls_left4(getId(), {p9, p5});    // NOLINT
    LineString3d ls_right4(getId(), {p10, p6});  // NOLINT

    road_lanelet = Lanelet(getId(), ls_left, ls_right);
    road_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Road;

    next_lanelet = Lanelet(getId(), ls_left2, ls_right2);
    next_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Road;

    next_lanelet2 = Lanelet(getId(), ls_left3, ls_right3);
    next_lanelet2.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Road;

    merging_lanelet = Lanelet(getId(), ls_left4, ls_right4);
    merging_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Road;

    sample_map_ptr->add(road_lanelet);
    sample_map_ptr->add(next_lanelet);
    sample_map_ptr->add(next_lanelet2);
    sample_map_ptr->add(merging_lanelet);
  }
  ~TestSuite() {}

  lanelet::LaneletMapPtr sample_map_ptr;
  Lanelet road_lanelet;
  Lanelet next_lanelet;
  Lanelet next_lanelet2;
  Lanelet merging_lanelet;

private:
};

TEST_F(TestSuite, OverwriteLaneletsCenterline)
{
  double resolution = 5.0;
  bool force_overwrite = false;
  lanelet::utils::overwriteLaneletsCenterline(sample_map_ptr, resolution, force_overwrite);

  for (const auto & lanelet : sample_map_ptr->laneletLayer) {
    ASSERT_TRUE(lanelet.hasCustomCenterline()) << "failed to calculate fine centerline";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
