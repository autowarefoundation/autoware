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

#include "lanelet2_extension/utility/query.hpp"

#include <gtest/gtest.h>
#include <math.h>

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::LineStringOrPolygon3d;
using lanelet::Point3d;
using lanelet::Points3d;
using lanelet::utils::getId;

class TestSuite : public ::testing::Test
{
public:
  TestSuite() : sample_map_ptr(new lanelet::LaneletMap())
  {  // NOLINT
    // create sample lanelets
    Point3d p1, p2, p3, p4;

    p1 = Point3d(getId(), 0., 0., 0.);
    p2 = Point3d(getId(), 0., 1., 0.);

    p3 = Point3d(getId(), 1., 0., 0.);
    p4 = Point3d(getId(), 1., 1., 0.);

    LineString3d ls_left(getId(), {p1, p2});   // NOLINT
    LineString3d ls_right(getId(), {p3, p4});  // NOLINT

    Lanelet road_lanelet(getId(), ls_left, ls_right);
    road_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Road;

    Lanelet crosswalk_lanelet(getId(), ls_left, ls_right);
    crosswalk_lanelet.attributes()[lanelet::AttributeName::Subtype] =
      lanelet::AttributeValueString::Crosswalk;

    // create sample traffic light
    Point3d p5, p6, p7, p8, p9, p10, p11, p12;
    LineString3d traffic_light_base, traffic_light_bulbs, stop_line;

    p6 = Point3d(getId(), 0., 1., 4.);
    p7 = Point3d(getId(), 1., 1., 4.);

    p8 = Point3d(getId(), 0., 1., 4.5);
    p9 = Point3d(getId(), 0.5, 1., 4.5);
    p10 = Point3d(getId(), 1., 1., 4.5);

    p11 = Point3d(getId(), 0., 0., 0.);
    p12 = Point3d(getId(), 1., 0., 0.);

    traffic_light_base = LineString3d(getId(), Points3d{p6, p7});        // NOLINT
    traffic_light_bulbs = LineString3d(getId(), Points3d{p8, p9, p10});  // NOLINT
    stop_line = LineString3d(getId(), Points3d{p11, p12});               // NOLINT

    auto tl = lanelet::autoware::AutowareTrafficLight::make(
      getId(), lanelet::AttributeMap(), {traffic_light_base}, stop_line,
      {traffic_light_bulbs});  // NOLINT

    road_lanelet.addRegulatoryElement(tl);

    // add items to map
    sample_map_ptr->add(road_lanelet);
    sample_map_ptr->add(crosswalk_lanelet);
  }
  ~TestSuite() {}

  lanelet::LaneletMapPtr sample_map_ptr;

private:
};

TEST_F(TestSuite, QueryLanelets)
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(sample_map_ptr);
  ASSERT_EQ(2U, all_lanelets.size()) << "failed to retrieve all lanelets";

  lanelet::ConstLanelets subtype_lanelets =
    lanelet::utils::query::subtypeLanelets(all_lanelets, lanelet::AttributeValueString::Road);
  ASSERT_EQ(1U, subtype_lanelets.size()) << "failed to retrieve road lanelet by subtypeLanelets";

  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  ASSERT_EQ(1U, road_lanelets.size()) << "failed to retrieve road lanelets";

  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  ASSERT_EQ(1U, crosswalk_lanelets.size()) << "failed to retrieve crosswalk lanelets";
}

TEST_F(TestSuite, QueryTrafficLights)
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(sample_map_ptr);

  auto traffic_lights = lanelet::utils::query::trafficLights(all_lanelets);
  ASSERT_EQ(1U, traffic_lights.size()) << "failed to retrieve traffic lights";

  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  ASSERT_EQ(1U, autoware_traffic_lights.size()) << "failed to retrieve autoware traffic lights";
}

TEST_F(TestSuite, QueryStopLine)
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(sample_map_ptr);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  auto stop_lines = lanelet::utils::query::stopLinesLanelets(all_lanelets);
  ASSERT_EQ(1U, stop_lines.size()) << "failed to retrieve stop lines from all lanelets";

  auto stop_lines2 = lanelet::utils::query::stopLinesLanelet(road_lanelets.front());
  ASSERT_EQ(1U, stop_lines2.size()) << "failed to retrieve stop lines from a lanelet";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
