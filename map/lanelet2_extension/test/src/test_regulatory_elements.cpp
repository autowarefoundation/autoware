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

#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <math.h>

#include <vector>

using lanelet::LineString3d;
using lanelet::LineStringOrPolygon3d;
using lanelet::Point3d;
using lanelet::Points3d;
using lanelet::utils::getId;

namespace
{
template <typename T>
std::vector<T> convertToVector(T item)
{
  std::vector<T> vector = {item};
  return vector;
}
}  // namespace

class TestSuite : public ::testing::Test
{
public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST(TestSuite, FactoryConstructsTrafficLight)
{
  Point3d p1, p2, p3, p4, p5, p6, p7;
  LineStringOrPolygon3d traffic_light_base;
  LineString3d traffic_light_bulbs, stop_line;

  p1 = Point3d(getId(), 0., 1., 4.);
  p2 = Point3d(getId(), 1., 1., 4.);

  p3 = Point3d(getId(), 0., 1., 4.5);
  p4 = Point3d(getId(), 0.5, 1., 4.5);
  p5 = Point3d(getId(), 1., 1., 4.5);

  p6 = Point3d(getId(), 0., 0., 0.);
  p7 = Point3d(getId(), 1., 0., 0.);

  Points3d base = {p1, p2};
  Points3d bulbs = {p3, p4, p5};
  Points3d stop = {p6, p7};

  traffic_light_base = LineString3d(getId(), base);
  traffic_light_bulbs = LineString3d(getId(), bulbs);
  stop_line = LineString3d(getId(), stop);

  auto tl = lanelet::autoware::AutowareTrafficLight::make(
    getId(), lanelet::AttributeMap(), convertToVector(traffic_light_base), stop_line,
    convertToVector(traffic_light_bulbs));

  auto factoryTl = lanelet::RegulatoryElementFactory::create(
    tl->attribute(lanelet::AttributeName::Subtype).value(),
    std::const_pointer_cast<lanelet::RegulatoryElementData>(tl->constData()));
  EXPECT_TRUE(!!std::dynamic_pointer_cast<lanelet::TrafficLight>(factoryTl));
}

TEST(TestSuite, TrafficLightWorksAsExpected)
{  // NOLINT
  Point3d p1, p2, p3, p4, p5, p6, p7;
  LineStringOrPolygon3d traffic_light_base, traffic_light_base2;
  LineString3d traffic_light_bulbs, traffic_light_bulbs2, stop_line;

  p1 = Point3d(getId(), 0., 1., 4.);
  p2 = Point3d(getId(), 1., 1., 4.);

  p3 = Point3d(getId(), 0., 1., 4.5);
  p4 = Point3d(getId(), 0.5, 1., 4.5);
  p5 = Point3d(getId(), 1., 1., 4.5);

  p6 = Point3d(getId(), 0., 0., 0.);
  p7 = Point3d(getId(), 1., 0., 0.);

  Points3d base = {p1, p2};
  Points3d bulbs = {p3, p4, p5};
  Points3d stop = {p6, p7};

  traffic_light_base = {LineString3d(getId(), base)};
  traffic_light_base2 = {LineString3d(getId(), base)};
  traffic_light_bulbs = {LineString3d(getId(), bulbs)};
  traffic_light_bulbs2 = {LineString3d(getId(), bulbs)};
  stop_line = LineString3d(getId(), stop);

  auto tl = lanelet::autoware::AutowareTrafficLight::make(
    getId(), lanelet::AttributeMap(), convertToVector(traffic_light_base), stop_line,
    convertToVector(traffic_light_bulbs));
  tl->setStopLine(stop_line);
  EXPECT_EQ(stop_line, tl->stopLine());
  tl->addTrafficLight(traffic_light_base2);
  EXPECT_EQ(2ul, tl->trafficLights().size());
  tl->addLightBulbs(traffic_light_bulbs2);
  EXPECT_EQ(2ul, tl->lightBulbs().size());
  tl->removeLightBulbs(traffic_light_bulbs);
  EXPECT_EQ(1ul, tl->lightBulbs().size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
