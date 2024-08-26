// Copyright 2023 TIER IV, Inc.
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

#include "gtest/gtest.h"
#include "traffic_light_utils/traffic_light_utils.hpp"

namespace traffic_light_utils
{

TEST(setSignalUnknown, set_signal_element)
{
  tier4_perception_msgs::msg::TrafficLight test_signal;
  tier4_perception_msgs::msg::TrafficLightElement element;
  element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
  element.shape = tier4_perception_msgs::msg::TrafficLightElement::CROSS;
  test_signal.elements.push_back(element);
  EXPECT_EQ(test_signal.elements[0].color, tier4_perception_msgs::msg::TrafficLightElement::RED);
  EXPECT_EQ(test_signal.elements[0].shape, tier4_perception_msgs::msg::TrafficLightElement::CROSS);
  setSignalUnknown(test_signal, 1.23f);
  EXPECT_EQ(
    test_signal.elements[0].color, tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN);
  EXPECT_EQ(
    test_signal.elements[0].shape, tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN);
  EXPECT_FLOAT_EQ(test_signal.elements[0].confidence, (float)1.23);
}

TEST(getTrafficLightCenter, get_signal)
{
  lanelet::LineString3d lineString;
  lanelet::Point3d p0(0, 0, 0, 0);
  lanelet::Point3d p1(1, 1, 1, 1);
  lanelet::Point3d p2(2, 2, 2, 2);
  lanelet::Point3d p3(3, 3, 3, 3);
  lineString.push_back(p0);
  lineString.push_back(p1);
  lineString.push_back(p2);
  lineString.push_back(p3);

  lanelet::ConstLineString3d test_light(lineString);
  EXPECT_FLOAT_EQ(getTrafficLightCenter(test_light).x(), (float)1.5);
  EXPECT_FLOAT_EQ(getTrafficLightCenter(test_light).y(), (float)1.5);
  EXPECT_FLOAT_EQ(getTrafficLightCenter(test_light).z(), (float)1.5);
}

}  // namespace traffic_light_utils
