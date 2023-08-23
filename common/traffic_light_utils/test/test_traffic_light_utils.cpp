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

TEST(isRoiValid, roi_validity)
{
  tier4_perception_msgs::msg::TrafficLightRoi test_roi;
  test_roi.roi.x_offset = 300;
  test_roi.roi.y_offset = 200;
  test_roi.roi.width = 340;
  test_roi.roi.height = 200;
  uint32_t img_width = 640;
  uint32_t img_heigh = 480;
  EXPECT_FALSE(isRoiValid(test_roi, img_width, img_heigh));
  test_roi.roi.width = 339;
  EXPECT_TRUE(isRoiValid(test_roi, img_width, img_heigh));
}

TEST(setRoiInvalid, set_roi_size)
{
  tier4_perception_msgs::msg::TrafficLightRoi test_roi;
  test_roi.roi.x_offset = 300;
  test_roi.roi.y_offset = 200;
  test_roi.roi.width = 300;
  test_roi.roi.height = 200;
  EXPECT_EQ(test_roi.roi.width, (uint32_t)300);
  EXPECT_EQ(test_roi.roi.height, (uint32_t)200);
  setRoiInvalid(test_roi);
  EXPECT_EQ(test_roi.roi.width, (uint32_t)0);
  EXPECT_EQ(test_roi.roi.height, (uint32_t)0);
}

TEST(isSignalUnknown, signal_element)
{
  tier4_perception_msgs::msg::TrafficSignal test_signal;
  tier4_perception_msgs::msg::TrafficLightElement element;
  element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
  element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
  test_signal.elements.push_back(element);
  EXPECT_TRUE(isSignalUnknown(test_signal));
  test_signal.elements[0].color = tier4_perception_msgs::msg::TrafficLightElement::RED;
  EXPECT_FALSE(isSignalUnknown(test_signal));
}

TEST(setSignalUnknown, set_signal_element)
{
  tier4_perception_msgs::msg::TrafficSignal test_signal;
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
