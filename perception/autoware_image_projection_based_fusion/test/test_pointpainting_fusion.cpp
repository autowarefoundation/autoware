// Copyright 2024 TIER IV, Inc.
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

#include <autoware/image_projection_based_fusion/pointpainting_fusion/node.hpp>

#include <gtest/gtest.h>

sensor_msgs::msg::RegionOfInterest createRoi(
  const int32_t x_offset, const int32_t y_offset, const int32_t width, const int32_t height)
{
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = x_offset;
  roi.y_offset = y_offset;
  roi.width = width;
  roi.height = height;
  return roi;
}

TEST(isInsideBboxTest, Inside)
{
  const sensor_msgs::msg::RegionOfInterest roi = createRoi(20, 20, 10, 10);
  bool result = autoware::image_projection_based_fusion::isInsideBbox(25.0, 25.0, roi, 1.0);
  EXPECT_TRUE(result);
}

TEST(isInsideBboxTest, Border)
{
  const sensor_msgs::msg::RegionOfInterest roi = createRoi(20, 20, 10, 10);
  bool result = autoware::image_projection_based_fusion::isInsideBbox(20.0, 30.0, roi, 1.0);
  EXPECT_TRUE(result);
}

TEST(isInsideBboxTest, Outside)
{
  const sensor_msgs::msg::RegionOfInterest roi = createRoi(20, 20, 10, 10);
  bool result = autoware::image_projection_based_fusion::isInsideBbox(15.0, 15.0, roi, 1.0);
  EXPECT_FALSE(result);
}

TEST(isInsideBboxTest, Zero)
{
  const sensor_msgs::msg::RegionOfInterest roi = createRoi(0, 0, 0, 0);
  bool result = autoware::image_projection_based_fusion::isInsideBbox(0.0, 0.0, roi, 1.0);
  EXPECT_TRUE(result);
}
