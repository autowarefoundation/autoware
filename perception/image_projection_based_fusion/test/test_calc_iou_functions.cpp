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

#include "autoware/image_projection_based_fusion/utils/geometry.hpp"

#include <gtest/gtest.h>

using autoware::image_projection_based_fusion::calcIoU;
using autoware::image_projection_based_fusion::calcIoUX;
using autoware::image_projection_based_fusion::calcIoUY;

TEST(GeometryTest, CalcIoU)
{
  sensor_msgs::msg::RegionOfInterest roi1, roi2;

  // Overlapping ROIs
  roi1.x_offset = 0;
  roi1.y_offset = 0;
  roi1.width = 4;
  roi1.height = 4;

  roi2.x_offset = 2;
  roi2.y_offset = 2;
  roi2.width = 4;
  roi2.height = 4;

  double iou = calcIoU(roi1, roi2);
  EXPECT_NEAR(iou, 1.0 / 7.0, 1e-6);

  // Non-overlapping ROIs
  roi2.x_offset = 5;
  roi2.y_offset = 5;

  iou = calcIoU(roi1, roi2);
  EXPECT_EQ(iou, 0.0);

  // Zero area ROI
  roi1.width = 0;
  roi1.height = 0;

  iou = calcIoU(roi1, roi2);
  EXPECT_EQ(iou, 0.0);
}

TEST(GeometryTest, CalcIoUX)
{
  sensor_msgs::msg::RegionOfInterest roi1, roi2;

  // Overlapping ROIs on x-axis
  roi1.x_offset = 0;
  roi1.y_offset = 0;
  roi1.width = 4;
  roi1.height = 4;

  roi2.x_offset = 2;
  roi2.y_offset = 0;
  roi2.width = 4;
  roi2.height = 4;

  double iou_x = calcIoUX(roi1, roi2);
  EXPECT_NEAR(iou_x, 2.0 / 6.0, 1e-6);

  // Non-overlapping ROIs on x-axis
  roi2.x_offset = 5;

  iou_x = calcIoUX(roi1, roi2);
  EXPECT_EQ(iou_x, 0.0);

  // Zero width ROI
  roi1.width = 0;

  iou_x = calcIoUX(roi1, roi2);
  EXPECT_EQ(iou_x, 0.0);
}

TEST(GeometryTest, CalcIoUY)
{
  sensor_msgs::msg::RegionOfInterest roi1, roi2;

  // Overlapping ROIs on y-axis
  roi1.x_offset = 0;
  roi1.y_offset = 0;
  roi1.width = 4;
  roi1.height = 4;

  roi2.x_offset = 0;
  roi2.y_offset = 2;
  roi2.width = 4;
  roi2.height = 4;

  double iou_y = calcIoUY(roi1, roi2);
  EXPECT_NEAR(iou_y, 2.0 / 6.0, 1e-6);

  // Non-overlapping ROIs on y-axis
  roi2.y_offset = 5;

  iou_y = calcIoUY(roi1, roi2);
  EXPECT_EQ(iou_y, 0.0);

  // Zero height ROI
  roi1.height = 0;

  iou_y = calcIoUY(roi1, roi2);
  EXPECT_EQ(iou_y, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
