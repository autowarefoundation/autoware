// Copyright 2023 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/utils.hpp"

#include <gmock/gmock.h>

TEST(CylinderAndBoxOverlapExists, NoOverlap)
{
  // Cylinder: center = (5, 0), radius = 4 - 0.1
  // Box: p_min = (-1, -1, -1), p_max = (1, 1, 1)
  double center_x = 5.0;
  double center_y = 0.0;

  const double radius = 4.0 - 0.1;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -1.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = 1.0;

  bool result = cylinderAndBoxOverlapExists(center_x, center_y, radius, p_min, p_max);
  EXPECT_FALSE(result);
}

TEST(CylinderAndBoxOverlapExists, Overlap1)
{
  // Cylinder: center = (0, 0), radius = 5
  // Box: p_min = (-1, -1, -1), p_max = (1, 1, 1)
  double center_x = 0.0;
  double center_y = 0.0;
  const double radius = 5.0;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -1.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = 1.0;

  bool result = cylinderAndBoxOverlapExists(center_x, center_y, radius, p_min, p_max);
  EXPECT_TRUE(result);
}

TEST(CylinderAndBoxOverlapExists, Overlap2)
{
  // Cylinder: center = (0, 0), radius = 5
  // Box: p_min = (-1, -1, -100), p_max = (1, 1, -99)
  double center_x = 0.0;
  double center_y = 0.0;
  const double radius = 5.0;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -100.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = -99.0;

  bool result = cylinderAndBoxOverlapExists(center_x, center_y, radius, p_min, p_max);
  EXPECT_TRUE(result);
}
