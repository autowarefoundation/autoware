// Copyright 2022 The Autoware Contributors
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

TEST(SphereAndBoxOverlapExists, NoOverlap1)
{
  // Sphere: center = (5, 0, 0), radius = 4 - 0.1
  // Box: p_min = (-1, -1, -1), p_max = (1, 1, 1)
  geometry_msgs::msg::Point center;
  center.x = 5.0;
  center.y = 0.0;
  center.z = 0.0;

  const double radius = 4.0 - 0.1;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -1.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = 1.0;

  bool result = sphereAndBoxOverlapExists(center, radius, p_min, p_max);
  EXPECT_FALSE(result);
}

TEST(SphereAndBoxOverlapExists, NoOverlap2)
{
  // Sphere: center = (2, 2, 0), radius = sqrt(2) - 0.1
  // Box: p_min = (-1, -1, -1), p_max = (1, 1, 1)
  geometry_msgs::msg::Point center;
  center.x = 2.0;
  center.y = 2.0;
  center.z = 0.0;

  const double radius = std::sqrt(2) - 0.1;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -1.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = 1.0;

  bool result = sphereAndBoxOverlapExists(center, radius, p_min, p_max);
  EXPECT_FALSE(result);
}

TEST(SphereAndBoxOverlapExists, Overlap1)
{
  // Sphere: center = (0, 0, 0), radius = 0.5
  // Box: p_min = (-1, -1, -1), p_max = (1, 1, 1)
  geometry_msgs::msg::Point center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;

  const double radius = 0.5;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -1.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = 1.0;

  bool result = sphereAndBoxOverlapExists(center, radius, p_min, p_max);
  EXPECT_TRUE(result);
}

TEST(SphereAndBoxOverlapExists, Overlap2)
{
  // Sphere: center = (0, 0, 5), radius = 4 + 0.1
  // Box: p_min = (-1, -1, -1), p_max = (1, 1, 1)
  geometry_msgs::msg::Point center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 5.0;

  const double radius = 4 + 0.1;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -1.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = 1.0;

  bool result = sphereAndBoxOverlapExists(center, radius, p_min, p_max);
  EXPECT_TRUE(result);
}

TEST(SphereAndBoxOverlapExists, Overlap3)
{
  // Sphere: center = (2, 2, 2), radius = sqrt(3) + 0.1
  // Box: p_min = (-1, -1, -1), p_max = (1, 1, 1)
  geometry_msgs::msg::Point center;
  center.x = 2.0;
  center.y = 2.0;
  center.z = 2.0;

  const double radius = std::sqrt(3) + 0.1;

  pcl::PointXYZ p_min;
  p_min.x = -1.0;
  p_min.y = -1.0;
  p_min.z = -1.0;

  pcl::PointXYZ p_max;
  p_max.x = 1.0;
  p_max.y = 1.0;
  p_max.z = 1.0;

  bool result = sphereAndBoxOverlapExists(center, radius, p_min, p_max);
  EXPECT_TRUE(result);
}
