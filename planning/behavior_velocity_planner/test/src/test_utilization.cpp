// Copyright 2021 Tier IV, Inc.
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

#include "utils.hpp"

#include <utilization/boost_geometry_helper.hpp>
#include <utilization/util.hpp>

#include <gtest/gtest.h>

TEST(to_footprint_polygon, nominal)
{
  using behavior_velocity_planner::planning_utils::toFootprintPolygon;
  autoware_auto_perception_msgs::msg::PredictedObject obj = test::generatePredictedObject(0.0);
  auto poly = toFootprintPolygon(obj);
  EXPECT_TRUE(true);
}

TEST(is_ahead_of, nominal)
{
  using behavior_velocity_planner::planning_utils::isAheadOf;
  geometry_msgs::msg::Pose target = test::generatePose(0);
  geometry_msgs::msg::Pose origin = test::generatePose(1);
  bool is_ahead = isAheadOf(target, origin);
  EXPECT_FALSE(is_ahead);
  target = test::generatePose(2);
  is_ahead = isAheadOf(target, origin);
  EXPECT_TRUE(is_ahead);
}
