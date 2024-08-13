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

#include "../src/filter_predicted_objects.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/LineString.h>

TEST(TestCollisionDistance, CutPredictedPathBeyondLine)
{
  using autoware::motion_velocity_planner::out_of_lane::cut_predicted_path_beyond_line;
  autoware_perception_msgs::msg::PredictedPath predicted_path;
  autoware::universe_utils::LineString2d stop_line;
  double object_front_overhang = 0.0;
  const auto eps = 1e-9;

  EXPECT_NO_THROW(cut_predicted_path_beyond_line(predicted_path, stop_line, object_front_overhang));

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  predicted_path.path.push_back(pose);
  pose.position.y = 1.0;
  predicted_path.path.push_back(pose);
  pose.position.y = 2.0;
  predicted_path.path.push_back(pose);
  pose.position.y = 3.0;
  predicted_path.path.push_back(pose);

  cut_predicted_path_beyond_line(predicted_path, stop_line, object_front_overhang);
  EXPECT_EQ(predicted_path.path.size(), 4UL);
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    EXPECT_EQ(predicted_path.path[i].position.x, 0.0);
    EXPECT_NEAR(predicted_path.path[i].position.y, static_cast<double>(i), eps);
  }
  stop_line = {{-1.0, 2.0}, {1.0, 2.0}};
  cut_predicted_path_beyond_line(predicted_path, stop_line, object_front_overhang);
  EXPECT_EQ(predicted_path.path.size(), 2UL);
  for (auto i = 0UL; i < predicted_path.path.size(); ++i) {
    EXPECT_EQ(predicted_path.path[i].position.x, 0.0);
    EXPECT_NEAR(predicted_path.path[i].position.y, static_cast<double>(i), eps);
  }
}
