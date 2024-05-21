// Copyright 2024 TIER IV
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

#include <gtest/gtest.h>

// Assuming the parseRouteFile function is in 'RouteHandler.h'
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "planning_test_utils/mock_data_parser.hpp"

namespace test_utils
{
// Example YAML structure as a string for testing
const char g_complete_yaml[] = R"(
start_pose:
  position:
    x: 1.0
    y: 2.0
    z: 3.0
  orientation:
    x: 0.1
    y: 0.2
    z: 0.3
    w: 0.4
goal_pose:
  position:
    x: 4.0
    y: 5.0
    z: 6.0
  orientation:
    x: 0.5
    y: 0.6
    z: 0.7
    w: 0.8
segments:
- preferred_primitive:
    id: 11
    primitive_type: ''
  primitives:
    - id: 22
      primitive_type: lane
    - id: 33
      primitive_type: lane
)";

TEST(ParseFunctions, CompleteYAMLTest)
{
  YAML::Node config = YAML::Load(g_complete_yaml);

  // Test parsing of start_pose and goal_pose
  Pose start_pose = parse_pose(config["start_pose"]);
  Pose goal_pose = parse_pose(config["goal_pose"]);

  EXPECT_DOUBLE_EQ(start_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(start_pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(start_pose.position.z, 3.0);

  EXPECT_DOUBLE_EQ(start_pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(start_pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(start_pose.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(start_pose.orientation.w, 0.4);

  EXPECT_DOUBLE_EQ(goal_pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(goal_pose.position.y, 5.0);
  EXPECT_DOUBLE_EQ(goal_pose.position.z, 6.0);
  EXPECT_DOUBLE_EQ(goal_pose.orientation.x, 0.5);
  EXPECT_DOUBLE_EQ(goal_pose.orientation.y, 0.6);
  EXPECT_DOUBLE_EQ(goal_pose.orientation.z, 0.7);
  EXPECT_DOUBLE_EQ(goal_pose.orientation.w, 0.8);

  // Test parsing of segments
  std::vector<LaneletSegment> segments = parse_segments(config["segments"]);
  ASSERT_EQ(
    segments.size(), uint64_t(1));  // Assuming only one segment in the provided YAML for this test

  const auto & segment0 = segments[0];
  EXPECT_EQ(segment0.preferred_primitive.id, 11);
  EXPECT_EQ(segment0.primitives.size(), uint64_t(2));
  EXPECT_EQ(segment0.primitives[0].id, 22);
  EXPECT_EQ(segment0.primitives[0].primitive_type, "lane");
  EXPECT_EQ(segment0.primitives[1].id, 33);
  EXPECT_EQ(segment0.primitives[1].primitive_type, "lane");
}

TEST(ParseFunction, CompleteFromFilename)
{
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto parser_test_route =
    planning_test_utils_dir + "/test_data/lanelet_route_parser_test.yaml";

  const auto lanelet_route = parse_lanelet_route_file(parser_test_route);
  EXPECT_DOUBLE_EQ(lanelet_route.start_pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(lanelet_route.start_pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(lanelet_route.start_pose.position.z, 3.0);

  EXPECT_DOUBLE_EQ(lanelet_route.start_pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(lanelet_route.start_pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(lanelet_route.start_pose.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(lanelet_route.start_pose.orientation.w, 0.4);

  EXPECT_DOUBLE_EQ(lanelet_route.goal_pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(lanelet_route.goal_pose.position.y, 5.0);
  EXPECT_DOUBLE_EQ(lanelet_route.goal_pose.position.z, 6.0);
  EXPECT_DOUBLE_EQ(lanelet_route.goal_pose.orientation.x, 0.5);
  EXPECT_DOUBLE_EQ(lanelet_route.goal_pose.orientation.y, 0.6);
  EXPECT_DOUBLE_EQ(lanelet_route.goal_pose.orientation.z, 0.7);
  EXPECT_DOUBLE_EQ(lanelet_route.goal_pose.orientation.w, 0.8);

  ASSERT_EQ(
    lanelet_route.segments.size(),
    uint64_t(2));  // Assuming only one segment in the provided YAML for this test
  const auto & segment1 = lanelet_route.segments[1];
  EXPECT_EQ(segment1.preferred_primitive.id, 44);
  EXPECT_EQ(segment1.primitives.size(), uint64_t(4));
  EXPECT_EQ(segment1.primitives[0].id, 55);
  EXPECT_EQ(segment1.primitives[0].primitive_type, "lane");
  EXPECT_EQ(segment1.primitives[1].id, 66);
  EXPECT_EQ(segment1.primitives[1].primitive_type, "lane");
  EXPECT_EQ(segment1.primitives[2].id, 77);
  EXPECT_EQ(segment1.primitives[2].primitive_type, "lane");
  EXPECT_EQ(segment1.primitives[3].id, 88);
  EXPECT_EQ(segment1.primitives[3].primitive_type, "lane");
}
}  // namespace test_utils
