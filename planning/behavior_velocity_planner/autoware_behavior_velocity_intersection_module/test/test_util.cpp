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

// TODO(Mamoru Sobue): create project include dir later
#include "../src/util.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

TEST(TestUtil, retrievePathsBackward)
{
  /*
    0 ----> 1 ----> 2 ----> 4 ----> 6
    ^        \              ^\
    |         \             | \
    |          3 ----> 5    |  \
    |                  |    <---7
    <------------------|
   */
  const std::vector<std::vector<bool>> adjacency = {
    {false, true /*1*/, false, false, false, false, false, false},       // 0
    {false, false, true /*2*/, true /*3*/, false, false, false, false},  // 1
    {false, false, false, false, true /*4*/, false, false, false},       // 2
    {false, false, false, false, false, true /*5*/, false, false},       // 3
    {false, false, false, false, false, false, true /*6*/, true /*7*/},  // 4
    {true /* 0*/, false, false, false, false, false, false, false},      // 5
    {false, false, false, false, false, false, false, false},            // 6
    {false, false, false, false, true /*4*/, false, false, false},       // 7
  };
  {
    const size_t src_ind = 6;
    std::vector<std::vector<size_t>> paths;
    autoware::behavior_velocity_planner::util::retrievePathsBackward(adjacency, src_ind, {}, paths);
    EXPECT_EQ(paths.size(), 1);
    EXPECT_EQ(paths.at(0).size(), 1);
    EXPECT_EQ(paths.at(0).at(0), 6);
  }
  {
    const size_t src_ind = 4;
    std::vector<std::vector<size_t>> paths;
    autoware::behavior_velocity_planner::util::retrievePathsBackward(adjacency, src_ind, {}, paths);
    EXPECT_EQ(paths.size(), 2);
    // 4 --> 6
    EXPECT_EQ(paths.at(0).size(), 2);
    EXPECT_EQ(paths.at(0).at(0), 4);
    EXPECT_EQ(paths.at(0).at(1), 6);
    // 4 --> 7
    EXPECT_EQ(paths.at(1).size(), 2);
    EXPECT_EQ(paths.at(1).at(0), 4);
    EXPECT_EQ(paths.at(1).at(1), 7);
  }
  {
    const size_t src_ind = 0;
    std::vector<std::vector<size_t>> paths;
    autoware::behavior_velocity_planner::util::retrievePathsBackward(adjacency, src_ind, {}, paths);
    EXPECT_EQ(paths.size(), 3);
    // 0 --> 1 --> 2 --> 4 --> 6
    EXPECT_EQ(paths.at(0).size(), 5);
    EXPECT_EQ(paths.at(0).at(0), 0);
    EXPECT_EQ(paths.at(0).at(1), 1);
    EXPECT_EQ(paths.at(0).at(2), 2);
    EXPECT_EQ(paths.at(0).at(3), 4);
    EXPECT_EQ(paths.at(0).at(4), 6);
    // 0 --> 1 --> 2 --> 4 --> 7
    EXPECT_EQ(paths.at(1).at(0), 0);
    EXPECT_EQ(paths.at(1).at(1), 1);
    EXPECT_EQ(paths.at(1).at(2), 2);
    EXPECT_EQ(paths.at(1).at(3), 4);
    EXPECT_EQ(paths.at(1).at(4), 7);
    // 0 --> 1 --> 3 --> 5
    EXPECT_EQ(paths.at(2).at(0), 0);
    EXPECT_EQ(paths.at(2).at(1), 1);
    EXPECT_EQ(paths.at(2).at(2), 3);
    EXPECT_EQ(paths.at(2).at(3), 5);
  }
}

/*
  TOOD(Mamoru Sobue): instantiating intersection_module and PlannerData is a messy
class TestWithMap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string intersection_map_file = autoware_test_utils::get_absolute_path_to_lanelet_map(
      "autoware_behavior_velocity_planner_common", "intersection/lanelet2_map.osm");
    const auto map_bin_msg = autoware_test_utils::make_map_bin_msg(intersection_map_file, 0.5);
    route_handler_ = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);
  }

private:
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
};

TEST_F(TestWithMap, mergeLaneletsByTopologicalSort)
{
}
*/
