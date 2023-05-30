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
#include "motion_utils/marker/virtual_wall_marker_creator.hpp"
namespace
{
constexpr auto wall_ns_suffix = "_virtual_wall";
constexpr auto text_ns_suffix = "_factor_text";

bool has_ns_id(
  const visualization_msgs::msg::MarkerArray & marker_array, const std::string & ns, const int id)
{
  return std::find_if(marker_array.markers.begin(), marker_array.markers.end(), [&](const auto m) {
           return m.id == id && m.ns == ns;
         }) != marker_array.markers.end();
}

bool has_ns_id(
  const visualization_msgs::msg::MarkerArray & marker_array, const std::string & ns,
  const int id_from, const int id_to)
{
  for (auto id = id_from; id <= id_to; ++id)
    if (!has_ns_id(marker_array, ns, id)) return false;
  return true;
}

TEST(VirtualWallMarkerCreator, oneWall)
{
  motion_utils::VirtualWall wall;
  motion_utils::VirtualWallMarkerCreator creator;
  wall.style = motion_utils::VirtualWallType::stop;
  wall.pose.position.x = 1.0;
  wall.pose.position.y = 2.0;
  creator.add_virtual_wall(wall);
  auto markers = creator.create_markers();
  ASSERT_EQ(markers.markers.size(), 2UL);
  EXPECT_TRUE(has_ns_id(markers, std::string("stop") + wall_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("stop") + text_ns_suffix, 0));
  for (const auto & marker : markers.markers) {
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
    EXPECT_EQ(marker.pose.position.x, 1.0);
    EXPECT_EQ(marker.pose.position.y, 2.0);
  }
  markers = creator.create_markers();
  ASSERT_EQ(markers.markers.size(), 2UL);
  EXPECT_TRUE(has_ns_id(markers, std::string("stop") + wall_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("stop") + text_ns_suffix, 0));
  for (const auto & marker : markers.markers)
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETE);
}

TEST(VirtualWallMarkerCreator, manyWalls)
{
  motion_utils::VirtualWall wall;
  motion_utils::VirtualWallMarkerCreator creator;
  wall.style = motion_utils::VirtualWallType::stop;
  wall.ns = "ns1_";
  creator.add_virtual_wall(wall);
  creator.add_virtual_wall(wall);
  creator.add_virtual_wall(wall);
  wall.ns = "ns2_";
  creator.add_virtual_wall(wall);
  wall.style = motion_utils::VirtualWallType::slowdown;
  wall.ns = "ns2_";
  creator.add_virtual_wall(wall);
  creator.add_virtual_wall(wall);
  wall.ns = "ns3_";
  creator.add_virtual_wall(wall);
  creator.add_virtual_wall(wall);
  creator.add_virtual_wall(wall);
  wall.style = motion_utils::VirtualWallType::deadline;
  wall.ns = "ns1_";
  creator.add_virtual_wall(wall);
  wall.ns = "ns2_";
  creator.add_virtual_wall(wall);
  wall.ns = "ns3_";
  creator.add_virtual_wall(wall);

  auto markers = creator.create_markers();
  ASSERT_EQ(markers.markers.size(), 12UL * 2);
  EXPECT_TRUE(has_ns_id(markers, std::string("ns1_stop") + wall_ns_suffix, 0, 2));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns1_stop") + text_ns_suffix, 0, 2));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns2_stop") + wall_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns2_stop") + text_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns2_slow_down") + wall_ns_suffix, 0, 1));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns2_slow_down") + text_ns_suffix, 0, 1));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns3_slow_down") + wall_ns_suffix, 0, 2));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns3_slow_down") + text_ns_suffix, 0, 2));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns1_dead_line") + wall_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns1_dead_line") + text_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns2_dead_line") + wall_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns2_dead_line") + text_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns3_dead_line") + wall_ns_suffix, 0));
  EXPECT_TRUE(has_ns_id(markers, std::string("ns3_dead_line") + text_ns_suffix, 0));
  markers = creator.create_markers();
  ASSERT_EQ(markers.markers.size(), 12UL * 2);
  for (const auto & marker : markers.markers)
    EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::DELETE);
  markers = creator.create_markers();
  ASSERT_TRUE(markers.markers.empty());
}
}  // namespace
