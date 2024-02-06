// Copyright 2023 Autoware Foundation
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

#include "pose_estimator_arbiter/rule_helper/grid_key.hpp"
#include "pose_estimator_arbiter/rule_helper/pcd_occupancy.hpp"
#include "pose_estimator_arbiter/rule_helper/pose_estimator_area.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <unordered_set>

class MockNode : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");
  }

  std::shared_ptr<rclcpp::Node> node{nullptr};

  virtual void TearDown() { rclcpp::shutdown(); }
};

TEST_F(MockNode, poseEstimatorArea)
{
  auto create_polygon3d = []() -> lanelet::Polygon3d {
    lanelet::Polygon3d polygon;
    polygon.setAttribute(lanelet::AttributeName::Type, {"pose_estimator_specify"});
    polygon.setAttribute(lanelet::AttributeName::Subtype, {"ndt"});
    lanelet::Id index = 0;
    polygon.push_back(lanelet::Point3d(index++, 0, 0));
    polygon.push_back(lanelet::Point3d(index++, 10, 0));
    polygon.push_back(lanelet::Point3d(index++, 10, 10));
    polygon.push_back(lanelet::Point3d(index++, 0, 10));
    return polygon;
  };

  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet_map->add(create_polygon3d());

  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Point = geometry_msgs::msg::Point;
  HADMapBin msg;
  lanelet::utils::conversion::toBinMsg(lanelet_map, &msg);

  pose_estimator_arbiter::rule_helper::PoseEstimatorArea pose_estimator_area(&(*node));
  pose_estimator_area.init(std::make_shared<HADMapBin>(msg));

  EXPECT_TRUE(pose_estimator_area.within(Point().set__x(5).set__y(5).set__z(0), "ndt"));
  EXPECT_FALSE(pose_estimator_area.within(Point().set__x(5).set__y(5).set__z(0), "yabloc"));
  EXPECT_FALSE(pose_estimator_area.within(Point().set__x(-5).set__y(-5).set__z(0), "ndt"));
  EXPECT_FALSE(pose_estimator_area.within(Point().set__x(-5).set__y(-5).set__z(0), "yabloc"));
}

TEST_F(MockNode, pcdOccupancy)
{
  using pose_estimator_arbiter::rule_helper::PcdOccupancy;
  const int pcd_density_upper_threshold = 20;
  const int pcd_density_lower_threshold = 10;

  pose_estimator_arbiter::rule_helper::PcdOccupancy pcd_occupancy(
    pcd_density_upper_threshold, pcd_density_lower_threshold);

  geometry_msgs::msg::Point point;
  std::string message;

  // Since we have not yet given a point cloud, this returns false.
  EXPECT_FALSE(pcd_occupancy.ndt_can_operate(point, &message));
}

TEST_F(MockNode, gridKey)
{
  using pose_estimator_arbiter::rule_helper::GridKey;
  EXPECT_TRUE(GridKey(10., -5.) == GridKey(10., -10.));
  EXPECT_TRUE(GridKey(10., -5.) != GridKey(10., -15.));

  EXPECT_TRUE(GridKey(10., -5.).get_center_point().x == 15.f);
  EXPECT_TRUE(GridKey(10., -5.).get_center_point().y == -5.f);
  EXPECT_TRUE(GridKey(10., -5.).get_center_point().z == 0.f);

  std::unordered_set<GridKey> set;
  set.emplace(10., -5.);
  EXPECT_EQ(set.count(GridKey(10., -5.)), 1ul);
  EXPECT_EQ(set.count(GridKey(10., -15.)), 0ul);
}
