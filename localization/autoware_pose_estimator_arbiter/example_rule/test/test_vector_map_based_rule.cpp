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

#include "switch_rule/vector_map_based_rule.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <unordered_set>

class VectorMapBasedRuleMockNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");

    const auto running_estimator_list =
      std::unordered_set<autoware::pose_estimator_arbiter::PoseEstimatorType>{
        autoware::pose_estimator_arbiter::PoseEstimatorType::ndt,
        autoware::pose_estimator_arbiter::PoseEstimatorType::yabloc,
        autoware::pose_estimator_arbiter::PoseEstimatorType::eagleye,
        autoware::pose_estimator_arbiter::PoseEstimatorType::artag};

    shared_data_ = std::make_shared<autoware::pose_estimator_arbiter::SharedData>();

    rule_ = std::make_shared<autoware::pose_estimator_arbiter::switch_rule::VectorMapBasedRule>(
      *node, running_estimator_list, shared_data_);
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<autoware::pose_estimator_arbiter::SharedData> shared_data_;
  std::shared_ptr<autoware::pose_estimator_arbiter::switch_rule::VectorMapBasedRule> rule_;

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(VectorMapBasedRuleMockNode, vectorMapBasedRule)
{
  // Create dummy lanelet2 and set
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

    using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;
    HADMapBin msg;
    lanelet::utils::conversion::toBinMsg(lanelet_map, &msg);

    // Set
    shared_data_->vector_map.set_and_invoke(std::make_shared<const HADMapBin>(msg));
  }

  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  auto create_pose = [](double x, double y) -> PoseCovStamped::ConstSharedPtr {
    PoseCovStamped msg;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    return std::make_shared<const PoseCovStamped>(msg);
  };

  {
    shared_data_->localization_pose_cov.set_and_invoke(create_pose(5, 5));
    auto ret = rule_->update();
    EXPECT_TRUE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::ndt));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::yabloc));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::eagleye));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::artag));
  }
  {
    shared_data_->localization_pose_cov.set_and_invoke(create_pose(15, 15));
    auto ret = rule_->update();
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::ndt));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::yabloc));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::eagleye));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::artag));
  }
}
