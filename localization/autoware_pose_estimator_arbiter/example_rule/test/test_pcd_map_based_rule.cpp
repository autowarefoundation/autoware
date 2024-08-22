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

#include "switch_rule/pcd_map_based_rule.hpp"

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <unordered_set>

class PcdMapBasedRuleMockNode : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("test_node");
    node->declare_parameter("pcd_density_lower_threshold", 1);
    node->declare_parameter("pcd_density_upper_threshold", 5);

    const auto running_estimator_list =
      std::unordered_set<autoware::pose_estimator_arbiter::PoseEstimatorType>{
        autoware::pose_estimator_arbiter::PoseEstimatorType::ndt,
        autoware::pose_estimator_arbiter::PoseEstimatorType::yabloc,
        autoware::pose_estimator_arbiter::PoseEstimatorType::eagleye,
        autoware::pose_estimator_arbiter::PoseEstimatorType::artag};

    shared_data_ = std::make_shared<autoware::pose_estimator_arbiter::SharedData>();

    rule_ = std::make_shared<autoware::pose_estimator_arbiter::switch_rule::PcdMapBasedRule>(
      *node, running_estimator_list, shared_data_);
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<autoware::pose_estimator_arbiter::SharedData> shared_data_;
  std::shared_ptr<autoware::pose_estimator_arbiter::switch_rule::PcdMapBasedRule> rule_;

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(PcdMapBasedRuleMockNode, pcdMapBasedRule)
{
  // Create dummy pcd and set
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (float x = -10; x < 10.0; x += 0.2) {
      for (float y = -10; y < 10.0; y += 0.2) {
        cloud.push_back(pcl::PointXYZ(x, y, 0));
      }
    }

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);

    // Set
    shared_data_->point_cloud_map.set_and_invoke(std::make_shared<const PointCloud2>(msg));
  }

  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  auto create_pose = [](double x, double y) -> PoseCovStamped {
    PoseCovStamped msg;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    return msg;
  };

  {
    auto position = create_pose(5, 5);
    shared_data_->localization_pose_cov.set_and_invoke(
      std::make_shared<const PoseCovStamped>(position));
    auto ret = rule_->update();
    EXPECT_TRUE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::ndt));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::yabloc));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::eagleye));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::artag));
  }

  {
    auto position = create_pose(100, 100);
    shared_data_->localization_pose_cov.set_and_invoke(
      std::make_shared<const PoseCovStamped>(position));
    auto ret = rule_->update();
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::ndt));
    EXPECT_TRUE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::yabloc));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::eagleye));
    EXPECT_FALSE(ret.at(autoware::pose_estimator_arbiter::PoseEstimatorType::artag));
  }
}
