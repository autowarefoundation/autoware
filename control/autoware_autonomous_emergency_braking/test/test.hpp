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

#ifndef TEST_HPP_
#define TEST_HPP_

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"
#include "gtest/gtest.h"

#include <autoware/autonomous_emergency_braking/node.hpp>
#include <autoware/autonomous_emergency_braking/utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <vector>
namespace autoware::motion::control::autonomous_emergency_braking::test
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_system_msgs::msg::AutowareState;
using autoware_vehicle_msgs::msg::VelocityReport;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using autoware::universe_utils::Polygon2d;
using autoware::vehicle_info_utils::VehicleInfo;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using Path = std::vector<geometry_msgs::msg::Pose>;
using Vector3 = geometry_msgs::msg::Vector3;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using std_msgs::msg::Header;

std::shared_ptr<AEB> generateNode();
Header get_header(const char * const frame_id, rclcpp::Time t);
Imu make_imu_message(
  const Header & header, const double ax, const double ay, const double yaw,
  const double angular_velocity_z);
VelocityReport make_velocity_report_msg(
  const Header & header, const double lat_velocity, const double long_velocity,
  const double heading_rate);
class PubSubNode : public rclcpp::Node
{
public:
  explicit PubSubNode(const rclcpp::NodeOptions & node_options);
  // publisher
  rclcpp::Publisher<Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_point_cloud_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_predicted_traj_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_predicted_objects_;
  rclcpp::Publisher<AutowareState>::SharedPtr pub_autoware_state_;
  // timer
  // rclcpp::TimerBase::SharedPtr timer_;
  void publishDefaultTopicsNoSpin()
  {
    const auto header = get_header("base_link", now());
    const auto imu_msg = make_imu_message(header, 0.0, 0.0, 0.0, 0.05);
    const auto velocity_msg = make_velocity_report_msg(header, 0.0, 3.0, 0.0);

    pub_imu_->publish(imu_msg);
    pub_velocity_->publish(velocity_msg);
  };
};

std::shared_ptr<PubSubNode> generatePubSubNode();

class TestAEB : public ::testing::Test
{
public:
  TestAEB() {}
  TestAEB(const TestAEB &) = delete;
  TestAEB(TestAEB &&) = delete;
  TestAEB & operator=(const TestAEB &) = delete;
  TestAEB & operator=(TestAEB &&) = delete;
  ~TestAEB() override = default;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    pub_sub_node_ = generatePubSubNode();
    aeb_node_ = generateNode();
  }

  void TearDown() override
  {
    aeb_node_.reset();
    pub_sub_node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<PubSubNode> pub_sub_node_;
  std::shared_ptr<AEB> aeb_node_;
};
}  // namespace autoware::motion::control::autonomous_emergency_braking::test

#endif  // TEST_HPP_
