// Copyright 2023 Tier IV, Inc.
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

#include <planning_interface_test_manager/planning_interface_test_manager.hpp>
#include <planning_interface_test_manager/planning_interface_test_manager_utils.hpp>

namespace planning_test_utils
{
void PlanningIntefaceTestManager::declareVehicleInfoParams(rclcpp::NodeOptions & node_options)
{
  // for vehicle info
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", 3.0);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);
}

void PlanningIntefaceTestManager::declareNearestSearchDistanceParams(
  rclcpp::NodeOptions & node_options)
{
  node_options.append_parameter_override("ego_nearest_dist_threshold", 3.0);
  node_options.append_parameter_override("ego_nearest_yaw_threshold", 1.046);
}

void PlanningIntefaceTestManager::publishOdometry(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishEmptyData<Odometry>(test_node_, target_node, topic_name, odom_pub_);
}

void PlanningIntefaceTestManager::publishMaxVelocity(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishEmptyData<VelocityLimit>(
    test_node_, target_node, topic_name, max_velocity_pub_);
}

void PlanningIntefaceTestManager::publishPointCloud(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishEmptyData<PointCloud2>(test_node_, target_node, topic_name, point_cloud_pub_);
}

void PlanningIntefaceTestManager::publishAcceleration(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishEmptyData<AccelWithCovarianceStamped>(
    test_node_, target_node, topic_name, acceleration_pub_);
}

void PlanningIntefaceTestManager::publishPredictedObjects(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishEmptyData<PredictedObjects>(
    test_node_, target_node, topic_name, predicted_objects_pub_);
}

void PlanningIntefaceTestManager::publishExpandStopRange(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishEmptyData<ExpandStopRange>(
    test_node_, target_node, topic_name, expand_stop_range_pub_);
}

void PlanningIntefaceTestManager::setTrajectoryInputTopicName(std::string topic_name)
{
  input_trajectory_name_ = topic_name;
}

void PlanningIntefaceTestManager::publishNominalTrajectory(std::string topic_name)
{
  test_utils::setPublisher(test_node_, topic_name, normal_trajectory_pub_);
  normal_trajectory_pub_->publish(test_utils::generateTrajectory<Trajectory>(10, 1.0));
}

void PlanningIntefaceTestManager::setTrajectorySubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, traj_sub_, count_);
}

// test for normal working
void PlanningIntefaceTestManager::testWithNominalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  publishNominalTrajectory(input_trajectory_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 2);
}

// check to see if target node is dead.
void PlanningIntefaceTestManager::testWithAbnormalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  ASSERT_NO_THROW(publishAbnormalTrajectory(target_node, Trajectory{}));
  ASSERT_NO_THROW(
    publishAbnormalTrajectory(target_node, test_utils::generateTrajectory<Trajectory>(1, 0.0)));
  ASSERT_NO_THROW(publishAbnormalTrajectory(
    target_node, test_utils::generateTrajectory<Trajectory>(10, 0.0, 0.0, 0.0, 0.0, 1)));
}

void PlanningIntefaceTestManager::publishAbnormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const Trajectory & abnormal_trajectory)
{
  test_utils::setPublisher(test_node_, input_trajectory_name_, abnormal_trajectory_pub_);
  abnormal_trajectory_pub_->publish(abnormal_trajectory);
  test_utils::spinSomeNodes(test_node_, target_node);
}

int PlanningIntefaceTestManager::getReceivedTopicNum() { return count_; }

}  // namespace planning_test_utils
