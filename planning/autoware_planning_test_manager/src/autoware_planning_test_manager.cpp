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

#include "autoware/motion_utils/trajectory/conversion.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

namespace autoware::planning_test_manager
{

PlanningInterfaceTestManager::PlanningInterfaceTestManager()
{
  test_node_ = std::make_shared<rclcpp::Node>("planning_interface_test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PlanningInterfaceTestManager::publishOdometry(
  rclcpp::Node::SharedPtr target_node, std::string topic_name, const double shift)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, odom_pub_, autoware::test_utils::makeOdometry(shift));
}

void PlanningInterfaceTestManager::publishMaxVelocity(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, max_velocity_pub_, VelocityLimit{});
}

void PlanningInterfaceTestManager::publishPointCloud(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  PointCloud2 point_cloud_msg{};
  point_cloud_msg.header.frame_id = "base_link";
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, point_cloud_pub_, point_cloud_msg);
}

void PlanningInterfaceTestManager::publishAcceleration(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, acceleration_pub_, AccelWithCovarianceStamped{});
}

void PlanningInterfaceTestManager::publishPredictedObjects(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, predicted_objects_pub_, PredictedObjects{});
}

void PlanningInterfaceTestManager::publishExpandStopRange(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, expand_stop_range_pub_, ExpandStopRange{});
}

void PlanningInterfaceTestManager::publishOccupancyGrid(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, occupancy_grid_pub_,
    autoware::test_utils::makeCostMapMsg());
}

void PlanningInterfaceTestManager::publishCostMap(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, cost_map_pub_, autoware::test_utils::makeCostMapMsg());
}

void PlanningInterfaceTestManager::publishMap(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, map_pub_, autoware::test_utils::makeMapBinMsg());
}

void PlanningInterfaceTestManager::publishLaneDrivingScenario(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, lane_driving_scenario_pub_,
    autoware::test_utils::makeScenarioMsg(Scenario::LANEDRIVING));
}

void PlanningInterfaceTestManager::publishParkingScenario(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, parking_scenario_pub_,
    autoware::test_utils::makeScenarioMsg(Scenario::PARKING));
}

void PlanningInterfaceTestManager::publishInitialPose(
  rclcpp::Node::SharedPtr target_node, std::string topic_name, const double shift,
  ModuleName module_name)
{
  if (module_name == ModuleName::START_PLANNER) {
    autoware::test_utils::publishToTargetNode(
      test_node_, target_node, topic_name, initial_pose_pub_,
      autoware_planning_test_manager::utils::makeInitialPoseFromLaneId(10291));
  } else {
    autoware::test_utils::publishToTargetNode(
      test_node_, target_node, topic_name, initial_pose_pub_,
      autoware::test_utils::makeInitialPose(shift));
  }
}

void PlanningInterfaceTestManager::publishParkingState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, parking_state_pub_, std_msgs::msg::Bool{});
}

void PlanningInterfaceTestManager::publishTrajectory(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, trajectory_pub_, Trajectory{});
}

void PlanningInterfaceTestManager::publishRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, route_pub_, autoware::test_utils::makeNormalRoute());
}

void PlanningInterfaceTestManager::publishTF(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, TF_pub_,
    autoware::test_utils::makeTFMsg(target_node, "base_link", "map"));
}

void PlanningInterfaceTestManager::publishLateralOffset(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, lateral_offset_pub_, LateralOffset{});
}

void PlanningInterfaceTestManager::publishOperationModeState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, operation_mode_state_pub_, OperationModeState{});
}

void PlanningInterfaceTestManager::publishTrafficSignals(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, traffic_signals_pub_, TrafficLightGroupArray{});
}

void PlanningInterfaceTestManager::publishVirtualTrafficLightState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, virtual_traffic_light_states_pub_,
    VirtualTrafficLightStateArray{});
}

void PlanningInterfaceTestManager::publishInitialPoseTF(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, initial_pose_tf_pub_,
    autoware::test_utils::makeTFMsg(target_node, "odom", "base_link"));
}

void PlanningInterfaceTestManager::setTrajectoryInputTopicName(std::string topic_name)
{
  input_trajectory_name_ = topic_name;
}

void PlanningInterfaceTestManager::setParkingTrajectoryInputTopicName(std::string topic_name)
{
  input_parking_trajectory_name_ = topic_name;
}

void PlanningInterfaceTestManager::setLaneDrivingTrajectoryInputTopicName(std::string topic_name)
{
  input_lane_driving_trajectory_name_ = topic_name;
}

void PlanningInterfaceTestManager::setRouteInputTopicName(std::string topic_name)
{
  input_route_name_ = topic_name;
}

void PlanningInterfaceTestManager::setPathInputTopicName(std::string topic_name)
{
  input_path_name_ = topic_name;
}

void PlanningInterfaceTestManager::setPathWithLaneIdTopicName(std::string topic_name)
{
  input_path_with_lane_id_name_ = topic_name;
}

void PlanningInterfaceTestManager::setInitialPoseTopicName(std::string topic_name)
{
  input_initial_pose_name_ = topic_name;
}

void PlanningInterfaceTestManager::setOdometryTopicName(std::string topic_name)
{
  input_odometry_name_ = topic_name;
}

void PlanningInterfaceTestManager::publishNominalTrajectory(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, normal_trajectory_pub_,
    autoware::test_utils::generateTrajectory<Trajectory>(10, 1.0), 5);
}

void PlanningInterfaceTestManager::publishAbnormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const Trajectory & abnormal_trajectory)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, input_trajectory_name_, abnormal_trajectory_pub_, abnormal_trajectory);
}

void PlanningInterfaceTestManager::publishNominalRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, normal_route_pub_, autoware::test_utils::makeNormalRoute(),
    5);
}

void PlanningInterfaceTestManager::publishBehaviorNominalRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name, ModuleName module_name)
{
  if (module_name == ModuleName::START_PLANNER) {
    autoware::test_utils::publishToTargetNode(
      test_node_, target_node, topic_name, behavior_normal_route_pub_,
      autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId(10291, 10333), 5);
  } else {
    autoware::test_utils::publishToTargetNode(
      test_node_, target_node, topic_name, behavior_normal_route_pub_,
      autoware::test_utils::makeBehaviorNormalRoute(), 5);
  }
}

void PlanningInterfaceTestManager::publishAbnormalRoute(
  rclcpp::Node::SharedPtr target_node, const LaneletRoute & abnormal_route)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, input_route_name_, abnormal_route_pub_, abnormal_route, 5);
}

void PlanningInterfaceTestManager::publishNominalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, normal_path_with_lane_id_pub_,
    autoware::test_utils::loadPathWithLaneIdInYaml(), 5);
}

void PlanningInterfaceTestManager::publishAbNominalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, abnormal_path_with_lane_id_pub_, PathWithLaneId{}, 5);
}

void PlanningInterfaceTestManager::publishNominalPath(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, normal_path_pub_,
    autoware::motion_utils::convertToPath<tier4_planning_msgs::msg::PathWithLaneId>(
      autoware::test_utils::loadPathWithLaneIdInYaml()),
    5);
}

void PlanningInterfaceTestManager::publishAbnormalPath(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  autoware::test_utils::publishToTargetNode(
    test_node_, target_node, topic_name, abnormal_path_pub_, Path{}, 5);
}

void PlanningInterfaceTestManager::setTrajectorySubscriber(std::string topic_name)
{
  autoware::test_utils::setSubscriber(test_node_, topic_name, traj_sub_, count_);
}

void PlanningInterfaceTestManager::setRouteSubscriber(std::string topic_name)
{
  autoware::test_utils::setSubscriber(test_node_, topic_name, route_sub_, count_);
}
void PlanningInterfaceTestManager::setScenarioSubscriber(std::string topic_name)
{
  autoware::test_utils::setSubscriber(test_node_, topic_name, scenario_sub_, count_);
}

void PlanningInterfaceTestManager::setPathWithLaneIdSubscriber(std::string topic_name)
{
  autoware::test_utils::setSubscriber(test_node_, topic_name, path_with_lane_id_sub_, count_);
}

void PlanningInterfaceTestManager::setPathSubscriber(std::string topic_name)
{
  autoware::test_utils::setSubscriber(test_node_, topic_name, path_sub_, count_);
}

// test for normal working
void PlanningInterfaceTestManager::testWithNominalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  publishNominalTrajectory(target_node, input_trajectory_name_);
}

// check to see if target node is dead.
void PlanningInterfaceTestManager::testWithAbnormalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  publishAbnormalTrajectory(target_node, Trajectory{});
  publishAbnormalTrajectory(
    target_node, autoware::test_utils::generateTrajectory<Trajectory>(1, 0.0));
  publishAbnormalTrajectory(
    target_node, autoware::test_utils::generateTrajectory<Trajectory>(10, 0.0, 0.0, 0.0, 0.0, 1));
}

// test for normal working
void PlanningInterfaceTestManager::testWithNominalRoute(rclcpp::Node::SharedPtr target_node)
{
  publishNominalRoute(target_node, input_route_name_);
}

// test for normal working
void PlanningInterfaceTestManager::testWithBehaviorNominalRoute(
  rclcpp::Node::SharedPtr target_node, ModuleName module_name)
{
  publishBehaviorNominalRoute(target_node, input_route_name_, module_name);
}

// check to see if target node is dead.
void PlanningInterfaceTestManager::testWithAbnormalRoute(rclcpp::Node::SharedPtr target_node)
{
  publishAbnormalRoute(target_node, LaneletRoute{});
}

// test for normal working
void PlanningInterfaceTestManager::testWithNominalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node)
{
  publishNominalPathWithLaneId(target_node, input_path_with_lane_id_name_);
}

// check to see if target node is dead.
void PlanningInterfaceTestManager::testWithAbnormalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node)
{
  publishAbNominalPathWithLaneId(target_node, input_path_with_lane_id_name_);
}

// put all abnormal ego pose related tests in this functions to run all tests with one line code
void PlanningInterfaceTestManager::testRouteWithInvalidEgoPose(rclcpp::Node::SharedPtr target_node)
{
  testOffTrackFromRoute(target_node);
}

// put all abnormal ego pose related tests in this functions to run all tests with one line code
void PlanningInterfaceTestManager::testPathWithInvalidEgoPose(rclcpp::Node::SharedPtr target_node)
{
  testOffTrackFromPath(target_node);
}

// put all abnormal ego pose related tests in this functions to run all tests with one line code
void PlanningInterfaceTestManager::testPathWithLaneIdWithInvalidEgoPose(
  rclcpp::Node::SharedPtr target_node)
{
  testOffTrackFromPathWithLaneId(target_node);
}

// put all abnormal ego pose related tests in this functions to run all tests with one line code
void PlanningInterfaceTestManager::testTrajectoryWithInvalidEgoPose(
  rclcpp::Node::SharedPtr target_node)
{
  testOffTrackFromTrajectory(target_node);
}

void PlanningInterfaceTestManager::testOffTrackFromRoute(rclcpp::Node::SharedPtr target_node)
{
  publishBehaviorNominalRoute(target_node, input_route_name_);

  const std::vector<double> deviation_from_route = {0.0, 1.0, 10.0, 100.0};
  for (const auto & deviation : deviation_from_route) {
    publishInitialPose(target_node, input_initial_pose_name_, deviation);
  }
}

void PlanningInterfaceTestManager::testOffTrackFromPath(rclcpp::Node::SharedPtr target_node)
{
  // write me
  (void)target_node;
}

void PlanningInterfaceTestManager::testOffTrackFromPathWithLaneId(
  rclcpp::Node::SharedPtr target_node)
{
  publishNominalPathWithLaneId(target_node, input_path_with_lane_id_name_);

  const std::vector<double> deviation_from_path = {0.0, 1.0, 10.0, 100.0};
  for (const auto & deviation : deviation_from_path) {
    publishOdometry(target_node, input_odometry_name_, deviation);
  }
}

void PlanningInterfaceTestManager::testOffTrackFromTrajectory(rclcpp::Node::SharedPtr target_node)
{
  publishNominalTrajectory(target_node, input_trajectory_name_);

  const std::vector<double> deviation_from_traj = {0.0, 1.0, 10.0, 100.0};
  for (const auto & deviation : deviation_from_traj) {
    publishOdometry(target_node, input_odometry_name_, deviation);
  }
}

void PlanningInterfaceTestManager::testWithNominalPath(rclcpp::Node::SharedPtr target_node)
{
  publishNominalPath(target_node, input_path_name_);
}

void PlanningInterfaceTestManager::testWithAbnormalPath(rclcpp::Node::SharedPtr target_node)
{
  publishAbnormalPath(target_node, input_path_name_);
}

int PlanningInterfaceTestManager::getReceivedTopicNum()
{
  return count_;
}

}  // namespace autoware::planning_test_manager
