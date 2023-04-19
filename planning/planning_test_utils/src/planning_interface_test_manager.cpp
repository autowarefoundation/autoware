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

PlanningInterfaceTestManager::PlanningInterfaceTestManager()
{
  test_node_ = std::make_shared<rclcpp::Node>("planning_interface_test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PlanningInterfaceTestManager::publishOdometry(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<Odometry>(test_node_, target_node, topic_name, odom_pub_);
}

void PlanningInterfaceTestManager::publishOdometry(rclcpp::Node::SharedPtr target_node)
{
  publishOdometry(target_node, input_odometry_name_);
}

void PlanningInterfaceTestManager::publishMaxVelocity(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<VelocityLimit>(test_node_, target_node, topic_name, max_velocity_pub_);
}

void PlanningInterfaceTestManager::publishPointCloud(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<PointCloud2>(test_node_, target_node, topic_name, point_cloud_pub_);
}

void PlanningInterfaceTestManager::publishAcceleration(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<AccelWithCovarianceStamped>(
    test_node_, target_node, topic_name, acceleration_pub_);
}

void PlanningInterfaceTestManager::publishPredictedObjects(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<PredictedObjects>(
    test_node_, target_node, topic_name, predicted_objects_pub_);
}

void PlanningInterfaceTestManager::publishExpandStopRange(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<ExpandStopRange>(
    test_node_, target_node, topic_name, expand_stop_range_pub_);
}

void PlanningInterfaceTestManager::publishOccupancyGrid(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<OccupancyGrid>(test_node_, target_node, topic_name, occupancy_grid_pub_);
}

void PlanningInterfaceTestManager::publishCostMap(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<OccupancyGrid>(test_node_, target_node, topic_name, cost_map_pub_);
}

void PlanningInterfaceTestManager::publishMap(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<HADMapBin>(test_node_, target_node, topic_name, map_pub_);
}

void PlanningInterfaceTestManager::publishLaneDrivingScenario(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishScenarioData(
    test_node_, target_node, topic_name, lane_driving_scenario_pub_, Scenario::LANEDRIVING);
}

void PlanningInterfaceTestManager::publishParkingScenario(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishScenarioData(
    test_node_, target_node, topic_name, parking_scenario_pub_, Scenario::PARKING);
}

void PlanningInterfaceTestManager::publishInitialPose(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  publishInitialPoseData(target_node, topic_name);
}

void PlanningInterfaceTestManager::publishInitialPose(rclcpp::Node::SharedPtr target_node)
{
  publishInitialPose(target_node, input_initial_pose_name_);
}

void PlanningInterfaceTestManager::publishParkingState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<std_msgs::msg::Bool>(
    test_node_, target_node, topic_name, parking_state_pub_);
}

void PlanningInterfaceTestManager::publishTrajectory(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<Trajectory>(test_node_, target_node, topic_name, trajectory_pub_);
}

void PlanningInterfaceTestManager::publishRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<LaneletRoute>(test_node_, target_node, topic_name, route_pub_);
}

void PlanningInterfaceTestManager::publishTF(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<TFMessage>(test_node_, target_node, topic_name, TF_pub_);
}

void PlanningInterfaceTestManager::publishLateralOffset(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<LateralOffset>(test_node_, target_node, topic_name, lateral_offset_pub_);
}

void PlanningInterfaceTestManager::publishOperationModeState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<OperationModeState>(
    test_node_, target_node, topic_name, operation_mode_state_pub_);
}

void PlanningInterfaceTestManager::publishTrafficSignals(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<TrafficSignalArray>(
    test_node_, target_node, topic_name, traffic_signals_pub_);
}
void PlanningInterfaceTestManager::publishExternalTrafficSignals(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<TrafficSignalArray>(
    test_node_, target_node, topic_name, external_traffic_signals_pub_);
}
void PlanningInterfaceTestManager::publishVirtualTrafficLightState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<VirtualTrafficLightStateArray>(
    test_node_, target_node, topic_name, virtual_traffic_light_states_pub_);
}
void PlanningInterfaceTestManager::publishExternalCrosswalkStates(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<CrosswalkStatus>(
    test_node_, target_node, topic_name, external_crosswalk_states_pub_);
}
void PlanningInterfaceTestManager::publishExternalIntersectionStates(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<IntersectionStatus>(
    test_node_, target_node, topic_name, external_intersection_states_pub_);
}

void PlanningInterfaceTestManager::publishInitialPoseData(
  rclcpp::Node::SharedPtr target_node, std::string topic_name, double shift)
{
  std::shared_ptr<Odometry> current_odometry = std::make_shared<Odometry>();
  const auto yaw = 0.9724497591854532;
  const auto shift_x = shift * std::sin(yaw);
  const auto shift_y = shift * std::cos(yaw);
  const std::array<double, 4> start_pose{
    3722.16015625 + shift_x, 73723.515625 + shift_y, 0.233112560494183, yaw};
  current_odometry->pose.pose = test_utils::createPose(start_pose);
  current_odometry->header.frame_id = "map";

  test_utils::setPublisher(test_node_, topic_name, initial_pose_pub_);
  initial_pose_pub_->publish(*current_odometry);
  test_utils::spinSomeNodes(test_node_, target_node);
}

void PlanningInterfaceTestManager::publishInitialPoseTF(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.;
  quaternion.y = 0.;
  quaternion.z = 0.23311256049418302;
  quaternion.w = 0.9724497591854532;

  TransformStamped tf;
  tf.header.stamp = target_node->get_clock()->now();
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = 3722.16015625;
  tf.transform.translation.y = 73723.515625;
  tf.transform.translation.z = 0;
  tf.transform.rotation = quaternion;

  tf2_msgs::msg::TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(tf));

  test_utils::setPublisher(test_node_, topic_name, initial_pose_tf_pub_);
  initial_pose_tf_pub_->publish(tf_msg);
  test_utils::spinSomeNodes(test_node_, target_node);
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
  test_utils::setPublisher(test_node_, topic_name, normal_trajectory_pub_);
  normal_trajectory_pub_->publish(test_utils::generateTrajectory<Trajectory>(10, 1.0));
  test_utils::spinSomeNodes(test_node_, target_node);
}

void PlanningInterfaceTestManager::publishAbnormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const Trajectory & abnormal_trajectory)
{
  test_utils::setPublisher(test_node_, input_trajectory_name_, abnormal_trajectory_pub_);
  abnormal_trajectory_pub_->publish(abnormal_trajectory);
  test_utils::spinSomeNodes(test_node_, target_node);
}

void PlanningInterfaceTestManager::publishNominalRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::setPublisher(test_node_, topic_name, normal_route_pub_);
  normal_route_pub_->publish(test_utils::makeNormalRoute());
  test_utils::spinSomeNodes(test_node_, target_node);
}

void PlanningInterfaceTestManager::publishBehaviorNominalRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::setPublisher(test_node_, topic_name, behavior_normal_route_pub_);
  behavior_normal_route_pub_->publish(test_utils::makeBehaviorNormalRoute());
  test_utils::spinSomeNodes(test_node_, target_node);
}

void PlanningInterfaceTestManager::publishAbnormalRoute(
  rclcpp::Node::SharedPtr target_node, const LaneletRoute & abnormal_route)
{
  test_utils::setPublisher(test_node_, input_route_name_, abnormal_route_pub_);
  abnormal_route_pub_->publish(abnormal_route);
  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

void PlanningInterfaceTestManager::publishNominalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::setPublisher(test_node_, topic_name, normal_path_with_lane_id_pub_);
  normal_path_with_lane_id_pub_->publish(test_utils::loadPathWithLaneIdInYaml());

  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

void PlanningInterfaceTestManager::publishAbNominalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::setPublisher(test_node_, topic_name, abnormal_path_with_lane_id_pub_);
  normal_path_with_lane_id_pub_->publish(PathWithLaneId{});
  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

void PlanningInterfaceTestManager::setTrajectorySubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, traj_sub_, count_);
}

void PlanningInterfaceTestManager::setRouteSubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, route_sub_, count_);
}
void PlanningInterfaceTestManager::setScenarioSubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, scenario_sub_, count_);
}

void PlanningInterfaceTestManager::setPathWithLaneIdSubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, path_with_lane_id_sub_, count_);
}

void PlanningInterfaceTestManager::setPathSubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, path_sub_, count_);
}

// test for normal working
void PlanningInterfaceTestManager::testWithNominalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  publishNominalTrajectory(target_node, input_trajectory_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 2);
}

// check to see if target node is dead.
void PlanningInterfaceTestManager::testWithAbnormalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  ASSERT_NO_THROW(publishAbnormalTrajectory(target_node, Trajectory{}));
  ASSERT_NO_THROW(
    publishAbnormalTrajectory(target_node, test_utils::generateTrajectory<Trajectory>(1, 0.0)));
  ASSERT_NO_THROW(publishAbnormalTrajectory(
    target_node, test_utils::generateTrajectory<Trajectory>(10, 0.0, 0.0, 0.0, 0.0, 1)));
}

// test for normal working
void PlanningInterfaceTestManager::testWithNominalRoute(rclcpp::Node::SharedPtr target_node)
{
  publishNominalRoute(target_node, input_route_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

// test for normal working
void PlanningInterfaceTestManager::testWithBehaviorNominalRoute(rclcpp::Node::SharedPtr target_node)
{
  publishBehaviorNominalRoute(target_node, input_route_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

// check to see if target node is dead.
void PlanningInterfaceTestManager::testWithAbnormalRoute(rclcpp::Node::SharedPtr target_node)
{
  ASSERT_NO_THROW(publishAbnormalRoute(target_node, LaneletRoute{}));
  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

// test for normal working
void PlanningInterfaceTestManager::testWithNominalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node)
{
  publishNominalPathWithLaneId(target_node, input_path_with_lane_id_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

// check to see if target node is dead.
void PlanningInterfaceTestManager::testWithAbnormalPathWithLaneId(
  rclcpp::Node::SharedPtr target_node)
{
  publishAbNominalPathWithLaneId(target_node, input_path_with_lane_id_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 5);
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
  if (input_route_name_.empty()) {
    throw std::runtime_error("route topic name is not set");
  }
  if (input_initial_pose_name_.empty()) {
    throw std::runtime_error("initial topic pose name is not set");
  }

  publishBehaviorNominalRoute(target_node, input_route_name_);

  const std::vector<double> deviation_from_route = {0.0, 1.0, 10.0, 100.0};
  for (const auto & deviation : deviation_from_route) {
    publishInitialPoseData(target_node, input_initial_pose_name_, deviation);
    test_utils::spinSomeNodes(test_node_, target_node, 5);
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
  // write me
  (void)target_node;
}

void PlanningInterfaceTestManager::testOffTrackFromTrajectory(rclcpp::Node::SharedPtr target_node)
{
  // write me
  (void)target_node;
}

int PlanningInterfaceTestManager::getReceivedTopicNum()
{
  return count_;
}

}  // namespace planning_test_utils
