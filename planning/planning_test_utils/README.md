# Planning Interface Test Manager

## Background

In each node of the planning module, when exceptional input, such as unusual routes or significantly deviated ego-position, is given, the node may not be prepared for such input and could crash. As a result, debugging node crashes can be time-consuming. For example, if an empty trajectory is given as input and it was not anticipated during implementation, the node might crash due to the unaddressed exceptional input when changes are merged, during scenario testing or while the system is running on an actual vehicle.

## Purpose

The purpose is to provide a utility for implementing tests to ensure that node operates correctly when receiving exceptional input. By utilizing this utility and implementing tests for exceptional input, the purpose is to reduce bugs that are only discovered when actually running the system, by requiring measures for exceptional input before merging PRs.

## Features

### Confirmation of normal operation

For the test target node, confirm that the node operates correctly and publishes the required messages for subsequent nodes. To do this, test_node publish the necessary messages and confirm that the node's output is being published.

### Robustness confirmation for special inputs

After confirming normal operation, ensure that the test target node does not crash when given exceptional input. To do this, provide exceptional input from the test_node and confirm that the node does not crash.

(WIP)

## Usage

```cpp

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);

  // instantiate test_manager with PlanningInterfaceTestManager type
  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  // get package directories for necessary configuration files
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto target_node_dir =
    ament_index_cpp::get_package_share_directory("target_node");

  // set arguments to get the config file
  node_options.arguments(
    {"--ros-args", "--params-file",
     planning_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     planning_validator_dir + "/config/planning_validator.param.yaml"});

  // instantiate the TargetNode with node_options
  auto test_target_node = std::make_shared<TargetNode>(node_options);

  // publish the necessary topics from test_manager second argument is topic name
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishMaxVelocity(
    test_target_node, "motion_velocity_smoother/input/external_velocity_limit_mps");

  // set scenario_selector's input topic name(this topic is changed to test node)
  test_manager->setTrajectoryInputTopicName("input/parking/trajectory");

  // test with normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));

  // make sure target_node is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with trajectory input with empty/one point/overlapping point
  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(test_target_node));

  // shutdown ROS context
  rclcpp::shutdown();
}
```

## Implemented tests

| Node                       | Test name                                                                                 | exceptional input | output         | Exceptional input pattern                                                             |
| -------------------------- | ----------------------------------------------------------------------------------------- | ----------------- | -------------- | ------------------------------------------------------------------------------------- |
| planning_validator         | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | Empty, single point, path with duplicate points                                       |
| motion_velocity_smoother   | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | Empty, single point, path with duplicate points                                       |
| obstacle_cruise_planner    | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | Empty, single point, path with duplicate points                                       |
| obstacle_stop_planner      | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | Empty, single point, path with duplicate points                                       |
| obstacle_velocity_limiter  | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | Empty, single point, path with duplicate points                                       |
| obstacle_avoidance_planner | NodeTestWithExceptionTrajectory                                                           | trajectory        | trajectory     | Empty, single point, path with duplicate points                                       |
| scenario_selector          | NodeTestWithExceptionTrajectoryLaneDrivingMode NodeTestWithExceptionTrajectoryParkingMode | trajectory        | scenario       | Empty, single point, path with duplicate points for scenarios:LANEDRIVING and PARKING |
| freespace_planner          | NodeTestWithExceptionRoute                                                                | route             | trajectory     | Empty route                                                                           |
| behavior_path_planner      | NodeTestWithExceptionRoute NodeTestWithOffTrackEgoPose                                    | route             | route odometry | Empty route Off-lane ego-position                                                     |
| behavior_velocity_planner  | NodeTestWithExceptionPathWithLaneID                                                       | path_with_lane_id | path           | Empty path                                                                            |

## Important Notes

During test execution, when launching a node, parameters are loaded from the parameter file within each package. Therefore, when adding parameters, it is necessary to add the required parameters to the parameter file in the target node package. This is to prevent the node from being unable to launch if there are missing parameters when retrieving them from the parameter file during node launch.

## Future extensions / Unimplemented parts

(WIP)
