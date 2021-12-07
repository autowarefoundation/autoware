// Copyright 2021 Tier IV, Inc.
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

#include "planning_error_monitor/planning_error_monitor_node.hpp"
#include "test_planning_error_monitor_helper.hpp"

#include <gtest/gtest.h>

#include <string>

constexpr double NOMINAL_INTERVAL = 1.0;
constexpr double ERROR_INTERVAL = 1000.0;
constexpr double ERROR_CURVATURE = 2.0;

TEST(PlanningErrorMonitor, ValidValueChecker)
{
  using autoware_auto_planning_msgs::msg::Trajectory;
  using planning_diagnostics::PlanningErrorMonitorNode;

  // Valid Trajectory
  Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);

  std::string valid_error_msg;
  ASSERT_TRUE(PlanningErrorMonitorNode::checkTrajectoryPointValue(valid_traj, valid_error_msg));
  ASSERT_EQ(valid_error_msg, "This Trajectory doesn't have any invalid values");

  // Nan Trajectory
  Trajectory nan_traj = generateNanTrajectory();

  std::string nan_error_msg;
  ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryPointValue(nan_traj, nan_error_msg));
  ASSERT_EQ(nan_error_msg, "This trajectory has an infinite value");

  // Inf Trajectory
  Trajectory inf_traj = generateInfTrajectory();

  std::string inf_error_msg;
  ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryPointValue(inf_traj, inf_error_msg));
  ASSERT_EQ(nan_error_msg, "This trajectory has an infinite value");
}

TEST(PlanningErrorMonitor, TrajectoryIntervalChecker)
{
  using autoware_auto_planning_msgs::msg::Trajectory;
  using planning_diagnostics::PlanningErrorMonitorNode;
  PlanningErrorMonitorDebugNode debug_marker;
  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);

    std::string valid_msg;
    ASSERT_TRUE(PlanningErrorMonitorNode::checkTrajectoryInterval(
      valid_traj, ERROR_INTERVAL, valid_msg, debug_marker));
    ASSERT_EQ(valid_msg, "Trajectory Interval Length is within the expected range");

    std::string boundary_msg;
    ASSERT_TRUE(PlanningErrorMonitorNode::checkTrajectoryInterval(
      valid_traj, NOMINAL_INTERVAL, boundary_msg, debug_marker));
    ASSERT_EQ(boundary_msg, "Trajectory Interval Length is within the expected range");
  }

  // Long Interval Trajectory
  {
    Trajectory long_interval_traj = generateTrajectory(ERROR_INTERVAL);
    std::string long_interval_error_msg;
    ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryInterval(
      long_interval_traj, NOMINAL_INTERVAL, long_interval_error_msg, debug_marker));
    ASSERT_EQ(
      long_interval_error_msg, "Trajectory Interval Length is longer than the expected range");
  }
}

TEST(PlanningErrorMonitor, TrajectoryCurvatureChecker)
{
  using autoware_auto_planning_msgs::msg::Trajectory;
  using planning_diagnostics::PlanningErrorMonitorNode;
  PlanningErrorMonitorDebugNode debug_marker;
  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);
    std::string valid_error_msg;
    ASSERT_TRUE(PlanningErrorMonitorNode::checkTrajectoryCurvature(
      valid_traj, ERROR_CURVATURE, valid_error_msg, debug_marker));
    ASSERT_EQ(valid_error_msg, "This trajectory's curvature is within the expected range");
  }
}

TEST(PlanningErrorMonitor, TrajectoryRelativeAngleChecker)
{
  using autoware_auto_planning_msgs::msg::Trajectory;
  using planning_diagnostics::PlanningErrorMonitorNode;
  PlanningErrorMonitorDebugNode debug_marker;
  const double too_close_dist = 0.05;
  const double too_sharp_turn = M_PI_4;
  {
    /**
     * y: 0 0 0 0&INF 0 0 0 0 0 0
     **/
    Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);
    valid_traj.points[4].pose.position.x = 3;
    valid_traj.points[4].pose.position.y = 10;
    // for (auto t : valid_traj.points) {
    //   std::cout << "p: (x , y) = " << "( "<<t.pose.position.x <<
    // " , " << t.pose.position.y <<" )"<< std::endl;
    // }
    std::string valid_error_msg;
    ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryRelativeAngle(
      valid_traj, too_sharp_turn, too_close_dist, valid_error_msg, debug_marker));
    ASSERT_EQ(
      valid_error_msg, "This Trajectory's relative angle has larger value than the expected value");
  }

  {
    /** <---inverted pattern-----
     * y: 0 0 0 0 INF 0 0 0 0 0 0
     **/
    Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);
    valid_traj.points[4].pose.position.y = -10000000;
    for (auto t : valid_traj.points) {
      t.pose.position.x *= -1;
    }
    std::string valid_error_msg;
    ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryRelativeAngle(
      valid_traj, too_sharp_turn, too_close_dist, valid_error_msg, debug_marker));
    ASSERT_EQ(
      valid_error_msg, "This Trajectory's relative angle has larger value than the expected value");
  }

  {
    /** vertical pattern
     * x: 0 0 0 0 INF 0 0 0 0 0 0
     **/
    Trajectory valid_traj = generateTrajectory(NOMINAL_INTERVAL);
    for (size_t i = 0; i < valid_traj.points.size(); i++) {
      auto & p = valid_traj.points[i].pose.position;
      p.x = 0;
      p.y = i;
    }
    valid_traj.points[4].pose.position.x = -10000000;
    std::string valid_error_msg;
    ASSERT_FALSE(PlanningErrorMonitorNode::checkTrajectoryRelativeAngle(
      valid_traj, too_sharp_turn, too_close_dist, valid_error_msg, debug_marker));
    ASSERT_EQ(
      valid_error_msg, "This Trajectory's relative angle has larger value than the expected value");
  }
}
