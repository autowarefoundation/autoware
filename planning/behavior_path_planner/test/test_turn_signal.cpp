// Copyright 2022 TIER IV, Inc.
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
#include "behavior_path_planner/turn_signal_decider.hpp"

#include "autoware_auto_planning_msgs/msg/path_point.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using behavior_path_planner::PathWithLaneId;
using behavior_path_planner::Pose;
using behavior_path_planner::TurnSignalDecider;
using behavior_path_planner::TurnSignalInfo;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromYaw;

constexpr double nearest_dist_threshold = 5.0;
constexpr double nearest_yaw_threshold = M_PI / 3.0;

namespace
{
PathWithLaneId generateStraightSamplePathWithLaneId(
  float initial_pose_value, float pose_increment, size_t point_sample)
{
  PathWithLaneId path;
  for (size_t idx = 0; idx < point_sample; ++idx) {
    PathPoint point;
    point.pose.position.x = std::exchange(initial_pose_value, initial_pose_value + pose_increment);
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.longitudinal_velocity_mps = 0.1;  // [m/s]
    point.heading_rate_rps = 0.0;           // [rad/s]
    point.is_final = (idx == point_sample - 1);

    PathPointWithLaneId path_point_with_lane_id;
    path_point_with_lane_id.point = point;
    path_point_with_lane_id.lane_ids = {};

    path.header.frame_id = "map";
    path.points.push_back(path_point_with_lane_id);
  }

  return path;
}

Pose generateEgoSamplePose(float && p_x, float && p_y, float && p_z)
{
  Pose pose;
  pose.position.x = p_x;
  pose.position.y = p_y;
  pose.position.z = p_z;
  return pose;
}
}  // namespace

TEST(BehaviorPathPlanningTurnSignal, Condition1)
{
  PathWithLaneId path = generateStraightSamplePathWithLaneId(0.0f, 1.0f, 70u);
  TurnSignalDecider turn_signal_decider;
  turn_signal_decider.setParameters(1.0, 30.0, 3.0, 15.0);

  TurnSignalInfo intersection_signal_info;
  intersection_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
  intersection_signal_info.desired_start_point.position = createPoint(0.0, 0.0, 0.0);
  intersection_signal_info.desired_start_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.desired_end_point.position = createPoint(65.0, 0.0, 0.0);
  intersection_signal_info.desired_end_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.required_start_point.position = createPoint(35.0, 0.0, 0.0);
  intersection_signal_info.required_start_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.required_end_point.position = createPoint(48.0, 0.0, 0.0);
  intersection_signal_info.required_end_point.orientation = createQuaternionFromYaw(0.0);

  TurnSignalInfo behavior_signal_info;
  behavior_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  behavior_signal_info.desired_start_point.position = createPoint(5.0, 0.0, 0.0);
  behavior_signal_info.desired_start_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.desired_end_point.position = createPoint(70.0, 0.0, 0.0);
  behavior_signal_info.desired_end_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.required_start_point.position = createPoint(45.0, 0.0, 0.0);
  behavior_signal_info.required_start_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.required_end_point.position = createPoint(50.0, 0.0, 0.0);
  behavior_signal_info.required_end_point.orientation = createQuaternionFromYaw(0.0);

  // current pose on the behavior desired start
  {
    Pose current_pose = generateEgoSamplePose(5.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right before the intersection required start
  {
    Pose current_pose = generateEgoSamplePose(34.99f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is within the intersection required section
  {
    Pose current_pose = generateEgoSamplePose(40.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is within the intersection and behavior required section
  {
    Pose current_pose = generateEgoSamplePose(45.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is on the intersection required end
  {
    Pose current_pose = generateEgoSamplePose(48.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right after the intersection required end
  {
    Pose current_pose = generateEgoSamplePose(48.1f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is on the behavior required end
  {
    Pose current_pose = generateEgoSamplePose(50.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is right after the behavior required end
  {
    Pose current_pose = generateEgoSamplePose(50.1f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is right on the intersection desired end
  {
    Pose current_pose = generateEgoSamplePose(65.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is right right after the intersection desired end
  {
    Pose current_pose = generateEgoSamplePose(65.1f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }
}

TEST(BehaviorPathPlanningTurnSignal, Condition2)
{
  PathWithLaneId path = generateStraightSamplePathWithLaneId(0.0f, 1.0f, 70u);
  TurnSignalDecider turn_signal_decider;
  turn_signal_decider.setParameters(1.0, 30.0, 3.0, 15.0);

  TurnSignalInfo intersection_signal_info;
  intersection_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
  intersection_signal_info.desired_start_point.position = createPoint(0.0, 0.0, 0.0);
  intersection_signal_info.desired_start_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.desired_end_point.position = createPoint(65.0, 0.0, 0.0);
  intersection_signal_info.desired_end_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.required_start_point.position = createPoint(35.0, 0.0, 0.0);
  intersection_signal_info.required_start_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.required_end_point.position = createPoint(50.0, 0.0, 0.0);
  intersection_signal_info.required_end_point.orientation = createQuaternionFromYaw(0.0);

  TurnSignalInfo behavior_signal_info;
  behavior_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  behavior_signal_info.desired_start_point.position = createPoint(5.0, 0.0, 0.0);
  behavior_signal_info.desired_start_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.desired_end_point.position = createPoint(70.0, 0.0, 0.0);
  behavior_signal_info.desired_end_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.required_start_point.position = createPoint(40.0, 0.0, 0.0);
  behavior_signal_info.required_start_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.required_end_point.position = createPoint(45.0, 0.0, 0.0);
  behavior_signal_info.required_end_point.orientation = createQuaternionFromYaw(0.0);

  // current pose on the behavior desired start
  {
    Pose current_pose = generateEgoSamplePose(5.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right before the intersection required start
  {
    Pose current_pose = generateEgoSamplePose(34.99f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is on the behavior required start
  {
    Pose current_pose = generateEgoSamplePose(40.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is on the behavior required end
  {
    Pose current_pose = generateEgoSamplePose(45.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is on the intersection required end
  {
    Pose current_pose = generateEgoSamplePose(50.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right after the intersection required end
  {
    Pose current_pose = generateEgoSamplePose(50.1f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is on the intersection desired end
  {
    Pose current_pose = generateEgoSamplePose(65.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }
}

TEST(BehaviorPathPlanningTurnSignal, Condition3)
{
  PathWithLaneId path = generateStraightSamplePathWithLaneId(0.0f, 1.0f, 70u);
  TurnSignalDecider turn_signal_decider;
  turn_signal_decider.setParameters(1.0, 30.0, 3.0, 15.0);

  TurnSignalInfo intersection_signal_info;
  intersection_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
  intersection_signal_info.desired_start_point.position = createPoint(0.0, 0.0, 0.0);
  intersection_signal_info.desired_start_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.desired_end_point.position = createPoint(65.0, 0.0, 0.0);
  intersection_signal_info.desired_end_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.required_start_point.position = createPoint(35.0, 0.0, 0.0);
  intersection_signal_info.required_start_point.orientation = createQuaternionFromYaw(0.0);
  intersection_signal_info.required_end_point.position = createPoint(50.0, 0.0, 0.0);
  intersection_signal_info.required_end_point.orientation = createQuaternionFromYaw(0.0);

  TurnSignalInfo behavior_signal_info;
  behavior_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  behavior_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  behavior_signal_info.desired_start_point.position = createPoint(5.0, 0.0, 0.0);
  behavior_signal_info.desired_start_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.desired_end_point.position = createPoint(70.0, 0.0, 0.0);
  behavior_signal_info.desired_end_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.required_start_point.position = createPoint(30.0, 0.0, 0.0);
  behavior_signal_info.required_start_point.orientation = createQuaternionFromYaw(0.0);
  behavior_signal_info.required_end_point.position = createPoint(45.0, 0.0, 0.0);
  behavior_signal_info.required_end_point.orientation = createQuaternionFromYaw(0.0);

  // current pose on the behavior desired start
  {
    Pose current_pose = generateEgoSamplePose(5.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is right before the behavior required start
  {
    Pose current_pose = generateEgoSamplePose(29.9f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is right on the behavior required start
  {
    Pose current_pose = generateEgoSamplePose(30.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is right before the intersection required start
  {
    Pose current_pose = generateEgoSamplePose(33.9f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_RIGHT);
  }

  // current pose is right on the intersection required start
  {
    Pose current_pose = generateEgoSamplePose(35.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right before the behavior required end
  {
    Pose current_pose = generateEgoSamplePose(44.9f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right on the behavior required end
  {
    Pose current_pose = generateEgoSamplePose(45.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right before the intersection required end
  {
    Pose current_pose = generateEgoSamplePose(49.9f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right on the intersection required end
  {
    Pose current_pose = generateEgoSamplePose(50.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }

  // current pose is right on the intersection desired end
  {
    Pose current_pose = generateEgoSamplePose(65.0f, 0.0f, 0.0);
    const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, 3.0, 1.0);
    const auto result_signal = turn_signal_decider.resolve_turn_signal(
      path, current_pose, current_seg_idx, intersection_signal_info, behavior_signal_info,
      nearest_dist_threshold, nearest_yaw_threshold);
    EXPECT_EQ(result_signal.command, TurnIndicatorsCommand::ENABLE_LEFT);
  }
}
