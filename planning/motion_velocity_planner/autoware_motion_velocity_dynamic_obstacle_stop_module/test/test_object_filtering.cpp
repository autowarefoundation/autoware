// Copyright 2024 TIER IV, Inc.
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

#include "../src/object_filtering.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/LineString.h>

TEST(TestObjectFiltering, isVehicle)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::is_vehicle;
  using autoware_perception_msgs::msg::ObjectClassification;
  autoware_perception_msgs::msg::PredictedObject object;
  EXPECT_NO_THROW(is_vehicle(object));
  ObjectClassification classification;
  object.classification = {};
  EXPECT_FALSE(is_vehicle(object));
  classification.label = ObjectClassification::PEDESTRIAN;
  object.classification.push_back(classification);
  EXPECT_FALSE(is_vehicle(object));
  classification.label = ObjectClassification::UNKNOWN;
  object.classification.push_back(classification);
  EXPECT_FALSE(is_vehicle(object));
  classification.label = ObjectClassification::TRAILER;
  object.classification.push_back(classification);
  EXPECT_TRUE(is_vehicle(object));
  object.classification.clear();
  for (const auto label :
       {ObjectClassification::CAR, ObjectClassification::BUS, ObjectClassification::BICYCLE,
        ObjectClassification::TRUCK, ObjectClassification::TRAILER,
        ObjectClassification::MOTORCYCLE}) {
    classification.label = label;
    object.classification = {classification};
    EXPECT_TRUE(is_vehicle(object));
  }
}

TEST(TestObjectFiltering, isInRange)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::is_in_range;
  autoware_perception_msgs::msg::PredictedObject object;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::TrajectoryPoints ego_trajectory;
  autoware_planning_msgs::msg::TrajectoryPoint trajectory_p;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::PlannerParam params;
  EXPECT_NO_THROW(is_in_range(object, ego_trajectory, params, {}));
  trajectory_p.pose.position.y = 0.0;
  for (auto x = -10.0; x <= 10.0; x += 1.0) {
    trajectory_p.pose.position.x = x;
    ego_trajectory.push_back(trajectory_p);
  }
  // object 4m from the ego trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 4.0;
  object.shape.dimensions.y = 2.0;
  params.minimum_object_distance_from_ego_trajectory = 1.0;
  params.ego_lateral_offset = 1.0;
  double hysteresis = 0.0;
  EXPECT_FALSE(is_in_range(object, ego_trajectory, params, hysteresis));
  // object 3m from the ego trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.y = 3.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  hysteresis = 1.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  // object 2m from the ego trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  hysteresis = 0.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  hysteresis = 1.0;
  EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
  // object exactly on the trajectory
  object.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  for (auto ego_lat_offset = 0.1; ego_lat_offset < 2.0; ego_lat_offset += 0.1) {
    for (auto object_width = 0.1; object_width < 2.0; object_width += 0.1) {
      for (auto min_dist = 0.1; min_dist < 2.0; min_dist += 0.1) {
        for (hysteresis = 0.1; hysteresis < 2.0; hysteresis += 0.1) {
          params.ego_lateral_offset = ego_lat_offset;
          object.shape.dimensions.y = object_width;
          params.minimum_object_distance_from_ego_trajectory = min_dist;
          EXPECT_TRUE(is_in_range(object, ego_trajectory, params, hysteresis));
        }
      }
    }
  }
}

TEST(TestObjectFiltering, isNotTooClose)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::is_not_too_close;
  autoware_perception_msgs::msg::PredictedObject object;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::EgoData ego_data;
  EXPECT_NO_THROW(is_not_too_close(object, ego_data, {}));

  double ego_longitudinal_offset = 1.0;
  ego_data.longitudinal_offset_to_first_trajectory_idx = 0.0;
  ego_data.pose.position.x = 0.0;
  ego_data.pose.position.y = 0.0;
  autoware_planning_msgs::msg::TrajectoryPoint trajectory_p;
  trajectory_p.pose.position.y = 0.0;
  for (auto x = -10.0; x <= 10.0; x += 1.0) {
    trajectory_p.pose.position.x = x;
    ego_data.trajectory.push_back(trajectory_p);
  }
  object.shape.dimensions.x = 2.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  // object ego with 1m offset = too close if poses are within 2m of arc length
  for (auto obj_x = -2.0; obj_x <= 2.0; obj_x += 0.1) {
    object.kinematics.initial_pose_with_covariance.pose.position.x = obj_x;
    EXPECT_FALSE(is_not_too_close(object, ego_data, ego_longitudinal_offset));
  }
  for (auto obj_x = -2.1; obj_x >= -10.0; obj_x -= 0.1) {
    object.kinematics.initial_pose_with_covariance.pose.position.x = obj_x;
    EXPECT_TRUE(is_not_too_close(object, ego_data, ego_longitudinal_offset));
  }
  for (auto obj_x = 2.1; obj_x <= 10.0; obj_x += 0.1) {
    object.kinematics.initial_pose_with_covariance.pose.position.x = obj_x;
    EXPECT_TRUE(is_not_too_close(object, ego_data, ego_longitudinal_offset));
  }
}

TEST(TestObjectFiltering, isUnavoidable)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::is_unavoidable;
  autoware_perception_msgs::msg::PredictedObject object;
  geometry_msgs::msg::Pose ego_pose;
  std::optional<geometry_msgs::msg::Pose> ego_earliest_stop_pose;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::PlannerParam params;
  EXPECT_NO_THROW(is_unavoidable(object, ego_pose, ego_earliest_stop_pose, params));

  params.ego_lateral_offset = 1.0;
  params.hysteresis = 0.0;

  object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  object.shape.dimensions.y = 2.0;

  ego_pose.position.x = 0.0;
  ego_pose.position.y = 0.0;
  // ego and object heading in the same direction -> not unavoidable
  for (auto ego_yaw = -0.4; ego_yaw <= 0.4; ego_yaw += 0.1) {
    ego_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(ego_yaw);
    for (auto obj_yaw = -0.4; obj_yaw <= 0.4; obj_yaw += 0.1) {
      object.kinematics.initial_pose_with_covariance.pose.orientation =
        autoware::universe_utils::createQuaternionFromYaw(obj_yaw);
      EXPECT_FALSE(is_unavoidable(object, ego_pose, ego_earliest_stop_pose, params));
    }
  }
  // ego and object heading in opposite direction -> unavoidable
  for (auto ego_yaw = -0.4; ego_yaw <= 0.4; ego_yaw += 0.1) {
    ego_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(ego_yaw);
    object.kinematics.initial_pose_with_covariance.pose.orientation =
      autoware::universe_utils::createQuaternionFromYaw(ego_yaw + M_PI);
    EXPECT_TRUE(is_unavoidable(object, ego_pose, ego_earliest_stop_pose, params));
  }

  // shift the object : even if they drive in opposite direction they are no longer aligned
  object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 5.0;
  for (auto ego_yaw = -0.4; ego_yaw <= 0.4; ego_yaw += 0.1) {
    ego_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(ego_yaw);
    object.kinematics.initial_pose_with_covariance.pose.orientation =
      autoware::universe_utils::createQuaternionFromYaw(ego_yaw + M_PI);
    EXPECT_FALSE(is_unavoidable(object, ego_pose, ego_earliest_stop_pose, params));
  }

  // perpendicular case
  object.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware::universe_utils::createQuaternionFromYaw(-M_PI_2);
  ego_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(0.0);
  EXPECT_FALSE(is_unavoidable(object, ego_pose, ego_earliest_stop_pose, params));
  // with earliest stop pose away from the object path -> no collision
  ego_earliest_stop_pose.emplace();
  ego_earliest_stop_pose->position.x = 2.0;
  ego_earliest_stop_pose->position.y = 0.0;
  EXPECT_FALSE(is_unavoidable(object, ego_pose, ego_earliest_stop_pose, params));
  // with earliest stop pose in on the object path (including the ego and object sizes) -> collision
  for (auto x = 3.1; x < 7.0; x += 0.1) {
    ego_earliest_stop_pose->position.x = x;
    EXPECT_TRUE(is_unavoidable(object, ego_pose, ego_earliest_stop_pose, params));
  }
}

TEST(TestObjectFiltering, filterPredictedObjects)
{
  using autoware::motion_velocity_planner::dynamic_obstacle_stop::filter_predicted_objects;
  autoware_perception_msgs::msg::PredictedObjects objects;
  autoware_perception_msgs::msg::PredictedObject object;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::EgoData ego_data;
  autoware::motion_velocity_planner::dynamic_obstacle_stop::PlannerParam params;
  double hysteresis{};
  EXPECT_NO_THROW(filter_predicted_objects(objects, ego_data, params, hysteresis));
}
