// Copyright 2024 TIER IV, inc.
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

#include "map_based_prediction/path_generator.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjectKinematics;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjectKinematics;
using autoware_auto_perception_msgs::msg::TrackedObjects;

TrackedObject generate_static_object(const int label)
{
  ObjectClassification classification;
  classification.probability = 1.0;
  classification.label = label;

  TrackedObjectKinematics kinematics;
  kinematics.pose_with_covariance = geometry_msgs::msg::PoseWithCovariance();    // At origin
  kinematics.twist_with_covariance = geometry_msgs::msg::TwistWithCovariance();  // Not moving
  kinematics.acceleration_with_covariance =
    geometry_msgs::msg::AccelWithCovariance();  // Not accelerating
  kinematics.orientation_availability = TrackedObjectKinematics::UNAVAILABLE;

  TrackedObject tracked_object;
  tracked_object.object_id = unique_identifier_msgs::msg::UUID();
  tracked_object.existence_probability = 1.0;
  tracked_object.classification.push_back(classification);
  tracked_object.kinematics = kinematics;

  return tracked_object;
}

TEST(PathGenerator, test_generatePathForNonVehicleObject)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double lateral_control_time_horizon = 5.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const map_based_prediction::PathGenerator path_generator = map_based_prediction::PathGenerator(
    prediction_time_horizon, lateral_control_time_horizon, prediction_sampling_time_interval,
    min_crosswalk_user_velocity);

  // Generate pedestrian object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::PEDESTRIAN);

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathForNonVehicleObject(tracked_object);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForLowSpeedVehicle)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double lateral_control_time_horizon = 5.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const map_based_prediction::PathGenerator path_generator = map_based_prediction::PathGenerator(
    prediction_time_horizon, lateral_control_time_horizon, prediction_sampling_time_interval,
    min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathForLowSpeedVehicle(tracked_object);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForOffLaneVehicle)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double lateral_control_time_horizon = 5.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const map_based_prediction::PathGenerator path_generator = map_based_prediction::PathGenerator(
    prediction_time_horizon, lateral_control_time_horizon, prediction_sampling_time_interval,
    min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  const PredictedPath predicted_path = path_generator.generatePathForOffLaneVehicle(tracked_object);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForOnLaneVehicle)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double lateral_control_time_horizon = 5.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const map_based_prediction::PathGenerator path_generator = map_based_prediction::PathGenerator(
    prediction_time_horizon, lateral_control_time_horizon, prediction_sampling_time_interval,
    min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  // Generate reference path
  map_based_prediction::PosePath ref_paths;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  ref_paths.push_back(pose);

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathForOnLaneVehicle(tracked_object, ref_paths);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathForCrosswalkUser)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double lateral_control_time_horizon = 5.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const map_based_prediction::PathGenerator path_generator = map_based_prediction::PathGenerator(
    prediction_time_horizon, lateral_control_time_horizon, prediction_sampling_time_interval,
    min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::PEDESTRIAN);

  // Generate dummy crosswalk
  map_based_prediction::CrosswalkEdgePoints reachable_crosswalk;
  reachable_crosswalk.front_center_point << 0.0, 0.0;
  reachable_crosswalk.front_right_point << 1.0, 0.0;
  reachable_crosswalk.front_left_point << -1.0, 0.0;
  reachable_crosswalk.back_center_point << 0.0, 1.0;
  reachable_crosswalk.back_right_point << 1.0, 1.0;
  reachable_crosswalk.back_left_point << -1.0, 1.0;

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathForCrosswalkUser(tracked_object, reachable_crosswalk);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}

TEST(PathGenerator, test_generatePathToTargetPoint)
{
  // Generate Path generator
  const double prediction_time_horizon = 10.0;
  const double lateral_control_time_horizon = 5.0;
  const double prediction_sampling_time_interval = 0.5;
  const double min_crosswalk_user_velocity = 0.1;
  const map_based_prediction::PathGenerator path_generator = map_based_prediction::PathGenerator(
    prediction_time_horizon, lateral_control_time_horizon, prediction_sampling_time_interval,
    min_crosswalk_user_velocity);

  // Generate dummy object
  TrackedObject tracked_object = generate_static_object(ObjectClassification::CAR);

  // Generate target point
  Eigen::Vector2d target_point;
  target_point << 0.0, 0.0;

  // Generate predicted path
  const PredictedPath predicted_path =
    path_generator.generatePathToTargetPoint(tracked_object, target_point);

  // Check
  EXPECT_FALSE(predicted_path.path.empty());
  EXPECT_EQ(predicted_path.path[0].position.x, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.y, 0.0);
  EXPECT_EQ(predicted_path.path[0].position.z, 0.0);
}
