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

#ifndef MAP_BASED_PREDICTION__PATH_GENERATOR_HPP_
#define MAP_BASED_PREDICTION__PATH_GENERATOR_HPP_

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace map_based_prediction
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjectKinematics;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjectKinematics;
using autoware_auto_perception_msgs::msg::TrackedObjects;

struct FrenetPoint
{
  double s;
  double d;
  float s_vel;
  float d_vel;
  float s_acc;
  float d_acc;
};

struct CrosswalkEdgePoints
{
  Eigen::Vector2d front_center_point;
  Eigen::Vector2d front_right_point;
  Eigen::Vector2d front_left_point;
  Eigen::Vector2d back_center_point;
  Eigen::Vector2d back_right_point;
  Eigen::Vector2d back_left_point;

  void swap()
  {
    const Eigen::Vector2d tmp_center_point = front_center_point;
    const Eigen::Vector2d tmp_right_point = front_right_point;
    const Eigen::Vector2d tmp_left_point = front_left_point;

    front_center_point = back_center_point;
    front_right_point = back_right_point;
    front_left_point = back_left_point;

    back_center_point = tmp_center_point;
    back_right_point = tmp_right_point;
    back_left_point = tmp_left_point;
  }
};

using FrenetPath = std::vector<FrenetPoint>;
using PosePath = std::vector<geometry_msgs::msg::Pose>;

class PathGenerator
{
public:
  PathGenerator(
    const double time_horizon, const double lateral_time_horizon,
    const double sampling_time_interval, const double min_crosswalk_user_velocity);

  PredictedPath generatePathForNonVehicleObject(const TrackedObject & object);

  PredictedPath generatePathForLowSpeedVehicle(const TrackedObject & object) const;

  PredictedPath generatePathForOffLaneVehicle(const TrackedObject & object);

  PredictedPath generatePathForOnLaneVehicle(
    const TrackedObject & object, const PosePath & ref_paths);

  PredictedPath generatePathForCrosswalkUser(
    const TrackedObject & object, const CrosswalkEdgePoints & reachable_crosswalk) const;

  PredictedPath generatePathToTargetPoint(
    const TrackedObject & object, const Eigen::Vector2d & point) const;

private:
  // Parameters
  double time_horizon_;
  double lateral_time_horizon_;
  double sampling_time_interval_;
  double min_crosswalk_user_velocity_;

  // Member functions
  PredictedPath generateStraightPath(const TrackedObject & object) const;

  PredictedPath generatePolynomialPath(const TrackedObject & object, const PosePath & ref_path);

  FrenetPath generateFrenetPath(
    const FrenetPoint & current_point, const FrenetPoint & target_point, const double max_length);
  Eigen::Vector3d calcLatCoefficients(
    const FrenetPoint & current_point, const FrenetPoint & target_point, const double T);
  Eigen::Vector2d calcLonCoefficients(
    const FrenetPoint & current_point, const FrenetPoint & target_point, const double T);

  PosePath interpolateReferencePath(
    const PosePath & base_path, const FrenetPath & frenet_predicted_path);

  PredictedPath convertToPredictedPath(
    const TrackedObject & object, const FrenetPath & frenet_predicted_path,
    const PosePath & ref_path);

  FrenetPoint getFrenetPoint(const TrackedObject & object, const PosePath & ref_path);
};
}  // namespace map_based_prediction

#endif  // MAP_BASED_PREDICTION__PATH_GENERATOR_HPP_
