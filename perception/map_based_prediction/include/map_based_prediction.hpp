// Copyright 2018-2019 Autoware Foundation
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

#ifndef MAP_BASED_PREDICTION_HPP_
#define MAP_BASED_PREDICTION_HPP_

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

struct DynamicObjectWithLanes
{
  autoware_auto_perception_msgs::msg::TrackedObject object;
  std::vector<std::vector<geometry_msgs::msg::Pose>> lanes;
  std::vector<double> confidence;
};

struct DynamicObjectWithLanesArray
{
  std_msgs::msg::Header header;
  std::vector<DynamicObjectWithLanes> objects;
};

class Spline2D;

class MapBasedPrediction
{
private:
  double interpolating_resolution_;
  double time_horizon_;
  double sampling_delta_time_;

  bool getPredictedPath(
    const double height, const double current_d_position, const double current_d_velocity,
    const double current_s_position, const double current_s_velocity,
    const std_msgs::msg::Header & origin_header, Spline2D & spline2d,
    autoware_auto_perception_msgs::msg::PredictedPath & path) const;

  void getLinearPredictedPath(
    const geometry_msgs::msg::Pose & object_pose, const geometry_msgs::msg::Twist & object_twist,
    autoware_auto_perception_msgs::msg::PredictedPath & predicted_path) const;

  void normalizeLikelihood(
    autoware_auto_perception_msgs::msg::PredictedObjectKinematics & predicted_object_kinematics);

  autoware_auto_perception_msgs::msg::PredictedObjectKinematics convertToPredictedKinematics(
    const autoware_auto_perception_msgs::msg::TrackedObjectKinematics & tracked_object);

public:
  MapBasedPrediction(
    double interpolating_resolution, double time_horizon, double sampling_delta_time);

  bool doPrediction(
    const DynamicObjectWithLanesArray & in_objects,
    std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & out_objects);

  bool doLinearPrediction(
    const autoware_auto_perception_msgs::msg::PredictedObjects & in_objects,
    std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & out_objects);

  autoware_auto_perception_msgs::msg::PredictedObject convertToPredictedObject(
    const autoware_auto_perception_msgs::msg::TrackedObject & tracked_object);
};

#endif  // MAP_BASED_PREDICTION_HPP_
