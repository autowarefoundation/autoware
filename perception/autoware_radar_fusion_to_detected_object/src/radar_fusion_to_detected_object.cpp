
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

#include "include/radar_fusion_to_detected_object.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/normalization.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace autoware::radar_fusion_to_detected_object
{
using autoware::universe_utils::LinearRing2d;
using autoware::universe_utils::Point2d;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;

void RadarFusionToDetectedObject::setParam(const Param & param)
{
  // Radar fusion param
  param_.bounding_box_margin = param.bounding_box_margin;
  param_.split_threshold_velocity = param.split_threshold_velocity;
  param_.threshold_yaw_diff = param.threshold_yaw_diff;

  // Normalize weight param
  double sum_weight = param.velocity_weight_median + param.velocity_weight_min_distance +
                      param.velocity_weight_average + param.velocity_weight_target_value_average +
                      param.velocity_weight_target_value_top;

  if (sum_weight < 0.01) {
    param_.velocity_weight_min_distance = 1.0;
    param_.velocity_weight_median = 0.0;
    param_.velocity_weight_average = 0.0;
    param_.velocity_weight_target_value_average = 0.0;
    param_.velocity_weight_target_value_top = 0.0;
  } else {
    param_.velocity_weight_min_distance = param.velocity_weight_min_distance / sum_weight;
    param_.velocity_weight_median = param.velocity_weight_median / sum_weight;
    param_.velocity_weight_average = param.velocity_weight_average / sum_weight;
    param_.velocity_weight_target_value_average =
      param.velocity_weight_target_value_average / sum_weight;
    param_.velocity_weight_target_value_top = param.velocity_weight_target_value_top / sum_weight;
  }

  // Parameters for fixing object information
  param_.threshold_probability = param.threshold_probability;
  param_.convert_doppler_to_twist = param.convert_doppler_to_twist;
  param_.compensate_probability = param.compensate_probability;
}

RadarFusionToDetectedObject::Output RadarFusionToDetectedObject::update(
  const RadarFusionToDetectedObject::Input & input)
{
  RadarFusionToDetectedObject::Output output{};
  output.objects.header = input.objects->header;

  if (!input.objects || input.objects->objects.empty()) {
    return output;
  }

  for (auto & object : input.objects->objects) {
    // Link between 3d bounding box and radar data
    std::shared_ptr<std::vector<RadarInput>> radars_within_object =
      filterRadarWithinObject(object, input.radars);

    // TODO(Satoshi Tanaka): Implement
    // Split the object going in a different direction
    // std::vector<DetectedObject> split_objects = splitObject(object, radars_within_object);
    std::vector<DetectedObject> split_objects;
    split_objects.emplace_back(object);

    for (auto & split_object : split_objects) {
      // set radars within objects
      std::shared_ptr<std::vector<RadarInput>> radars_within_split_object;

      // cppcheck-suppress knownConditionTrueFalse
      if (split_objects.size() == 1) {
        // If object is not split, radar data within object is same
        radars_within_split_object = radars_within_object;
      } else {
        // If object is split, then filter radar again
        radars_within_split_object = filterRadarWithinObject(split_object, radars_within_object);
      }

      // Estimate twist of object
      if (!radars_within_split_object || !(*radars_within_split_object).empty()) {
        TwistWithCovariance twist_with_covariance =
          estimateTwist(split_object, radars_within_split_object);

        if (isYawCorrect(split_object, twist_with_covariance, param_.threshold_yaw_diff)) {
          split_object.kinematics.twist_with_covariance = twist_with_covariance;
          split_object.kinematics.has_twist = true;
          if (hasTwistCovariance(twist_with_covariance)) {
            split_object.kinematics.has_twist_covariance = true;
          }
        }
      }

      // Delete objects with low probability
      if (isQualified(split_object, radars_within_split_object)) {
        if (param_.compensate_probability) {
          split_object.existence_probability =
            std::max(split_object.existence_probability, param_.threshold_probability);
        }
        output.objects.objects.emplace_back(split_object);
      } else {
        output.debug_low_confidence_objects.objects.emplace_back(split_object);
      }
    }
  }
  return output;
}

// Judge whether twist covariance is available.
bool RadarFusionToDetectedObject::hasTwistCovariance(
  const TwistWithCovariance & twist_with_covariance)
{
  using IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  auto covariance = twist_with_covariance.covariance;
  if (covariance[IDX::X_X] == 0.0 && covariance[IDX::Y_Y] == 0.0 && covariance[IDX::Z_Z] == 0.0) {
    return false;
  } else {
    return true;
  }
}

// Judge whether object's yaw is same direction with twist's yaw.
// This function improve multi object tracking with observed speed.
bool RadarFusionToDetectedObject::isYawCorrect(
  const DetectedObject & object, const TwistWithCovariance & twist_with_covariance,
  const double & yaw_threshold)
{
  const double twist_yaw = autoware::universe_utils::normalizeRadian(
    std::atan2(twist_with_covariance.twist.linear.y, twist_with_covariance.twist.linear.x));
  const double object_yaw = autoware::universe_utils::normalizeRadian(
    tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation));
  const double diff_yaw = autoware::universe_utils::normalizeRadian(twist_yaw - object_yaw);
  if (std::abs(diff_yaw) < yaw_threshold) {
    return true;
  } else if (M_PI - yaw_threshold < std::abs(diff_yaw)) {
    return true;
  } else {
    return false;
  }
}

// Choose radar pointcloud/objects within 3D bounding box from lidar-base detection with margin
// space from bird's-eye view.
std::shared_ptr<std::vector<RadarFusionToDetectedObject::RadarInput>>
RadarFusionToDetectedObject::filterRadarWithinObject(
  const DetectedObject & object,
  const std::shared_ptr<std::vector<RadarFusionToDetectedObject::RadarInput>> & radars)
{
  std::vector<RadarInput> outputs{};

  autoware::universe_utils::Point2d object_size{
    object.shape.dimensions.x, object.shape.dimensions.y};
  LinearRing2d object_box = createObject2dWithMargin(object_size, param_.bounding_box_margin);
  object_box = autoware::universe_utils::transformVector(
    object_box,
    autoware::universe_utils::pose2transform(object.kinematics.pose_with_covariance.pose));

  for (const auto & radar : (*radars)) {
    Point2d radar_point{
      radar.pose_with_covariance.pose.position.x, radar.pose_with_covariance.pose.position.y};
    if (boost::geometry::within(radar_point, object_box)) {
      outputs.emplace_back(radar);
    }
  }
  return std::make_shared<std::vector<RadarFusionToDetectedObject::RadarInput>>(outputs);
}

// TODO(Satoshi Tanaka): Implementation
// std::vector<DetectedObject> RadarFusionToDetectedObject::splitObject(
//   const DetectedObject & object, const std::vector<RadarInput> & radars)
// {
//   std::vector<DetectedObject> output{};
//   return output;
// }

// Estimate twist from chosen radar pointcloud/objects using twist and target value
// (Target value is amplitude if using radar pointcloud. Target value is probability if using radar
// objects).
TwistWithCovariance RadarFusionToDetectedObject::estimateTwist(
  const DetectedObject & object, const std::shared_ptr<std::vector<RadarInput>> & radars) const
{
  if (!radars || (*radars).empty()) {
    TwistWithCovariance output{};
    return output;
  }

  // calculate twist for radar data with min distance
  Eigen::Vector2d vec_min_distance(0.0, 0.0);
  if (param_.velocity_weight_min_distance > 0.0) {
    auto comp_func = [&](const RadarInput & a, const RadarInput & b) {
      return autoware::universe_utils::calcSquaredDistance2d(
               a.pose_with_covariance.pose.position,
               object.kinematics.pose_with_covariance.pose.position) <
             autoware::universe_utils::calcSquaredDistance2d(
               b.pose_with_covariance.pose.position,
               object.kinematics.pose_with_covariance.pose.position);
    };
    auto iter = std::min_element(std::begin(*radars), std::end(*radars), comp_func);
    TwistWithCovariance twist_min_distance = iter->twist_with_covariance;
    vec_min_distance = toVector2d(twist_min_distance);
  }

  // calculate twist for radar data with median twist
  Eigen::Vector2d vec_median(0.0, 0.0);
  if (param_.velocity_weight_median > 0.0) {
    auto ascending_func = [&](const RadarInput & a, const RadarInput & b) {
      return getTwistNorm(a.twist_with_covariance.twist) <
             getTwistNorm(b.twist_with_covariance.twist);
    };
    std::sort((*radars).begin(), (*radars).end(), ascending_func);

    if ((*radars).size() % 2 == 1) {
      int median_index = ((*radars).size() - 1) / 2;
      vec_median = toVector2d((*radars).at(median_index).twist_with_covariance);
    } else {
      int median_index = (*radars).size() / 2;
      Eigen::Vector2d v1 = toVector2d((*radars).at(median_index - 1).twist_with_covariance);
      Eigen::Vector2d v2 = toVector2d((*radars).at(median_index).twist_with_covariance);
      vec_median = (v1 + v2) / 2.0;
    }
  }

  // calculate twist for radar data with average twist
  Eigen::Vector2d vec_average(0.0, 0.0);
  if (param_.velocity_weight_average > 0.0) {
    for (const auto & radar : (*radars)) {
      vec_average += toVector2d(radar.twist_with_covariance);
    }
    vec_average /= (*radars).size();
  }

  // calculate twist for radar data with top target value
  Eigen::Vector2d vec_top_target_value(0.0, 0.0);
  auto comp_func = [](const RadarInput & a, const RadarInput & b) {
    return a.target_value < b.target_value;
  };
  auto iter = std::max_element(std::begin((*radars)), std::end((*radars)), comp_func);
  if (param_.velocity_weight_target_value_top > 0.0) {
    vec_top_target_value = toVector2d(iter->twist_with_covariance);
  }
  // Get covariance values
  auto twist_covariance = iter->twist_with_covariance.covariance;

  // calculate twist for radar data with target_value * average
  Eigen::Vector2d vec_target_value_average(0.0, 0.0);
  if (param_.velocity_weight_target_value_average > 0.0) {
    double sum_target_value = 0.0;
    for (const auto & radar : (*radars)) {
      vec_target_value_average += (toVector2d(radar.twist_with_covariance) * radar.target_value);
      sum_target_value += radar.target_value;
    }
    vec_target_value_average /= sum_target_value;
  }

  Eigen::Vector2d sum_vec = vec_min_distance * param_.velocity_weight_min_distance +
                            vec_median * param_.velocity_weight_median +
                            vec_average * param_.velocity_weight_average +
                            vec_top_target_value * param_.velocity_weight_target_value_top +
                            vec_target_value_average * param_.velocity_weight_target_value_average;
  TwistWithCovariance estimated_twist_with_covariance = toTwistWithCovariance(sum_vec);
  estimated_twist_with_covariance.covariance = twist_covariance;

  // TODO(Satoshi Tanaka): Implement
  // Convert doppler velocity to twist
  // if (param_.convert_doppler_to_twist) {
  //   twist_with_covariance = convertDopplerToTwist(object, twist_with_covariance);
  // }
  return estimated_twist_with_covariance;
}

// Judge whether low confidence objects that do not have some radar points/objects or not.
bool RadarFusionToDetectedObject::isQualified(
  const DetectedObject & object, const std::shared_ptr<std::vector<RadarInput>> & radars) const
{
  if (object.existence_probability > param_.threshold_probability) {
    return true;
  } else {
    if (!radars || !(*radars).empty()) {
      return true;
    } else {
      return false;
    }
  }
}

// TODO(Satoshi Tanaka): Implement for radar pointcloud fusion
// TwistWithCovariance RadarFusionToDetectedObject::convertDopplerToTwist(
//   const DetectedObject & object, const TwistWithCovariance & twist_with_covariance)
// {
//   return twist_with_covariance;
// }

Eigen::Vector2d RadarFusionToDetectedObject::toVector2d(
  const TwistWithCovariance & twist_with_covariance)
{
  auto vec = twist_with_covariance.twist.linear;
  Eigen::Vector2d output(vec.x, vec.y);
  return output;
}

TwistWithCovariance RadarFusionToDetectedObject::toTwistWithCovariance(
  const Eigen::Vector2d & vector2d)
{
  TwistWithCovariance output{};
  output.twist.linear.x = vector2d(0);
  output.twist.linear.y = vector2d(1);
  return output;
}

double RadarFusionToDetectedObject::getTwistNorm(const Twist & twist)
{
  double output = std::sqrt(
    twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y +
    twist.linear.z * twist.linear.z);
  return output;
}

LinearRing2d RadarFusionToDetectedObject::createObject2dWithMargin(
  const Point2d object_size, const double margin)
{
  const double x_front = object_size.x() / 2.0 + margin;
  const double x_rear = -object_size.x() / 2.0 - margin;
  const double y_left = object_size.y() / 2.0 + margin;
  const double y_right = -object_size.y() / 2.0 - margin;

  LinearRing2d box{};
  box.push_back(Point2d{x_front, y_left});
  box.push_back(Point2d{x_front, y_right});
  box.push_back(Point2d{x_rear, y_right});
  box.push_back(Point2d{x_rear, y_left});
  box.push_back(Point2d{x_front, y_left});

  return box;
}
}  // namespace autoware::radar_fusion_to_detected_object
