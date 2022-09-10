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

#ifndef RADAR_FUSION_TO_DETECTED_OBJECT_HPP_
#define RADAR_FUSION_TO_DETECTED_OBJECT_HPP_

#include "rclcpp/logger.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
// #include "std_msgs/msg/header.hpp"

#include <memory>
#include <string>
#include <vector>

namespace radar_fusion_to_detected_object
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TwistWithCovariance;
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

enum MSG_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};

class RadarFusionToDetectedObject
{
public:
  explicit RadarFusionToDetectedObject(const rclcpp::Logger & logger) : logger_(logger) {}

  struct Param
  {
    // Radar fusion param
    double bounding_box_margin{};
    double split_threshold_velocity{};
    double threshold_yaw_diff{};

    // Weight param for velocity estimation
    double velocity_weight_average{};
    double velocity_weight_median{};
    double velocity_weight_min_distance{};
    double velocity_weight_target_value_average{};
    double velocity_weight_target_value_top{};

    // Parameters for fixed object information
    bool convert_doppler_to_twist{};
    float threshold_probability{};
    bool compensate_probability{};
  };

  struct RadarInput
  {
    std_msgs::msg::Header header{};
    PoseWithCovariance pose_with_covariance{};
    TwistWithCovariance twist_with_covariance{};
    double target_value{};
  };

  struct Input
  {
    std::shared_ptr<std::vector<RadarInput>> radars{};
    DetectedObjects::ConstSharedPtr objects{};
  };

  struct Output
  {
    DetectedObjects objects{};
    DetectedObjects debug_low_confidence_objects{};
  };

  void setParam(const Param & param);
  Output update(const Input & input);

private:
  rclcpp::Logger logger_;
  Param param_{};
  std::shared_ptr<std::vector<RadarInput>> filterRadarWithinObject(
    const DetectedObject & object, const std::shared_ptr<std::vector<RadarInput>> & radars);
  // TODO(Satoshi Tanaka): Implement
  // std::vector<DetectedObject> splitObject(
  //   const DetectedObject & object, const std::shared_ptr<std::vector<RadarInput>> & radars);
  TwistWithCovariance estimateTwist(
    const DetectedObject & object, std::shared_ptr<std::vector<RadarInput>> & radars);
  bool isQualified(
    const DetectedObject & object, std::shared_ptr<std::vector<RadarInput>> & radars);
  TwistWithCovariance convertDopplerToTwist(
    const DetectedObject & object, const TwistWithCovariance & twist_with_covariance);
  bool isYawCorrect(
    const DetectedObject & object, const TwistWithCovariance & twist_with_covariance,
    const double & yaw_threshold);
  bool hasTwistCovariance(const TwistWithCovariance & twist_with_covariance);
  Eigen::Vector2d toVector2d(const TwistWithCovariance & twist_with_covariance);
  TwistWithCovariance toTwistWithCovariance(const Eigen::Vector2d & vector2d);

  double getTwistNorm(const Twist & twist);
  LinearRing2d createObject2dWithMargin(const Point2d object_size, const double margin);
};
}  // namespace radar_fusion_to_detected_object

#endif  // RADAR_FUSION_TO_DETECTED_OBJECT_HPP_
