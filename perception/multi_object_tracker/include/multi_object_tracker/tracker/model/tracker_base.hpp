// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_

#define EIGEN_MPL2_ONLY
#include "multi_object_tracker/utils/utils.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_object.hpp"
#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <vector>

class Tracker
{
private:
  unique_identifier_msgs::msg::UUID uuid_;

  // classification
  std::vector<autoware_perception_msgs::msg::ObjectClassification> classification_;

  // existence states
  int no_measurement_count_;
  int total_no_measurement_count_;
  int total_measurement_count_;
  rclcpp::Time last_update_with_measurement_time_;
  std::vector<float> existence_probabilities_;
  float total_existence_probability_;

public:
  Tracker(
    const rclcpp::Time & time,
    const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification,
    const size_t & channel_size);
  virtual ~Tracker() = default;

  void initializeExistenceProbabilities(
    const uint & channel_index, const float & existence_probability);
  bool getExistenceProbabilityVector(std::vector<float> & existence_vector) const
  {
    existence_vector = existence_probabilities_;
    return existence_vector.size() > 0;
  }
  bool updateWithMeasurement(
    const autoware_perception_msgs::msg::DetectedObject & object,
    const rclcpp::Time & measurement_time, const geometry_msgs::msg::Transform & self_transform,
    const uint & channel_index);
  bool updateWithoutMeasurement(const rclcpp::Time & now);

  // classification
  std::vector<autoware_perception_msgs::msg::ObjectClassification> getClassification() const
  {
    return classification_;
  }
  std::uint8_t getHighestProbLabel() const
  {
    return object_recognition_utils::getHighestProbLabel(classification_);
  }

  // existence states
  int getNoMeasurementCount() const { return no_measurement_count_; }
  int getTotalNoMeasurementCount() const { return total_no_measurement_count_; }
  int getTotalMeasurementCount() const { return total_measurement_count_; }
  double getElapsedTimeFromLastUpdate(const rclcpp::Time & current_time) const
  {
    return (current_time - last_update_with_measurement_time_).seconds();
  }

protected:
  unique_identifier_msgs::msg::UUID getUUID() const { return uuid_; }
  void setClassification(
    const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification)
  {
    classification_ = classification;
  }
  void updateClassification(
    const std::vector<autoware_perception_msgs::msg::ObjectClassification> & classification);

  // virtual functions
public:
  virtual geometry_msgs::msg::PoseWithCovariance getPoseWithCovariance(
    const rclcpp::Time & time) const;

protected:
  virtual bool measure(
    const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) = 0;

public:
  virtual bool getTrackedObject(
    const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObject & object) const = 0;
  virtual bool predict(const rclcpp::Time & time) = 0;
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__TRACKER_BASE_HPP_
