// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__TRACKING_OBJECT_MERGER__UTILS__TRACKER_STATE_HPP_
#define AUTOWARE__TRACKING_OBJECT_MERGER__UTILS__TRACKER_STATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include "autoware_perception_msgs/msg/tracked_object_kinematics.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tracking_object_merger
{
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

// Enum classes to distinguish input sources
enum class MEASUREMENT_STATE : int {
  NONE = 0,                // 0000
  LIDAR = 1,               // 0001
  RADAR = 2,               // 0010
  CAMERA = 4,              // 0100
  LIDAR_RADAR = 3,         // 0011
  LIDAR_CAMERA = 5,        // 0101
  RADAR_CAMERA = 6,        // 0110
  LIDAR_RADAR_CAMERA = 7,  // 0111
};

// Operator overloading for MEASUREMENT_STATE
// 1. operator| for MEASUREMENT_STATE
//    e.g. MEASUREMENT_STATE::LIDAR | MEASUREMENT_STATE::RADAR == MEASUREMENT_STATE::LIDAR_RADAR
// 2. operator& for MEASUREMENT_STATE
//    e.g. MEASUREMENT_STATE::LIDAR_RADAR & MEASUREMENT_STATE::LIDAR == true
//    e.g. MEASUREMENT_STATE::LIDAR_RADAR & MEASUREMENT_STATE::CAMERA == false
inline MEASUREMENT_STATE operator|(MEASUREMENT_STATE lhs, MEASUREMENT_STATE rhs)
{
  return static_cast<MEASUREMENT_STATE>(static_cast<int>(lhs) | static_cast<int>(rhs));
}
inline bool operator&(MEASUREMENT_STATE combined, MEASUREMENT_STATE target)
{
  return (static_cast<int>(combined) & static_cast<int>(target)) != 0;
}

// Struct to handle tracker state public parameter
struct TrackerStateParameter
{
  double publish_probability_threshold = 0.5;
  double remove_probability_threshold = 0.3;
  double default_lidar_existence_probability = 0.8;
  double default_radar_existence_probability = 0.7;
  double default_camera_existence_probability = 0.6;
  double decay_rate = 0.1;
  double max_dt = 2.0;
};

// Class to handle tracker state
class TrackerState
{
private:
  /* data */
  TrackedObject tracked_object_;
  rclcpp::Time last_update_time_;
  // Eigen::MatrixXf state_matrix_;
  // Eigen::MatrixXf covariance_matrix_;

  // timer handle
  std::unordered_map<MEASUREMENT_STATE, rclcpp::Time> last_updated_time_map_;
  double max_dt_ = 2.0;

public:
  TrackerState(
    const MEASUREMENT_STATE input, const rclcpp::Time & last_update_time,
    const TrackedObject & tracked_object);
  TrackerState(
    const MEASUREMENT_STATE input_source, const rclcpp::Time & last_update_time,
    const autoware_perception_msgs::msg::TrackedObject & tracked_object,
    const unique_identifier_msgs::msg::UUID & uuid);

  ~TrackerState();

public:
  void setParameter(const TrackerStateParameter & parameter);
  bool predict(const rclcpp::Time & time);
  bool predict(const double dt, std::function<TrackedObject(const TrackedObject &, double)> func);
  MEASUREMENT_STATE getCurrentMeasurementState(const rclcpp::Time & current_time) const;
  bool updateState(
    const MEASUREMENT_STATE input, const rclcpp::Time & current_time,
    const TrackedObject & tracked_object);
  void updateWithLIDAR(const rclcpp::Time & current_time, const TrackedObject & tracked_object);
  void updateWithRADAR(const rclcpp::Time & current_time, const TrackedObject & tracked_object);
  void updateWithCAMERA(const rclcpp::Time & current_time, const TrackedObject & tracked_object);
  void updateWithoutSensor(const rclcpp::Time & current_time);
  bool update(const MEASUREMENT_STATE input, const TrackedObject & tracked_object);
  bool updateWithFunction(
    const MEASUREMENT_STATE input, const rclcpp::Time & current_time,
    const TrackedObject & tracked_object,
    std::function<void(TrackedObject &, const TrackedObject &)> update_func);
  // const functions
  bool hasUUID(const MEASUREMENT_STATE input, const unique_identifier_msgs::msg::UUID & uuid) const;
  bool isValid(const rclcpp::Time & current_time) const;
  bool canPublish() const;
  TrackedObject getObject() const;

public:
  // handle uuid
  unique_identifier_msgs::msg::UUID const_uuid_;
  // each sensor input to uuid map
  std::unordered_map<MEASUREMENT_STATE, std::optional<unique_identifier_msgs::msg::UUID>>
    input_uuid_map_;
  MEASUREMENT_STATE measurement_state_;

  // following lifecycle control parameter is overwritten by TrackerStateParameter
  std::unordered_map<MEASUREMENT_STATE, double> default_existence_probability_map_ = {
    {MEASUREMENT_STATE::LIDAR, 0.8},
    {MEASUREMENT_STATE::RADAR, 0.7},
    {MEASUREMENT_STATE::CAMERA, 0.6},
  };
  double existence_probability_ = 0.0;
  double publish_probability_threshold_ = 0.5;
  double remove_probability_threshold_ = 0.3;
  double decay_rate_ = 0.1;
};

TrackedObjects getTrackedObjectsFromTrackerStates(
  std::vector<TrackerState> & tracker_states, const rclcpp::Time & time,
  const std::string & frame_id);
}  // namespace autoware::tracking_object_merger

#endif  // AUTOWARE__TRACKING_OBJECT_MERGER__UTILS__TRACKER_STATE_HPP_
