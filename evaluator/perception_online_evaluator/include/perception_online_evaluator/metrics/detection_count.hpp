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

#ifndef PERCEPTION_ONLINE_EVALUATOR__METRICS__DETECTION_COUNT_HPP_
#define PERCEPTION_ONLINE_EVALUATOR__METRICS__DETECTION_COUNT_HPP_

#include "perception_online_evaluator/parameters.hpp"
#include "perception_online_evaluator/stat.hpp"
#include "tf2_ros/buffer.h"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace perception_diagnostics
{
namespace metrics
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;

struct DetectionRange
{
  double radius;
  double height;

  DetectionRange(double radius, double height) : radius(radius), height(height) {}

  std::string toString() const
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "r" << radius << "_h" << height;
    return oss.str();
  }
};
using DetectionRanges = std::vector<DetectionRange>;

/*
 * @brief Class to count objects detected within a certain range for each class
  * The overall frame is not eternal, and data older than the purge seconds is deleted to prevent
 memory bloat.
  1. Count the total number of unique objects detected across the overall frame. Data older than the
 purge seconds is deleted to prevent memory bloat.
  2. Calculate the average number of objects detected in each frame across the overall frame.
  3. Calculate the average number of objects detected in each frame within a certain time frame.
 */
class DetectionCounter
{
public:
  /*
   * @brief Constructor
   * @param parameters Parameters for the perception online evaluator
   */
  explicit DetectionCounter(const std::shared_ptr<Parameters> & parameters)
  : parameters_(parameters)
  {
  }
  /*
   * @brief Add objects to the detection counter
   * @param objects Predicted objects
   * @param tf_buffer tf2 buffer for transforming object poses
   */
  void addObjects(const PredictedObjects & objects, const tf2_ros::Buffer & tf_buffer);

  /*
   * @brief Get the average count of objects detected in the last `seconds` seconds
   * @param seconds Time window in seconds
   * @return Map of classification to range to average count
   */
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, double>> getAverageCount(
    double seconds) const;

  /*
   * @brief Get the overall average count of objects detected
   * @return Map of classification to range to average count
   */
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, double>> getOverallAverageCount()
    const;

  /*
   * @brief Get the total count of objects detected
   * @return Map of classification to range to total count
   */
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, size_t>> getTotalCount() const;

private:
  /*
   * @brief Initialize the detection map
   */
  void initializeDetectionMap();

  /*
   * @brief Update the detection map with a new detection
   * @param uuid UUID of the detected object
   * @param classification Classification of the detected object
   * @param range Range of the detected object
   * @param timestamp Timestamp of the detection
   */
  void updateDetectionMap(
    const std::string uuid, const std::uint8_t classification, const std::string & range,
    const rclcpp::Time & timestamp);

  /*
   * @brief Purge old records from the detection map
   * @param current_time Current time
   */
  void purgeOldRecords(rclcpp::Time current_time);

  /*
   * @brief Get the DetectionRanges from parameters
   * @return Detection ranges
   */
  DetectionRanges getRanges() const;

  std::shared_ptr<Parameters> parameters_;
  std::set<rclcpp::Time> unique_timestamps_;

  // Structures for storing detection counts and UUIDs for uniqueness checks
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, std::vector<rclcpp::Time>>>
    time_series_counts_;
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, std::set<std::string>>>
    seen_uuids_;
};
}  // namespace metrics
}  // namespace perception_diagnostics

#endif  // PERCEPTION_ONLINE_EVALUATOR__METRICS__DETECTION_COUNT_HPP_
