// Copyright 2024 Tier IV, Inc.
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

#ifndef MULTI_OBJECT_TRACKER__DEBUGGER_HPP_
#define MULTI_OBJECT_TRACKER__DEBUGGER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/ros/published_time_publisher.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>

/**
 * @brief Debugger class for multi object tracker
 * @details This class is used to publish debug information of multi object tracker
 */
class TrackerDebugger
{
public:
  explicit TrackerDebugger(rclcpp::Node & node);
  void publishTentativeObjects(
    const autoware_auto_perception_msgs::msg::TrackedObjects & tentative_objects) const;
  void startMeasurementTime(
    const rclcpp::Time & now, const rclcpp::Time & measurement_header_stamp);
  void endPublishTime(const rclcpp::Time & now, const rclcpp::Time & object_time);
  void setupDiagnostics();
  void checkDelay(diagnostic_updater::DiagnosticStatusWrapper & stat);
  struct DEBUG_SETTINGS
  {
    bool publish_processing_time;
    bool publish_tentative_objects;
    double diagnostics_warn_delay;
    double diagnostics_error_delay;
  } debug_settings_;
  double pipeline_latency_ms_ = 0.0;
  diagnostic_updater::Updater diagnostic_updater_;

private:
  void loadParameters();
  rclcpp::Node & node_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    debug_tentative_objects_pub_;
  std::unique_ptr<tier4_autoware_utils::DebugPublisher> processing_time_publisher_;
  rclcpp::Time last_input_stamp_;
  rclcpp::Time stamp_process_start_;
  rclcpp::Time stamp_publish_output_;
};

#endif  // MULTI_OBJECT_TRACKER__DEBUGGER_HPP_
