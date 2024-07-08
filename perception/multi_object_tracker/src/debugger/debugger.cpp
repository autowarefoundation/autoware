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

#include "debugger.hpp"

#include <memory>

namespace autoware::multi_object_tracker
{
TrackerDebugger::TrackerDebugger(rclcpp::Node & node, const std::string & frame_id)
: node_(node), diagnostic_updater_(&node), object_debugger_(frame_id)
{
  // declare debug parameters to decide whether to publish debug topics
  loadParameters();
  // initialize debug publishers
  if (debug_settings_.publish_processing_time) {
    processing_time_publisher_ =
      std::make_unique<autoware::universe_utils::DebugPublisher>(&node_, "multi_object_tracker");
  }

  if (debug_settings_.publish_tentative_objects) {
    debug_tentative_objects_pub_ =
      node_.create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
        "~/debug/tentative_objects", rclcpp::QoS{1});
  }

  if (debug_settings_.publish_debug_markers) {
    debug_objects_markers_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/objects_markers", rclcpp::QoS{1});
  }

  // initialize timestamps
  const rclcpp::Time now = node_.now();
  last_input_stamp_ = now;
  stamp_process_start_ = now;
  stamp_process_end_ = now;
  stamp_publish_start_ = now;
  stamp_publish_output_ = now;

  // setup diagnostics
  setupDiagnostics();
}

void TrackerDebugger::loadParameters()
{
  try {
    debug_settings_.publish_processing_time =
      node_.declare_parameter<bool>("publish_processing_time");
    debug_settings_.publish_tentative_objects =
      node_.declare_parameter<bool>("publish_tentative_objects");
    debug_settings_.publish_debug_markers = node_.declare_parameter<bool>("publish_debug_markers");
    debug_settings_.diagnostics_warn_delay =
      node_.declare_parameter<double>("diagnostics_warn_delay");
    debug_settings_.diagnostics_error_delay =
      node_.declare_parameter<double>("diagnostics_error_delay");
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_.get_logger(), "Failed to declare parameter: %s", e.what());
    debug_settings_.publish_processing_time = false;
    debug_settings_.publish_tentative_objects = false;
    debug_settings_.publish_debug_markers = false;
    debug_settings_.diagnostics_warn_delay = 0.5;
    debug_settings_.diagnostics_error_delay = 1.0;
  }
}

void TrackerDebugger::setupDiagnostics()
{
  diagnostic_updater_.setHardwareID(node_.get_name());
  diagnostic_updater_.add(
    "Perception delay check from original header stamp", this, &TrackerDebugger::checkDelay);
  diagnostic_updater_.setPeriod(0.1);
}

// Object publishing functions

void TrackerDebugger::publishTentativeObjects(
  const autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const
{
  if (debug_settings_.publish_tentative_objects) {
    debug_tentative_objects_pub_->publish(tentative_objects);
  }
}

// Time measurement functions

void TrackerDebugger::checkDelay(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!is_initialized_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Measurement time is not set.");
    return;
  }
  const double & delay = pipeline_latency_ms_ / 1e3;  // [s]

  if (delay == 0.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Detection delay is not calculated.");
  } else if (delay < debug_settings_.diagnostics_warn_delay) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Detection delay is acceptable");
  } else if (delay < debug_settings_.diagnostics_error_delay) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Detection delay is over warn threshold.");
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Detection delay is over error threshold.");
  }

  stat.add("Detection delay", delay);
}

void TrackerDebugger::startMeasurementTime(
  const rclcpp::Time & now, const rclcpp::Time & measurement_header_stamp)
{
  last_input_stamp_ = measurement_header_stamp;
  stamp_process_start_ = now;
  if (debug_settings_.publish_processing_time) {
    double input_latency_ms = (now - last_input_stamp_).seconds() * 1e3;
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/input_latency_ms", input_latency_ms);
  }
  // initialize debug time stamps
  if (!is_initialized_) {
    stamp_publish_output_ = now;
    is_initialized_ = true;
  }
}

void TrackerDebugger::endMeasurementTime(const rclcpp::Time & now)
{
  stamp_process_end_ = now;
}

void TrackerDebugger::startPublishTime(const rclcpp::Time & now)
{
  stamp_publish_start_ = now;
}

void TrackerDebugger::endPublishTime(const rclcpp::Time & now, const rclcpp::Time & object_time)
{
  // if the measurement time is not set, do not publish debug information
  if (!is_initialized_) return;

  // pipeline latency: time from sensor measurement to publish, used for 'checkDelay'
  pipeline_latency_ms_ = (now - last_input_stamp_).seconds() * 1e3;

  if (debug_settings_.publish_processing_time) {
    // processing latency: time between input and publish
    double processing_latency_ms = ((now - stamp_process_start_).seconds()) * 1e3;
    // processing time: only the time spent in the tracking processes
    double processing_time_ms = ((now - stamp_publish_start_).seconds() +
                                 (stamp_process_end_ - stamp_process_start_).seconds()) *
                                1e3;
    // cycle time: time between two consecutive publish
    double cyclic_time_ms = (now - stamp_publish_output_).seconds() * 1e3;
    // measurement to tracked-object time: time from the sensor measurement to the publishing object
    // time If there is not latency compensation, this value is zero
    double measurement_to_object_ms = (object_time - last_input_stamp_).seconds() * 1e3;

    // starting from the measurement time
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms_);
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_latency_ms", processing_latency_ms);
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/meas_to_tracked_object_ms", measurement_to_object_ms);
  }
  stamp_publish_output_ = now;
}

void TrackerDebugger::collectObjectInfo(
  const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
  const uint & channel_index,
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
  const std::unordered_map<int, int> & direct_assignment,
  const std::unordered_map<int, int> & reverse_assignment)
{
  if (!debug_settings_.publish_debug_markers) return;
  object_debugger_.collect(
    message_time, list_tracker, channel_index, detected_objects, direct_assignment,
    reverse_assignment);
}

// ObjectDebugger
void TrackerDebugger::publishObjectsMarkers()
{
  if (!debug_settings_.publish_debug_markers) return;

  visualization_msgs::msg::MarkerArray marker_message;
  // process data
  object_debugger_.process();

  // publish markers
  object_debugger_.getMessage(marker_message);
  debug_objects_markers_pub_->publish(marker_message);

  // reset object data
  object_debugger_.reset();
}

}  // namespace autoware::multi_object_tracker
