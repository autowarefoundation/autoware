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

#include <boost/optional.hpp>

#include <glog/logging.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "multi_object_tracker/multi_object_tracker_core.hpp"
#include "multi_object_tracker/utils/utils.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp_components/register_node_macro.hpp>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

namespace
{
boost::optional<geometry_msgs::msg::Transform> getTransformAnonymous(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    // check if the frames are ready
    std::string errstr;  // This argument prevents error msg from being displayed in the terminal.
    if (!tf_buffer.canTransform(
          target_frame_id, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return boost::none;
    }

    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return boost::none;
  }
}

}  // namespace

TrackerDebugger::TrackerDebugger(rclcpp::Node & node)
: diagnostic_updater_(&node), node_(node), last_input_stamp_(node.now())
{
  // declare debug parameters to decide whether to publish debug topics
  loadParameters();
  // initialize debug publishers
  stop_watch_ptr_ = std::make_unique<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>>();
  if (debug_settings_.publish_processing_time) {
    processing_time_publisher_ =
      std::make_unique<tier4_autoware_utils::DebugPublisher>(&node_, "multi_object_tracker");
  }

  if (debug_settings_.publish_tentative_objects) {
    debug_tentative_objects_pub_ =
      node_.create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>(
        "debug/tentative_objects", rclcpp::QoS{1});
  }

  // initialize stop watch and diagnostics
  startStopWatch();
  setupDiagnostics();
}

void TrackerDebugger::setupDiagnostics()
{
  diagnostic_updater_.setHardwareID(node_.get_name());
  diagnostic_updater_.add(
    "Perception delay check from original header stamp", this, &TrackerDebugger::checkDelay);
  diagnostic_updater_.setPeriod(0.1);
}

void TrackerDebugger::loadParameters()
{
  try {
    debug_settings_.publish_processing_time =
      node_.declare_parameter<bool>("publish_processing_time");
    debug_settings_.publish_tentative_objects =
      node_.declare_parameter<bool>("publish_tentative_objects");
    debug_settings_.diagnostics_warn_delay =
      node_.declare_parameter<double>("diagnostics_warn_delay");
    debug_settings_.diagnostics_error_delay =
      node_.declare_parameter<double>("diagnostics_error_delay");
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_.get_logger(), "Failed to declare parameter: %s", e.what());
    debug_settings_.publish_processing_time = false;
    debug_settings_.publish_tentative_objects = false;
    debug_settings_.diagnostics_warn_delay = 0.5;
    debug_settings_.diagnostics_error_delay = 1.0;
  }
}

void TrackerDebugger::checkDelay(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const double delay = elapsed_time_from_sensor_input_;  // [s]

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

void TrackerDebugger::publishProcessingTime()
{
  const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
  const auto current_time = node_.now();
  elapsed_time_from_sensor_input_ = (current_time - last_input_stamp_).seconds();
  if (debug_settings_.publish_processing_time) {
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/elapsed_time_from_sensor_input_ms", elapsed_time_from_sensor_input_ * 1e3);
  }
}

void TrackerDebugger::publishTentativeObjects(
  const autoware_auto_perception_msgs::msg::TrackedObjects & tentative_objects) const
{
  if (debug_settings_.publish_tentative_objects) {
    debug_tentative_objects_pub_->publish(tentative_objects);
  }
}

void TrackerDebugger::startStopWatch()
{
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void TrackerDebugger::startMeasurementTime(const rclcpp::Time & measurement_header_stamp)
{
  last_input_stamp_ = measurement_header_stamp;
  // start measuring processing time
  stop_watch_ptr_->toc("processing_time", true);
}

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // glog for debug
  google::InitGoogleLogging("multi_object_tracker");
  google::InstallFailureSignalHandler();

  // Create publishers and subscribers
  detected_object_sub_ = create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input", rclcpp::QoS{1},
    std::bind(&MultiObjectTracker::onMeasurement, this, std::placeholders::_1));
  tracked_objects_pub_ =
    create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>("output", rclcpp::QoS{1});

  // Parameters
  double publish_rate = declare_parameter<double>("publish_rate");
  world_frame_id_ = declare_parameter<std::string>("world_frame_id");
  bool enable_delay_compensation{declare_parameter<bool>("enable_delay_compensation")};

  // Debug publishers
  debugger_ = std::make_unique<TrackerDebugger>(*this);

  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);

  // Create ROS time based timer
  if (enable_delay_compensation) {
    const auto period_ns = rclcpp::Rate(publish_rate).period();
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&MultiObjectTracker::onTimer, this));
  }

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());

  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");

  // tracker map
  tracker_map_.insert(
    std::make_pair(Label::CAR, this->declare_parameter<std::string>("car_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::TRUCK, this->declare_parameter<std::string>("truck_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::BUS, this->declare_parameter<std::string>("bus_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::TRAILER, this->declare_parameter<std::string>("trailer_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::PEDESTRIAN, this->declare_parameter<std::string>("pedestrian_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::BICYCLE, this->declare_parameter<std::string>("bicycle_tracker")));
  tracker_map_.insert(
    std::make_pair(Label::MOTORCYCLE, this->declare_parameter<std::string>("motorcycle_tracker")));

  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_area_matrix, min_area_matrix, max_rad_matrix,
    min_iou_matrix);
}

void MultiObjectTracker::onMeasurement(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objects_msg)
{
  /* keep the latest input stamp and check transform*/
  debugger_->startMeasurementTime(rclcpp::Time(input_objects_msg->header.stamp));
  const auto self_transform = getTransformAnonymous(
    tf_buffer_, "base_link", world_frame_id_, input_objects_msg->header.stamp);
  if (!self_transform) {
    return;
  }

  /* transform to world coordinate */
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *input_objects_msg, world_frame_id_, tf_buffer_, transformed_objects)) {
    return;
  }
  /* tracker prediction */
  rclcpp::Time measurement_time = input_objects_msg->header.stamp;
  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    (*itr)->predict(measurement_time);
  }

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  Eigen::MatrixXd score_matrix = data_association_->calcScoreMatrix(
    transformed_objects, list_tracker_);  // row : tracker, col : measurement
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  /* tracker measurement update */
  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker_.begin(); tracker_itr != list_tracker_.end();
       ++tracker_itr, ++tracker_idx) {
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {  // found
      (*(tracker_itr))
        ->updateWithMeasurement(
          transformed_objects.objects.at(direct_assignment.find(tracker_idx)->second),
          measurement_time, *self_transform);
    } else {  // not found
      (*(tracker_itr))->updateWithoutMeasurement();
    }
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, measurement_time, *self_transform);
  /* sanitize trackers */
  sanitizeTracker(list_tracker_, measurement_time);

  /* new tracker */
  for (size_t i = 0; i < transformed_objects.objects.size(); ++i) {
    if (reverse_assignment.find(i) != reverse_assignment.end()) {  // found
      continue;
    }
    std::shared_ptr<Tracker> tracker =
      createNewTracker(transformed_objects.objects.at(i), measurement_time, *self_transform);
    if (tracker) list_tracker_.push_back(tracker);
  }

  if (publish_timer_ == nullptr) {
    publish(measurement_time);
  }
}

std::shared_ptr<Tracker> MultiObjectTracker::createNewTracker(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform) const
{
  const std::uint8_t label = object_recognition_utils::getHighestProbLabel(object.classification);
  if (tracker_map_.count(label) != 0) {
    const auto tracker = tracker_map_.at(label);

    if (tracker == "bicycle_tracker") {
      return std::make_shared<BicycleTracker>(time, object, self_transform);
    } else if (tracker == "big_vehicle_tracker") {
      return std::make_shared<BigVehicleTracker>(time, object, self_transform);
    } else if (tracker == "multi_vehicle_tracker") {
      return std::make_shared<MultipleVehicleTracker>(time, object, self_transform);
    } else if (tracker == "normal_vehicle_tracker") {
      return std::make_shared<NormalVehicleTracker>(time, object, self_transform);
    } else if (tracker == "pass_through_tracker") {
      return std::make_shared<PassThroughTracker>(time, object, self_transform);
    } else if (tracker == "pedestrian_and_bicycle_tracker") {
      return std::make_shared<PedestrianAndBicycleTracker>(time, object, self_transform);
    } else if (tracker == "pedestrian_tracker") {
      return std::make_shared<PedestrianTracker>(time, object, self_transform);
    }
  }
  return std::make_shared<UnknownTracker>(time, object, self_transform);
}

void MultiObjectTracker::onTimer()
{
  rclcpp::Time current_time = this->now();
  const auto self_transform =
    getTransformAnonymous(tf_buffer_, world_frame_id_, "base_link", current_time);
  if (!self_transform) {
    return;
  }

  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, current_time, *self_transform);
  /* sanitize trackers */
  sanitizeTracker(list_tracker_, current_time);

  // Publish
  publish(current_time);
}

void MultiObjectTracker::checkTrackerLifeCycle(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time,
  [[maybe_unused]] const geometry_msgs::msg::Transform & self_transform)
{
  /* params */
  constexpr float max_elapsed_time = 1.0;

  /* delete tracker */
  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    const bool is_old = max_elapsed_time < (*itr)->getElapsedTimeFromLastUpdate(time);
    if (is_old) {
      auto erase_itr = itr;
      --itr;
      list_tracker.erase(erase_itr);
    }
  }
}

void MultiObjectTracker::sanitizeTracker(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time)
{
  constexpr float min_iou = 0.1;
  constexpr float min_iou_for_unknown_object = 0.001;
  constexpr double distance_threshold = 5.0;
  /* delete collision tracker */
  for (auto itr1 = list_tracker.begin(); itr1 != list_tracker.end(); ++itr1) {
    autoware_auto_perception_msgs::msg::TrackedObject object1;
    (*itr1)->getTrackedObject(time, object1);
    for (auto itr2 = std::next(itr1); itr2 != list_tracker.end(); ++itr2) {
      autoware_auto_perception_msgs::msg::TrackedObject object2;
      (*itr2)->getTrackedObject(time, object2);
      const double distance = std::hypot(
        object1.kinematics.pose_with_covariance.pose.position.x -
          object2.kinematics.pose_with_covariance.pose.position.x,
        object1.kinematics.pose_with_covariance.pose.position.y -
          object2.kinematics.pose_with_covariance.pose.position.y);
      if (distance_threshold < distance) {
        continue;
      }

      const double min_union_iou_area = 1e-2;
      const auto iou = object_recognition_utils::get2dIoU(object1, object2, min_union_iou_area);
      const auto & label1 = (*itr1)->getHighestProbLabel();
      const auto & label2 = (*itr2)->getHighestProbLabel();
      bool should_delete_tracker1 = false;
      bool should_delete_tracker2 = false;

      // If at least one of them is UNKNOWN, delete the one with lower IOU. Because the UNKNOWN
      // objects are not reliable.
      if (label1 == Label::UNKNOWN || label2 == Label::UNKNOWN) {
        if (min_iou_for_unknown_object < iou) {
          if (label1 == Label::UNKNOWN && label2 == Label::UNKNOWN) {
            if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
              should_delete_tracker1 = true;
            } else {
              should_delete_tracker2 = true;
            }
          } else if (label1 == Label::UNKNOWN) {
            should_delete_tracker1 = true;
          } else if (label2 == Label::UNKNOWN) {
            should_delete_tracker2 = true;
          }
        }
      } else {  // If neither is UNKNOWN, delete the one with lower IOU.
        if (min_iou < iou) {
          if ((*itr1)->getTotalMeasurementCount() < (*itr2)->getTotalMeasurementCount()) {
            should_delete_tracker1 = true;
          } else {
            should_delete_tracker2 = true;
          }
        }
      }

      if (should_delete_tracker1) {
        itr1 = list_tracker.erase(itr1);
        --itr1;
        break;
      } else if (should_delete_tracker2) {
        itr2 = list_tracker.erase(itr2);
        --itr2;
      }
    }
  }
}

inline bool MultiObjectTracker::shouldTrackerPublish(
  const std::shared_ptr<const Tracker> tracker) const
{
  constexpr int measurement_count_threshold = 3;
  if (tracker->getTotalMeasurementCount() < measurement_count_threshold) {
    return false;
  }
  return true;
}

void MultiObjectTracker::publish(const rclcpp::Time & time)
{
  const auto subscriber_count = tracked_objects_pub_->get_subscription_count() +
                                tracked_objects_pub_->get_intra_process_subscription_count();
  if (subscriber_count < 1) {
    return;
  }
  // Create output msg
  autoware_auto_perception_msgs::msg::TrackedObjects output_msg, tentative_objects_msg;
  output_msg.header.frame_id = world_frame_id_;
  output_msg.header.stamp = time;
  tentative_objects_msg.header = output_msg.header;

  for (auto itr = list_tracker_.begin(); itr != list_tracker_.end(); ++itr) {
    if (!shouldTrackerPublish(*itr)) {  // for debug purpose
      autoware_auto_perception_msgs::msg::TrackedObject object;
      (*itr)->getTrackedObject(time, object);
      tentative_objects_msg.objects.push_back(object);
      continue;
    }
    autoware_auto_perception_msgs::msg::TrackedObject object;
    (*itr)->getTrackedObject(time, object);
    output_msg.objects.push_back(object);
  }

  // Publish
  tracked_objects_pub_->publish(output_msg);

  // Debugger Publish if enabled
  debugger_->publishProcessingTime();
  debugger_->publishTentativeObjects(tentative_objects_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(MultiObjectTracker)
