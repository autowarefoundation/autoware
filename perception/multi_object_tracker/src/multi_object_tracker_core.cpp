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

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options),
  last_published_time_(this->now()),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Create publishers and subscribers
  detected_object_sub_ = create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input", rclcpp::QoS{1},
    std::bind(&MultiObjectTracker::onMeasurement, this, std::placeholders::_1));
  tracked_objects_pub_ =
    create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>("output", rclcpp::QoS{1});

  // Parameters
  double publish_rate = declare_parameter<double>("publish_rate");  // [hz]
  world_frame_id_ = declare_parameter<std::string>("world_frame_id");
  bool enable_delay_compensation{declare_parameter<bool>("enable_delay_compensation")};

  // Debug publishers
  debugger_ = std::make_unique<TrackerDebugger>(*this);

  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);

  // Create ROS time based timer.
  // If the delay compensation is enabled, the timer is used to publish the output at the correct
  // time.
  if (enable_delay_compensation) {
    publisher_period_ = 1.0 / publish_rate;    // [s]
    constexpr double timer_multiplier = 20.0;  // 20 times frequent for publish timing check
    const auto timer_period = rclcpp::Rate(publish_rate * timer_multiplier).period();
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), timer_period, std::bind(&MultiObjectTracker::onTimer, this));
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
  published_time_publisher_ = std::make_unique<tier4_autoware_utils::PublishedTimePublisher>(this);
}

void MultiObjectTracker::onMeasurement(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objects_msg)
{
  // Get the time of the measurement
  const rclcpp::Time measurement_time =
    rclcpp::Time(input_objects_msg->header.stamp, this->now().get_clock_type());

  /* keep the latest input stamp and check transform*/
  debugger_->startMeasurementTime(this->now(), measurement_time);
  const auto self_transform =
    getTransformAnonymous(tf_buffer_, "base_link", world_frame_id_, measurement_time);
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
  checkTrackerLifeCycle(list_tracker_, measurement_time);
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

  // debugger time
  debugger_->endMeasurementTime(this->now());

  // Publish objects if the timer is not enabled
  if (publish_timer_ == nullptr) {
    // Publish if the delay compensation is disabled
    publish(measurement_time);
  } else {
    // Publish if the next publish time is close
    const double minimum_publish_interval = publisher_period_ * 0.70;  // 70% of the period
    if ((this->now() - last_published_time_).seconds() > minimum_publish_interval) {
      checkAndPublish(this->now());
    }
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
  const rclcpp::Time current_time = this->now();
  // check the publish period
  const auto elapsed_time = (current_time - last_published_time_).seconds();
  // if the elapsed time is over the period, publish objects with prediction
  constexpr double maximum_latency_ratio = 1.11;  // 11% margin
  const double maximum_publish_latency = publisher_period_ * maximum_latency_ratio;
  if (elapsed_time > maximum_publish_latency) {
    checkAndPublish(current_time);
  }
}

void MultiObjectTracker::checkAndPublish(const rclcpp::Time & time)
{
  /* life cycle check */
  checkTrackerLifeCycle(list_tracker_, time);
  /* sanitize trackers */
  sanitizeTracker(list_tracker_, time);

  // Publish
  publish(time);

  // Update last published time
  last_published_time_ = this->now();
}

void MultiObjectTracker::checkTrackerLifeCycle(
  std::list<std::shared_ptr<Tracker>> & list_tracker, const rclcpp::Time & time)
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
    if (!(*itr1)->getTrackedObject(time, object1)) continue;
    for (auto itr2 = std::next(itr1); itr2 != list_tracker.end(); ++itr2) {
      autoware_auto_perception_msgs::msg::TrackedObject object2;
      if (!(*itr2)->getTrackedObject(time, object2)) continue;
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

void MultiObjectTracker::publish(const rclcpp::Time & time) const
{
  debugger_->startPublishTime(this->now());
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
      if (!(*itr)->getTrackedObject(time, object)) continue;
      tentative_objects_msg.objects.push_back(object);
      continue;
    }
    autoware_auto_perception_msgs::msg::TrackedObject object;
    if (!(*itr)->getTrackedObject(time, object)) continue;
    output_msg.objects.push_back(object);
  }

  // Publish
  tracked_objects_pub_->publish(output_msg);
  published_time_publisher_->publish_if_subscribed(tracked_objects_pub_, output_msg.header.stamp);

  // Debugger Publish if enabled
  debugger_->endPublishTime(this->now(), time);
  debugger_->publishTentativeObjects(tentative_objects_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(MultiObjectTracker)
