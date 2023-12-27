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

#include "tracking_object_merger/decorative_tracker_merger.hpp"

#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tracking_object_merger/data_association/solver/successive_shortest_path.hpp"
#include "tracking_object_merger/utils/utils.hpp"

#include <boost/optional.hpp>

#include <glog/logging.h>

#include <chrono>
#include <unordered_map>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

namespace tracking_object_merger
{

using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

// get unix time from header
double getUnixTime(const std_msgs::msg::Header & header)
{
  return header.stamp.sec + header.stamp.nanosec * 1e-9;
}

// calc association score matrix
Eigen::MatrixXd calcScoreMatrixForAssociation(
  const MEASUREMENT_STATE measurement_state,
  const autoware_auto_perception_msgs::msg::TrackedObjects & objects0,
  const std::vector<TrackerState> & trackers,
  const std::unordered_map<std::string, std::unique_ptr<DataAssociation>> & data_association_map
  // const bool debug_log, const std::string & file_name // do not logging for now
)
{
  // get current time
  const rclcpp::Time current_time = rclcpp::Time(objects0.header.stamp);

  // calc score matrix
  Eigen::MatrixXd score_matrix = Eigen::MatrixXd::Zero(trackers.size(), objects0.objects.size());
  for (size_t trackers_idx = 0; trackers_idx < trackers.size(); ++trackers_idx) {
    const auto & tracker_obj = trackers.at(trackers_idx);
    const auto & object1 = tracker_obj.getObject();
    const auto & tracker_state = tracker_obj.getCurrentMeasurementState(current_time);

    for (size_t objects0_idx = 0; objects0_idx < objects0.objects.size(); ++objects0_idx) {
      const auto & object0 = objects0.objects.at(objects0_idx);
      // switch calc score function by input and trackers measurement state
      // we assume that lidar and radar are exclusive
      double score;
      const auto input_has_lidar = measurement_state & MEASUREMENT_STATE::LIDAR;
      const auto tracker_has_lidar = tracker_state & MEASUREMENT_STATE::LIDAR;
      if (input_has_lidar && tracker_has_lidar) {
        score = data_association_map.at("lidar-lidar")->calcScoreBetweenObjects(object0, object1);
      } else if (!input_has_lidar && !tracker_has_lidar) {
        score = data_association_map.at("radar-radar")->calcScoreBetweenObjects(object0, object1);
      } else {
        score = data_association_map.at("lidar-radar")->calcScoreBetweenObjects(object0, object1);
      }
      score_matrix(trackers_idx, objects0_idx) = score;
    }
  }
  return score_matrix;
}

DecorativeTrackerMergerNode::DecorativeTrackerMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("decorative_object_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  // glog for debug
  google::InitGoogleLogging("decorative_object_merger_node");
  google::InstallFailureSignalHandler();

  // Subscriber
  sub_main_objects_ = create_subscription<TrackedObjects>(
    "input/main_object", rclcpp::QoS{1},
    std::bind(&DecorativeTrackerMergerNode::mainObjectsCallback, this, std::placeholders::_1));
  sub_sub_objects_ = create_subscription<TrackedObjects>(
    "input/sub_object", rclcpp::QoS{1},
    std::bind(&DecorativeTrackerMergerNode::subObjectsCallback, this, std::placeholders::_1));

  // merged object publisher
  merged_object_pub_ = create_publisher<TrackedObjects>("output/object", rclcpp::QoS{1});
  // debug object publisher
  debug_object_pub_ =
    create_publisher<TrackedObjects>("debug/interpolated_sub_object", rclcpp::QoS{1});

  // logging
  logging_.enable = declare_parameter<bool>("enable_logging");
  logging_.path = declare_parameter<std::string>("logging_file_path");

  // Parameters
  publish_interpolated_sub_objects_ = declare_parameter<bool>("publish_interpolated_sub_objects");
  base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id");
  time_sync_threshold_ = declare_parameter<double>("time_sync_threshold");
  sub_object_timeout_sec_ = declare_parameter<double>("sub_object_timeout_sec");
  // default setting parameter for tracker
  tracker_state_parameter_.remove_probability_threshold =
    declare_parameter<double>("tracker_state_parameter.remove_probability_threshold");
  tracker_state_parameter_.publish_probability_threshold =
    declare_parameter<double>("tracker_state_parameter.publish_probability_threshold");
  tracker_state_parameter_.default_lidar_existence_probability =
    declare_parameter<double>("tracker_state_parameter.default_lidar_existence_probability");
  tracker_state_parameter_.default_radar_existence_probability =
    declare_parameter<double>("tracker_state_parameter.default_radar_existence_probability");
  tracker_state_parameter_.default_camera_existence_probability =
    declare_parameter<double>("tracker_state_parameter.default_camera_existence_probability");
  tracker_state_parameter_.decay_rate =
    declare_parameter<double>("tracker_state_parameter.decay_rate");
  tracker_state_parameter_.max_dt = declare_parameter<double>("tracker_state_parameter.max_dt");

  const std::string main_sensor_type = declare_parameter<std::string>("main_sensor_type");
  const std::string sub_sensor_type = declare_parameter<std::string>("sub_sensor_type");
  // str to MEASUREMENT_STATE
  auto str2measurement_state = [](const std::string & str) {
    if (str == "lidar") {
      return MEASUREMENT_STATE::LIDAR;
    } else if (str == "radar") {
      return MEASUREMENT_STATE::RADAR;
    } else {
      throw std::runtime_error("invalid sensor type");
    }
  };
  main_sensor_type_ = str2measurement_state(main_sensor_type);
  sub_sensor_type_ = str2measurement_state(sub_sensor_type);

  /* init association **/
  // lidar-lidar association matrix
  set3dDataAssociation("lidar-lidar", data_association_map_);
  // lidar-radar association matrix
  set3dDataAssociation("lidar-radar", data_association_map_);
  // radar-radar association matrix
  set3dDataAssociation("radar-radar", data_association_map_);
}

void DecorativeTrackerMergerNode::set3dDataAssociation(
  const std::string & prefix,
  std::unordered_map<std::string, std::unique_ptr<DataAssociation>> & data_association_map)
{
  const auto tmp = this->declare_parameter<std::vector<int64_t>>(prefix + ".can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
  const auto max_dist_matrix =
    this->declare_parameter<std::vector<double>>(prefix + ".max_dist_matrix");
  const auto max_rad_matrix =
    this->declare_parameter<std::vector<double>>(prefix + ".max_rad_matrix");
  const auto min_iou_matrix =
    this->declare_parameter<std::vector<double>>(prefix + ".min_iou_matrix");
  const auto max_velocity_diff_matrix =
    this->declare_parameter<std::vector<double>>(prefix + ".max_velocity_diff_matrix");

  data_association_map[prefix] = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_rad_matrix, min_iou_matrix, max_velocity_diff_matrix);
}

/**
 * @brief callback function for main objects
 *
 * @param main_objects
 * @note if there are no sub objects, publish main objects as it is
 *       else, merge main objects and sub objects
 */
void DecorativeTrackerMergerNode::mainObjectsCallback(
  const TrackedObjects::ConstSharedPtr & main_objects)
{
  // try to merge sub object
  if (!sub_objects_buffer_.empty()) {
    // get interpolated sub objects
    // get newest sub objects which timestamp is earlier to main objects
    TrackedObjects::ConstSharedPtr closest_time_sub_objects;
    TrackedObjects::ConstSharedPtr closest_time_sub_objects_later;
    for (const auto & sub_object : sub_objects_buffer_) {
      if (getUnixTime(sub_object->header) < getUnixTime(main_objects->header)) {
        closest_time_sub_objects = sub_object;
      } else {
        closest_time_sub_objects_later = sub_object;
        break;
      }
    }
    // get delay compensated sub objects
    const auto interpolated_sub_objects = interpolateObjectState(
      closest_time_sub_objects, closest_time_sub_objects_later, main_objects->header);
    if (interpolated_sub_objects.has_value()) {
      // Merge sub objects
      const auto interp_sub_objs = interpolated_sub_objects.value();
      debug_object_pub_->publish(interp_sub_objs);
      this->decorativeMerger(
        sub_sensor_type_, std::make_shared<TrackedObjects>(interpolated_sub_objects.value()));
    } else {
      RCLCPP_DEBUG(this->get_logger(), "interpolated_sub_objects is null");
    }
  }

  // try to merge main object
  this->decorativeMerger(main_sensor_type_, main_objects);

  merged_object_pub_->publish(getTrackedObjects(main_objects->header));
}

/**
 * @brief callback function for sub objects
 *
 * @param msg
 * @note push back sub objects to buffer and remove old sub objects
 */
void DecorativeTrackerMergerNode::subObjectsCallback(const TrackedObjects::ConstSharedPtr & msg)
{
  sub_objects_buffer_.push_back(msg);
  // remove old sub objects
  // const auto now = get_clock()->now();
  const auto now = rclcpp::Time(msg->header.stamp);
  const auto remove_itr = std::remove_if(
    sub_objects_buffer_.begin(), sub_objects_buffer_.end(), [now, this](const auto & sub_object) {
      return (now - rclcpp::Time(sub_object->header.stamp)).seconds() > sub_object_timeout_sec_;
    });
  sub_objects_buffer_.erase(remove_itr, sub_objects_buffer_.end());
}

/**
 * @brief merge objects into inner_tracker_objects_
 *
 * @param main_objects
 * @return TrackedObjects
 *
 * @note 1. interpolate sub objects to sync main objects
 *       2. do association
 *       3. merge objects
 */
bool DecorativeTrackerMergerNode::decorativeMerger(
  const MEASUREMENT_STATE input_sensor, const TrackedObjects::ConstSharedPtr & input_objects_msg)
{
  // get current time
  const auto current_time = rclcpp::Time(input_objects_msg->header.stamp);
  if (input_objects_msg->objects.empty()) {
    return false;
  }
  if (inner_tracker_objects_.empty()) {
    for (const auto & object : input_objects_msg->objects) {
      inner_tracker_objects_.push_back(createNewTracker(input_sensor, current_time, object));
    }
  }

  // do prediction for inner objects
  for (auto & object : inner_tracker_objects_) {
    object.predict(current_time);
  }

  // TODO(yoshiri): pre-association

  // associate inner objects and input objects
  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  const auto & objects1 = input_objects_msg->objects;
  Eigen::MatrixXd score_matrix = calcScoreMatrixForAssociation(
    input_sensor, *input_objects_msg, inner_tracker_objects_, data_association_map_);
  data_association_map_.at("lidar-lidar")
    ->assign(score_matrix, direct_assignment, reverse_assignment);

  // look for tracker
  for (int tracker_idx = 0; tracker_idx < static_cast<int>(inner_tracker_objects_.size());
       ++tracker_idx) {
    auto & object0_state = inner_tracker_objects_.at(tracker_idx);
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {  // found and merge
      const auto & object1 = objects1.at(direct_assignment.at(tracker_idx));
      // merge object1 into object0_state
      object0_state.updateState(input_sensor, current_time, object1);
    } else {  // not found
      // decrease existence probability
      object0_state.updateWithoutSensor(current_time);
    }
  }
  // look for new object
  for (int object1_idx = 0; object1_idx < static_cast<int>(objects1.size()); ++object1_idx) {
    const auto & object1 = objects1.at(object1_idx);
    if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
    } else {                                                                 // not found
      inner_tracker_objects_.push_back(createNewTracker(input_sensor, current_time, object1));
    }
  }
  return true;
}

/**
 * @brief interpolate sub objects to sync main objects
 *
 * @param former_msg
 * @param latter_msg
 * @param output_header
 * @return std::optional<TrackedObjects>
 *
 * @note 1. if both msg is nullptr, return null optional
 *       2. if former_msg is nullptr, return latter_msg
 *       3. if latter_msg is nullptr, return former_msg
 *       4. if both msg is not nullptr, do the interpolation
 */
std::optional<TrackedObjects> DecorativeTrackerMergerNode::interpolateObjectState(
  const TrackedObjects::ConstSharedPtr & former_msg,
  const TrackedObjects::ConstSharedPtr & latter_msg, const std_msgs::msg::Header & output_header)
{
  // Assumption: output_header must be newer than former_msg and older than latter_msg
  // There three possible patterns
  // 1. both msg is nullptr
  // 2. former_msg is nullptr
  // 3. latter_msg is nullptr
  // 4. both msg is not nullptr

  // 1. both msg is nullptr
  if (former_msg == nullptr && latter_msg == nullptr) {
    // return null optional
    RCLCPP_DEBUG(this->get_logger(), "interpolation err: both msg is nullptr");
    return std::nullopt;
  }

  // 2. former_msg is nullptr
  if (former_msg == nullptr) {
    // std::cout << "interpolation: 2. former_msg is nullptr" << std::endl;
    // depends on header stamp difference
    if (
      (rclcpp::Time(latter_msg->header.stamp) - rclcpp::Time(output_header.stamp)).seconds() >
      time_sync_threshold_) {
      // do nothing
      RCLCPP_DEBUG(
        this->get_logger(), "interpolation err: latter msg is too different from output msg");
      return std::nullopt;
    } else {  // else, return latter_msg
      return *latter_msg;
    }

    // 3. latter_msg is nullptr
  } else if (latter_msg == nullptr) {
    // std::cout << "interpolation: 3. latter_msg is nullptr" << std::endl;
    // depends on header stamp difference
    const auto dt =
      (rclcpp::Time(output_header.stamp) - rclcpp::Time(former_msg->header.stamp)).seconds();
    if (dt > time_sync_threshold_) {
      // do nothing
      RCLCPP_DEBUG(this->get_logger(), "interpolation err: former msg is too old");
      return std::nullopt;
    } else {
      // else, return former_msg
      return utils::predictPastOrFutureTrackedObjects(*former_msg, output_header);
    }

    // 4. both msg is not nullptr
  } else {
    // do the interpolation
    // std::cout << "interpolation: 4. both msg is not nullptr" << std::endl;
    TrackedObjects interpolated_msg =
      utils::interpolateTrackedObjects(*former_msg, *latter_msg, output_header);
    return interpolated_msg;
  }
}

// get merged objects
TrackedObjects DecorativeTrackerMergerNode::getTrackedObjects(const std_msgs::msg::Header & header)
{
  // get main objects
  rclcpp::Time current_time = rclcpp::Time(header.stamp);
  return getTrackedObjectsFromTrackerStates(inner_tracker_objects_, current_time);
}

// create new tracker
TrackerState DecorativeTrackerMergerNode::createNewTracker(
  const MEASUREMENT_STATE input_index, rclcpp::Time current_time,
  const autoware_auto_perception_msgs::msg::TrackedObject & input_object)
{
  // check if object id is not included in innner_tracker_objects_
  for (const auto & object : inner_tracker_objects_) {
    if (object.const_uuid_ == input_object.object_id) {
      // create new uuid
      unique_identifier_msgs::msg::UUID uuid;
      // Generate random number
      std::mt19937 gen(std::random_device{}());
      std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
      std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
      auto new_tracker = TrackerState(input_index, current_time, input_object, uuid);
      new_tracker.setParameter(tracker_state_parameter_);
      return new_tracker;
    }
  }
  // if not found, just create new tracker
  auto new_tracker = TrackerState(input_index, current_time, input_object);
  new_tracker.setParameter(tracker_state_parameter_);
  return new_tracker;
}

}  // namespace tracking_object_merger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tracking_object_merger::DecorativeTrackerMergerNode)
