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

#include "tracking_object_merger/utils/tracker_state.hpp"

#include "tracking_object_merger/utils/utils.hpp"

using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

/**
 * @brief Construct a new Tracker State:: Tracker State object
 *
 * @param input_source : input source distinguished
 * @param tracked_object : input source tracked object
 * @param last_update_time : last update time
 */
TrackerState::TrackerState(
  const MEASUREMENT_STATE input_source, const rclcpp::Time & last_update_time,
  const autoware_auto_perception_msgs::msg::TrackedObject & tracked_object)
: tracked_object_(tracked_object),
  last_update_time_(last_update_time),
  const_uuid_(tracked_object.object_id),
  measurement_state_(input_source)
{
  input_uuid_map_[input_source] = tracked_object_.object_id;
  last_updated_time_map_[input_source] = last_update_time;
  existence_probability_ = default_existence_probability_map_[input_source];
}

TrackerState::TrackerState(
  const MEASUREMENT_STATE input_source, const rclcpp::Time & last_update_time,
  const autoware_auto_perception_msgs::msg::TrackedObject & tracked_object,
  const unique_identifier_msgs::msg::UUID & uuid)
: tracked_object_(tracked_object),
  last_update_time_(last_update_time),
  const_uuid_(uuid),
  measurement_state_(input_source)
{
  input_uuid_map_[input_source] = tracked_object_.object_id;
  last_updated_time_map_[input_source] = last_update_time;
  existence_probability_ = default_existence_probability_map_[input_source];
}

void TrackerState::setParameter(const TrackerStateParameter & parameter)
{
  max_dt_ = parameter.max_dt;
  publish_probability_threshold_ = parameter.publish_probability_threshold;
  remove_probability_threshold_ = parameter.remove_probability_threshold;
  default_existence_probability_map_.at(MEASUREMENT_STATE::LIDAR) =
    parameter.default_lidar_existence_probability;
  default_existence_probability_map_.at(MEASUREMENT_STATE::RADAR) =
    parameter.default_radar_existence_probability;
  default_existence_probability_map_.at(MEASUREMENT_STATE::CAMERA) =
    parameter.default_camera_existence_probability;
  decay_rate_ = parameter.decay_rate;
}

/**
 * @brief Predict state to current time
 *
 * @param current_time
 * @return true
 * @return false
 */
bool TrackerState::predict(const rclcpp::Time & current_time)
{
  // get dt and give warning if dt is too large
  double dt = (current_time - last_update_time_).seconds();
  if (std::abs(dt) > max_dt_) {
    std::cerr << "[tracking_object_merger] dt is too large: " << dt << std::endl;
    return false;
  }

  // do prediction
  last_update_time_ = current_time;
  return this->predict(dt, utils::predictPastOrFutureTrackedObject);
}

/**
 * @brief Predict state to current time with given update function
 *
 * @param dt : time to predict
 * @param func : update function (e.g. PredictPastOrFutureTrackedObject)
 * @return true: prediction succeeded
 * @return false: prediction failed
 */
bool TrackerState::predict(
  const double dt, std::function<TrackedObject(const TrackedObject &, double)> func)
{
  const auto predicted_object = func(tracked_object_, dt);
  tracked_object_ = predicted_object;
  return true;
}

// get current measurement state
MEASUREMENT_STATE TrackerState::getCurrentMeasurementState(const rclcpp::Time & current_time) const
{
  MEASUREMENT_STATE measurement_state = MEASUREMENT_STATE::NONE;
  // check LIDAR
  if (last_updated_time_map_.find(MEASUREMENT_STATE::LIDAR) != last_updated_time_map_.end()) {
    if ((current_time - last_updated_time_map_.at(MEASUREMENT_STATE::LIDAR)).seconds() < max_dt_) {
      measurement_state = measurement_state | MEASUREMENT_STATE::LIDAR;
    }
  }
  // check RADAR
  if (last_updated_time_map_.find(MEASUREMENT_STATE::RADAR) != last_updated_time_map_.end()) {
    if ((current_time - last_updated_time_map_.at(MEASUREMENT_STATE::RADAR)).seconds() < max_dt_) {
      measurement_state = measurement_state | MEASUREMENT_STATE::RADAR;
    }
  }
  // check CAMERA
  if (last_updated_time_map_.find(MEASUREMENT_STATE::CAMERA) != last_updated_time_map_.end()) {
    if ((current_time - last_updated_time_map_.at(MEASUREMENT_STATE::CAMERA)).seconds() < max_dt_) {
      measurement_state = measurement_state | MEASUREMENT_STATE::CAMERA;
    }
  }
  return measurement_state;
}

bool TrackerState::updateState(
  const MEASUREMENT_STATE input, const rclcpp::Time & current_time,
  const TrackedObject & tracked_object)
{
  // calc dt
  double dt = (current_time - last_update_time_).seconds();
  if (dt < 0.0) {
    std::cerr << "[tracking_object_merger] dt is negative: " << dt << std::endl;
    return false;
  }

  // predict
  if (dt > 0.0) {
    this->predict(dt, utils::predictPastOrFutureTrackedObject);
  }

  // get current measurement state
  measurement_state_ = getCurrentMeasurementState(current_time);

  // update with input
  if (input & MEASUREMENT_STATE::LIDAR) {
    updateWithLIDAR(current_time, tracked_object);
  }
  if (input & MEASUREMENT_STATE::RADAR) {
    updateWithRADAR(current_time, tracked_object);
  }
  if (input & MEASUREMENT_STATE::CAMERA) {
    updateWithCAMERA(current_time, tracked_object);
  }
  return true;
}

void TrackerState::updateWithCAMERA(
  const rclcpp::Time & current_time, const TrackedObject & tracked_object)
{
  // update tracked object
  updateWithFunction(
    MEASUREMENT_STATE::CAMERA, current_time, tracked_object,
    merger_utils::updateOnlyClassification);
}

void TrackerState::updateWithLIDAR(
  const rclcpp::Time & current_time, const TrackedObject & tracked_object)
{
  // if radar is available, do not update velocity
  if (measurement_state_ & MEASUREMENT_STATE::RADAR) {
    // update tracked object
    updateWithFunction(
      MEASUREMENT_STATE::LIDAR, current_time, tracked_object, merger_utils::updateExceptVelocity);
  } else {
    // else just update tracked object
    updateWithFunction(
      MEASUREMENT_STATE::LIDAR, current_time, tracked_object,
      merger_utils::updateWholeTrackedObject);
  }
}

void TrackerState::updateWithRADAR(
  const rclcpp::Time & current_time, const TrackedObject & tracked_object)
{
  // if lidar is available, update only velocity
  if (measurement_state_ & MEASUREMENT_STATE::LIDAR) {
    // update tracked object
    updateWithFunction(
      MEASUREMENT_STATE::RADAR, current_time, tracked_object,
      merger_utils::updateOnlyObjectVelocity);
  } else {
    // else just update tracked object
    updateWithFunction(
      MEASUREMENT_STATE::RADAR, current_time, tracked_object,
      merger_utils::updateWholeTrackedObject);
  }
}

bool TrackerState::updateWithFunction(
  const MEASUREMENT_STATE input, const rclcpp::Time & current_time,
  const TrackedObject & input_tracked_object,
  std::function<void(TrackedObject &, const TrackedObject &)> update_func)
{
  // put input uuid and last update time
  if (current_time > last_update_time_) {
    const auto predict_successful = predict(current_time);
    if (!predict_successful) {
      return false;
    }
  }

  // update with measurement state
  last_updated_time_map_[input] = current_time;
  input_uuid_map_[input] = input_tracked_object.object_id;
  measurement_state_ = measurement_state_ | input;
  existence_probability_ =
    std::max(existence_probability_, default_existence_probability_map_[input]);

  // update tracked object
  update_func(tracked_object_, input_tracked_object);
  tracked_object_.object_id = const_uuid_;  // overwrite uuid to stay same
  return true;
}

void TrackerState::updateWithoutSensor(const rclcpp::Time & current_time)
{
  // calc dt
  double dt = (current_time - last_update_time_).seconds();
  if (dt < 0) {
    std::cerr << "[tracking_object_merger] dt is negative: " << dt << std::endl;
    return;
  }

  // predict
  if (dt > 0.0) {
    const auto predict_successful = this->predict(dt, utils::predictPastOrFutureTrackedObject);
    if (!predict_successful) {
      existence_probability_ = 0.0;  // remove object
    }
  }

  // reduce probability
  existence_probability_ -= decay_rate_;
  if (existence_probability_ < 0.0) {
    existence_probability_ = 0.0;
  }
}

TrackedObject TrackerState::getObject() const
{
  TrackedObject tracked_object = tracked_object_;
  tracked_object.object_id = const_uuid_;
  tracked_object.existence_probability =
    static_cast<float>(existence_probability_);  // double -> float
  return tracked_object;
}

bool TrackerState::hasUUID(
  const MEASUREMENT_STATE input, const unique_identifier_msgs::msg::UUID & uuid) const
{
  if (input_uuid_map_.find(input) == input_uuid_map_.end()) {
    return false;
  }
  return input_uuid_map_.at(input) == uuid;
}

bool TrackerState::isValid() const
{
  // check if tracker state is valid
  if (existence_probability_ < remove_probability_threshold_) {
    return false;
  }
  return true;
}

bool TrackerState::canPublish() const
{
  // check if tracker state is valid
  if (existence_probability_ < publish_probability_threshold_) {
    return false;
  }
  return true;
}

TrackerState::~TrackerState()
{
  // destructor
}

TrackedObjects getTrackedObjectsFromTrackerStates(
  std::vector<TrackerState> & tracker_states, const rclcpp::Time & current_time)
{
  TrackedObjects tracked_objects;

  // sanitize and get tracked objects
  for (auto it = tracker_states.begin(); it != tracker_states.end();) {
    // check if tracker state is valid
    if (it->isValid()) {
      if (it->canPublish()) {
        // if valid, get tracked object
        tracked_objects.objects.push_back(it->getObject());
      }
      ++it;
    } else {
      // if not valid, delete tracker state
      it = tracker_states.erase(it);
    }
  }

  // update header
  tracked_objects.header.stamp = current_time;
  tracked_objects.header.frame_id = "map";  // TODO(yoshiri): get frame_id from input
  return tracked_objects;
}
