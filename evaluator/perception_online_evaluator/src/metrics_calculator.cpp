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

#include "perception_online_evaluator/metrics_calculator.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "object_recognition_utils/object_classification.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <tier4_autoware_utils/ros/uuid_helper.hpp>

namespace perception_diagnostics
{
using object_recognition_utils::convertLabelToString;
using tier4_autoware_utils::inverseTransformPoint;

std::optional<MetricStatMap> MetricsCalculator::calculate(const Metric & metric) const
{
  // clang-format off
  const bool use_past_objects = metric == Metric::lateral_deviation        ||
                                metric == Metric::yaw_deviation            ||
                                metric == Metric::predicted_path_deviation ||
                                metric == Metric::yaw_rate;
  // clang-format on

  // todo(kosuke55): todo separate function and add timestamp of checked objects to diagnostics
  if (use_past_objects) {
    if (object_map_.empty()) {
      return {};
    }
    // time delay is max element of parameters_->prediction_time_horizons
    const double time_delay = getTimeDelay();
    const auto target_stamp = current_stamp_ - rclcpp::Duration::from_seconds(time_delay);
    if (!hasPassedTime(target_stamp)) {
      return {};
    }
    const auto target_stamp_objects = getObjectsByStamp(target_stamp);

    // extract moving objects
    const auto moving_objects = filterObjectsByVelocity(
      target_stamp_objects, parameters_->stopped_velocity_threshold,
      /*remove_above_threshold=*/false);
    const auto class_moving_objects_map = separateObjectsByClass(moving_objects);

    // extract stopped objects
    const auto stopped_objects =
      filterObjectsByVelocity(target_stamp_objects, parameters_->stopped_velocity_threshold);
    const auto class_stopped_objects_map = separateObjectsByClass(stopped_objects);

    switch (metric) {
      case Metric::lateral_deviation:
        return calcLateralDeviationMetrics(class_moving_objects_map);
      case Metric::yaw_deviation:
        return calcYawDeviationMetrics(class_moving_objects_map);
      case Metric::predicted_path_deviation:
        return calcPredictedPathDeviationMetrics(class_moving_objects_map);
      case Metric::yaw_rate:
        return calcYawRateMetrics(class_stopped_objects_map);
      default:
        return {};
    }
  }

  // use latest objects
  switch (metric) {
    case Metric::objects_count:
      return calcObjectsCountMetrics();
    default:
      return {};
  }
}

double MetricsCalculator::getTimeDelay() const
{
  const auto max_element_it = std::max_element(
    parameters_->prediction_time_horizons.begin(), parameters_->prediction_time_horizons.end());
  if (max_element_it != parameters_->prediction_time_horizons.end()) {
    return *max_element_it;
  }
  throw std::runtime_error("prediction_time_horizons is empty");
}

bool MetricsCalculator::hasPassedTime(const std::string uuid, const rclcpp::Time stamp) const
{
  if (object_map_.find(uuid) == object_map_.end()) {
    return false;
  }
  const auto oldest_stamp = object_map_.at(uuid).begin()->first;
  return oldest_stamp <= stamp;
}

bool MetricsCalculator::hasPassedTime(const rclcpp::Time stamp) const
{
  std::vector<rclcpp::Time> timestamps;
  for (const auto & [uuid, stamp_and_objects] : object_map_) {
    if (!stamp_and_objects.empty()) {
      timestamps.push_back(stamp_and_objects.begin()->first);
    }
  }

  auto it = std::min_element(timestamps.begin(), timestamps.end());
  if (it != timestamps.end()) {
    rclcpp::Time oldest_stamp = *it;
    if (oldest_stamp > stamp) {
      return false;
    }
  }
  return true;
}

rclcpp::Time MetricsCalculator::getClosestStamp(const rclcpp::Time stamp) const
{
  rclcpp::Time closest_stamp;
  rclcpp::Duration min_duration =
    rclcpp::Duration::from_nanoseconds(std::numeric_limits<int64_t>::max());

  for (const auto & [uuid, stamp_and_objects] : object_map_) {
    if (!stamp_and_objects.empty()) {
      auto it = std::lower_bound(
        stamp_and_objects.begin(), stamp_and_objects.end(), stamp,
        [](const auto & pair, const rclcpp::Time & val) { return pair.first < val; });

      // check the upper bound
      if (it != stamp_and_objects.end()) {
        const auto duration = it->first - stamp;
        if (std::abs(duration.nanoseconds()) < min_duration.nanoseconds()) {
          min_duration = duration;
          closest_stamp = it->first;
        }
      }

      // check the lower bound (if it is not the first element)
      if (it != stamp_and_objects.begin()) {
        const auto prev_it = std::prev(it);
        const auto duration = stamp - prev_it->first;
        if (std::abs(duration.nanoseconds()) < min_duration.nanoseconds()) {
          min_duration = duration;
          closest_stamp = prev_it->first;
        }
      }
    }
  }

  return closest_stamp;
}

std::optional<StampObjectMapIterator> MetricsCalculator::getClosestObjectIterator(
  const std::string & uuid, const rclcpp::Time & stamp) const
{
  const auto closest_stamp = getClosestStamp(stamp);
  const auto it = std::lower_bound(
    object_map_.at(uuid).begin(), object_map_.at(uuid).end(), closest_stamp,
    [](const auto & pair, const rclcpp::Time & val) { return pair.first < val; });

  return it != object_map_.at(uuid).end() ? std::optional<StampObjectMapIterator>(it)
                                          : std::nullopt;
}

std::optional<PredictedObject> MetricsCalculator::getObjectByStamp(
  const std::string uuid, const rclcpp::Time stamp) const
{
  constexpr double eps = 0.01;
  constexpr double close_time_threshold = 0.1;

  const auto obj_it_opt = getClosestObjectIterator(uuid, stamp);
  if (obj_it_opt.has_value()) {
    const auto it = obj_it_opt.value();
    if (std::abs((it->first - getClosestStamp(stamp)).seconds()) < eps) {
      const double time_diff = std::abs((it->first - stamp).seconds());
      if (time_diff < close_time_threshold) {
        return it->second;
      }
    }
  }
  return std::nullopt;
}

std::optional<std::pair<rclcpp::Time, PredictedObject>> MetricsCalculator::getPreviousObjectByStamp(
  const std::string uuid, const rclcpp::Time stamp) const
{
  const auto obj_it_opt = getClosestObjectIterator(uuid, stamp);
  if (obj_it_opt.has_value()) {
    auto it = obj_it_opt.value();
    if (it != object_map_.at(uuid).begin()) {
      // If it is exactly the closest stamp, move one back to get the previous
      if (it->first == getClosestStamp(stamp)) {
        --it;
      } else {
        // If it is not the closest stamp, it already points to the previous one due to lower_bound
      }
      return std::make_pair(it->first, it->second);
    }
  }
  return std::nullopt;
}

PredictedObjects MetricsCalculator::getObjectsByStamp(const rclcpp::Time stamp) const
{
  const auto closest_stamp = getClosestStamp(stamp);

  PredictedObjects objects;
  objects.header.stamp = stamp;
  for (const auto & [uuid, stamp_and_objects] : object_map_) {
    auto it = std::lower_bound(
      stamp_and_objects.begin(), stamp_and_objects.end(), closest_stamp,
      [](const auto & pair, const rclcpp::Time & val) { return pair.first < val; });

    // add the object only if the element pointed to by lower_bound is equal to stamp
    if (it != stamp_and_objects.end() && it->first == closest_stamp) {
      objects.objects.push_back(it->second);
    }
  }
  return objects;
}

MetricStatMap MetricsCalculator::calcLateralDeviationMetrics(
  const ClassObjectsMap & class_objects_map) const
{
  MetricStatMap metric_stat_map{};
  for (const auto & [label, objects] : class_objects_map) {
    if (
      parameters_->object_parameters.find(label) == parameters_->object_parameters.end() ||
      !parameters_->object_parameters.at(label).check_lateral_deviation) {
      continue;
    }
    Stat<double> stat{};
    const auto stamp = rclcpp::Time(objects.header.stamp);
    for (const auto & object : objects.objects) {
      const auto uuid = tier4_autoware_utils::toHexString(object.object_id);
      if (!hasPassedTime(uuid, stamp)) {
        continue;
      }
      const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
      const auto history_path = history_path_map_.at(uuid).second;
      if (history_path.empty()) {
        continue;
      }
      stat.add(metrics::calcLateralDeviation(history_path, object_pose));
    }
    metric_stat_map["lateral_deviation_" + convertLabelToString(label)] = stat;
  }
  return metric_stat_map;
}

MetricStatMap MetricsCalculator::calcYawDeviationMetrics(
  const ClassObjectsMap & class_objects_map) const
{
  MetricStatMap metric_stat_map{};
  for (const auto & [label, objects] : class_objects_map) {
    if (
      parameters_->object_parameters.find(label) == parameters_->object_parameters.end() ||
      !parameters_->object_parameters.at(label).check_yaw_deviation) {
      continue;
    }
    Stat<double> stat{};
    const auto stamp = rclcpp::Time(objects.header.stamp);
    for (const auto & object : objects.objects) {
      const auto uuid = tier4_autoware_utils::toHexString(object.object_id);
      if (!hasPassedTime(uuid, stamp)) {
        continue;
      }
      const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
      const auto history_path = history_path_map_.at(uuid).second;
      if (history_path.empty()) {
        continue;
      }
      stat.add(metrics::calcYawDeviation(history_path, object_pose));
    }
    metric_stat_map["yaw_deviation_" + convertLabelToString(label)] = stat;
  }
  return metric_stat_map;
}

MetricStatMap MetricsCalculator::calcPredictedPathDeviationMetrics(
  const ClassObjectsMap & class_objects_map) const
{
  const auto time_horizons = parameters_->prediction_time_horizons;

  MetricStatMap metric_stat_map{};
  for (const auto & [label, objects] : class_objects_map) {
    if (
      parameters_->object_parameters.find(label) == parameters_->object_parameters.end() ||
      !parameters_->object_parameters.at(label).check_predicted_path_deviation) {
      continue;
    }
    for (const double time_horizon : time_horizons) {
      const auto metrics = calcPredictedPathDeviationMetrics(objects, time_horizon);
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(2) << time_horizon;
      std::string time_horizon_str = stream.str();
      metric_stat_map
        ["predicted_path_deviation_" + convertLabelToString(label) + "_" + time_horizon_str] =
          metrics.mean;
      metric_stat_map
        ["predicted_path_deviation_variance_" + convertLabelToString(label) + "_" +
         time_horizon_str] = metrics.variance;
    }
  }
  return metric_stat_map;
}

PredictedPathDeviationMetrics MetricsCalculator::calcPredictedPathDeviationMetrics(
  const PredictedObjects & objects, const double time_horizon) const
{
  // Step 1: For each object and its predicted paths, calculate the deviation between each predicted
  // path pose and the corresponding historical path pose. Store the deviations in
  // deviation_map_for_paths.
  std::unordered_map<std::string, std::unordered_map<size_t, std::vector<double>>>
    deviation_map_for_paths;

  // For debugging: Save the pairs of predicted path pose and history path pose.
  std::unordered_map<std::string, std::vector<std::pair<Pose, Pose>>>
    debug_predicted_path_pairs_map;

  const auto stamp = objects.header.stamp;
  for (const auto & object : objects.objects) {
    const auto uuid = tier4_autoware_utils::toHexString(object.object_id);
    const auto predicted_paths = object.kinematics.predicted_paths;
    for (size_t i = 0; i < predicted_paths.size(); i++) {
      const auto predicted_path = predicted_paths[i];
      const std::string path_id = uuid + "_" + std::to_string(i);
      for (size_t j = 0; j < predicted_path.path.size(); j++) {
        const double time_duration =
          rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(j);
        if (time_duration > time_horizon) {
          break;
        }
        const rclcpp::Time target_stamp =
          rclcpp::Time(stamp) + rclcpp::Duration::from_seconds(time_duration);
        if (!hasPassedTime(uuid, target_stamp)) {
          continue;
        }
        const auto history_object_opt = getObjectByStamp(uuid, target_stamp);
        if (!history_object_opt.has_value()) {
          continue;
        }
        const auto history_object = history_object_opt.value();
        const auto history_pose = history_object.kinematics.initial_pose_with_covariance.pose;
        const Pose & p = predicted_path.path[j];
        const double distance =
          tier4_autoware_utils::calcDistance2d(p.position, history_pose.position);
        deviation_map_for_paths[uuid][i].push_back(distance);

        // Save debug information
        debug_predicted_path_pairs_map[path_id].push_back(std::make_pair(p, history_pose));
      }
    }
  }

  // Step 2: For each object, select the predicted path with the smallest mean deviation.
  // Store the selected path's deviations in deviation_map_for_objects.
  std::unordered_map<std::string, std::vector<double>> deviation_map_for_objects;
  for (const auto & [uuid, deviation_map] : deviation_map_for_paths) {
    std::optional<std::pair<size_t, double>> min_deviation;
    for (const auto & [i, deviations] : deviation_map) {
      if (deviations.empty()) {
        continue;
      }
      const double sum = std::accumulate(deviations.begin(), deviations.end(), 0.0);
      const double mean = sum / deviations.size();
      if (!min_deviation.has_value() || mean < min_deviation.value().second) {
        min_deviation = std::make_pair(i, mean);
      }
    }

    if (!min_deviation.has_value()) {
      continue;
    }

    // Save the delayed target object and the corresponding predicted path for debugging
    const auto [min_deviation_index, min_mean_deviation] = min_deviation.value();
    deviation_map_for_objects[uuid] = deviation_map.at(min_deviation_index);
    const auto path_id = uuid + "_" + std::to_string(min_deviation_index);
    const auto target_stamp_object = getObjectByStamp(uuid, stamp);
    if (target_stamp_object.has_value()) {
      ObjectData object_data;
      object_data.object = target_stamp_object.value();
      object_data.path_pairs = debug_predicted_path_pairs_map[path_id];
      debug_target_object_[uuid] = object_data;
    }
  }

  // Step 3: Calculate the mean and variance of the deviations for each object's selected predicted
  // path. Store the results in the PredictedPathDeviationMetrics structure.
  PredictedPathDeviationMetrics metrics;
  for (const auto & [object_id, object_path_deviations] : deviation_map_for_objects) {
    if (!object_path_deviations.empty()) {
      const double sum_of_deviations =
        std::accumulate(object_path_deviations.begin(), object_path_deviations.end(), 0.0);
      const double mean_deviation = sum_of_deviations / object_path_deviations.size();
      metrics.mean.add(mean_deviation);
      double sum_of_squared_deviations = 0.0;
      for (const auto path_point_deviation : object_path_deviations) {
        sum_of_squared_deviations += std::pow(path_point_deviation - mean_deviation, 2);
      }
      const double variance_deviation = sum_of_squared_deviations / object_path_deviations.size();

      metrics.variance.add(variance_deviation);
    }
  }

  return metrics;
}

MetricStatMap MetricsCalculator::calcYawRateMetrics(const ClassObjectsMap & class_objects_map) const
{
  // calculate yaw rate for each object

  MetricStatMap metric_stat_map{};
  for (const auto & [label, objects] : class_objects_map) {
    Stat<double> stat{};
    const auto stamp = rclcpp::Time(objects.header.stamp);

    for (const auto & object : objects.objects) {
      const auto uuid = tier4_autoware_utils::toHexString(object.object_id);
      if (!hasPassedTime(uuid, stamp)) {
        continue;
      }
      const auto previous_object_with_stamp_opt = getPreviousObjectByStamp(uuid, stamp);
      if (!previous_object_with_stamp_opt.has_value()) {
        continue;
      }
      const auto [previous_stamp, previous_object] = previous_object_with_stamp_opt.value();

      const double time_diff = (stamp - previous_stamp).seconds();
      if (time_diff < 0.01) {
        continue;
      }
      const double current_yaw =
        tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation);
      const double previous_yaw =
        tf2::getYaw(previous_object.kinematics.initial_pose_with_covariance.pose.orientation);
      // Calculate the absolute difference between current_yaw and previous_yaw
      const double yaw_diff =
        std::abs(tier4_autoware_utils::normalizeRadian(current_yaw - previous_yaw));
      // The yaw difference is flipped if the angle is larger than 90 degrees
      const double yaw_diff_flip_fixed = std::min(yaw_diff, M_PI - yaw_diff);
      const double yaw_rate = yaw_diff_flip_fixed / time_diff;
      stat.add(yaw_rate);
    }
    metric_stat_map["yaw_rate_" + convertLabelToString(label)] = stat;
  }
  return metric_stat_map;
}

MetricStatMap MetricsCalculator::calcObjectsCountMetrics() const
{
  MetricStatMap metric_stat_map;
  // calculate the average number of objects in the detection area in all past frames
  const auto overall_average_count = detection_counter_.getOverallAverageCount();
  for (const auto & [label, range_and_count] : overall_average_count) {
    for (const auto & [range, count] : range_and_count) {
      metric_stat_map["average_objects_count_" + convertLabelToString(label) + "_" + range].add(
        count);
    }
  }
  // calculate the average number of objects in the detection area in the past
  // `objects_count_window_seconds`
  const auto average_count =
    detection_counter_.getAverageCount(parameters_->objects_count_window_seconds);
  for (const auto & [label, range_and_count] : average_count) {
    for (const auto & [range, count] : range_and_count) {
      metric_stat_map["interval_average_objects_count_" + convertLabelToString(label) + "_" + range]
        .add(count);
    }
  }

  // calculate the total number of objects in the detection area
  const auto total_count = detection_counter_.getTotalCount();
  for (const auto & [label, range_and_count] : total_count) {
    for (const auto & [range, count] : range_and_count) {
      metric_stat_map["total_objects_count_" + convertLabelToString(label) + "_" + range].add(
        count);
    }
  }

  return metric_stat_map;
}

void MetricsCalculator::setPredictedObjects(
  const PredictedObjects & objects, const tf2_ros::Buffer & tf_buffer)
{
  current_stamp_ = objects.header.stamp;

  // store objects to check deviation
  {
    using tier4_autoware_utils::toHexString;
    for (const auto & object : objects.objects) {
      std::string uuid = toHexString(object.object_id);
      updateObjects(uuid, current_stamp_, object);
    }
    deleteOldObjects(current_stamp_);
    updateHistoryPath();
  }

  // store objects to calculate object count
  detection_counter_.addObjects(objects, tf_buffer);
}

void MetricsCalculator::deleteOldObjects(const rclcpp::Time stamp)
{
  // delete the data older than 2*time_delay_
  const double time_delay = getTimeDelay();
  for (auto & [uuid, stamp_and_objects] : object_map_) {
    for (auto it = stamp_and_objects.begin(); it != stamp_and_objects.end();) {
      if (it->first < stamp - rclcpp::Duration::from_seconds(time_delay * 2)) {
        it = stamp_and_objects.erase(it);
      } else {
        break;
      }
    }
  }

  const auto object_map = object_map_;
  for (const auto & [uuid, stamp_and_objects] : object_map) {
    if (stamp_and_objects.empty()) {
      object_map_.erase(uuid);
      history_path_map_.erase(uuid);
      debug_target_object_.erase(uuid);  // debug
    }
  }
}

void MetricsCalculator::updateObjects(
  const std::string uuid, const rclcpp::Time stamp, const PredictedObject & object)
{
  object_map_[uuid][stamp] = object;
}

void MetricsCalculator::updateHistoryPath()
{
  const double window_size = parameters_->smoothing_window_size;

  for (const auto & [uuid, stamp_and_objects] : object_map_) {
    std::vector<Pose> history_path;
    for (auto it = stamp_and_objects.begin(); it != stamp_and_objects.end(); ++it) {
      const auto & [stamp, object] = *it;

      // skip if the object is stopped
      // calculate velocity from previous object
      if (it != stamp_and_objects.begin()) {
        const auto & [prev_stamp, prev_object] = *std::prev(it);
        const double time_diff = (stamp - prev_stamp).seconds();
        if (time_diff < 0.01) {
          continue;
        }
        const auto current_pose = object.kinematics.initial_pose_with_covariance.pose;
        const auto prev_pose = prev_object.kinematics.initial_pose_with_covariance.pose;
        const auto velocity =
          tier4_autoware_utils::calcDistance2d(current_pose.position, prev_pose.position) /
          time_diff;
        if (velocity < parameters_->stopped_velocity_threshold) {
          continue;
        }
      }
      if (
        std::hypot(
          object.kinematics.initial_twist_with_covariance.twist.linear.x,
          object.kinematics.initial_twist_with_covariance.twist.linear.y) <
        parameters_->stopped_velocity_threshold) {
        continue;
      }

      history_path.push_back(object.kinematics.initial_pose_with_covariance.pose);
    }

    // pair of history_path(raw) and smoothed_history_path
    // history_path(raw) is just for debugging
    history_path_map_[uuid] =
      std::make_pair(history_path, averageFilterPath(history_path, window_size));
  }
}

std::vector<Pose> MetricsCalculator::generateHistoryPathWithPrev(
  const std::vector<Pose> & prev_history_path, const Pose & new_pose, const size_t window_size)
{
  std::vector<Pose> history_path;
  const size_t half_window_size = static_cast<size_t>(window_size / 2);
  history_path.insert(
    history_path.end(), prev_history_path.begin(), prev_history_path.end() - half_window_size);

  std::vector<Pose> updated_poses;
  updated_poses.insert(
    updated_poses.end(), prev_history_path.end() - half_window_size * 2, prev_history_path.end());
  updated_poses.push_back(new_pose);

  updated_poses = averageFilterPath(updated_poses, window_size);
  history_path.insert(
    history_path.end(), updated_poses.begin() + half_window_size, updated_poses.end());
  return history_path;
}

std::vector<Pose> MetricsCalculator::averageFilterPath(
  const std::vector<Pose> & path, const size_t window_size) const
{
  using tier4_autoware_utils::calcAzimuthAngle;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createQuaternionFromYaw;

  std::vector<Pose> filtered_path;
  filtered_path.reserve(path.size());  // Reserve space to avoid reallocations

  const size_t half_window = static_cast<size_t>(window_size / 2);

  // Calculate the moving average for positions
  for (size_t i = 0; i < path.size(); ++i) {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    size_t valid_points = 0;  // Correctly initialize and use as counter

    for (size_t j = std::max(i - half_window, static_cast<size_t>(0));
         j <= std::min(i + half_window, path.size() - 1); ++j) {
      sum_x += path[j].position.x;
      sum_y += path[j].position.y;
      sum_z += path[j].position.z;
      ++valid_points;
    }

    Pose average_pose;
    if (valid_points > 0) {  // Prevent division by zero
      average_pose.position.x = sum_x / valid_points;
      average_pose.position.y = sum_y / valid_points;
      average_pose.position.z = sum_z / valid_points;
    } else {
      average_pose.position = path.at(i).position;
    }

    // skip if the points are too close
    if (
      filtered_path.size() > 0 &&
      calcDistance2d(filtered_path.back().position, average_pose.position) < 0.5) {
      continue;
    }

    // skip if the difference between the current orientation and the azimuth angle is large
    if (i > 0) {
      const double azimuth = calcAzimuthAngle(path.at(i - 1).position, path.at(i).position);
      const double yaw = tf2::getYaw(path.at(i).orientation);
      if (tier4_autoware_utils::normalizeRadian(yaw - azimuth) > M_PI_2) {
        continue;
      }
    }

    // Placeholder for orientation to ensure structure integrity
    average_pose.orientation = path.at(i).orientation;
    filtered_path.push_back(average_pose);
  }

  // delete pose if the difference between the azimuth angle of the previous and next poses is large
  if (filtered_path.size() > 2) {
    auto it = filtered_path.begin() + 2;
    while (it != filtered_path.end()) {
      const double azimuth_to_prev = calcAzimuthAngle((it - 2)->position, (it - 1)->position);
      const double azimuth_to_current = calcAzimuthAngle((it - 1)->position, it->position);
      if (
        std::abs(tier4_autoware_utils::normalizeRadian(azimuth_to_prev - azimuth_to_current)) >
        M_PI_2) {
        it = filtered_path.erase(it);
        continue;
      }
      ++it;
    }
  }

  // Calculate yaw and convert to quaternion after averaging positions
  for (size_t i = 0; i < filtered_path.size(); ++i) {
    Pose & p = filtered_path[i];
    if (i < filtered_path.size() - 1) {
      const double azimuth_to_next = calcAzimuthAngle(p.position, filtered_path[i + 1].position);
      filtered_path[i].orientation = createQuaternionFromYaw(azimuth_to_next);
    } else if (filtered_path.size() > 1) {
      // For the last point, use the orientation of the second-to-last point
      p.orientation = filtered_path[i - 1].orientation;
    }
  }

  return filtered_path;
}

}  // namespace perception_diagnostics
