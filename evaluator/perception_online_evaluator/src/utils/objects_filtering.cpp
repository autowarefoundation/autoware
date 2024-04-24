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

#include "perception_online_evaluator/utils/objects_filtering.hpp"

namespace perception_diagnostics
{
namespace filter
{
bool velocity_filter(const PredictedObject & object, double velocity_threshold, double max_velocity)
{
  const auto v_norm = std::hypot(
    object.kinematics.initial_twist_with_covariance.twist.linear.x,
    object.kinematics.initial_twist_with_covariance.twist.linear.y);
  return (velocity_threshold < v_norm && v_norm < max_velocity);
}
}  // namespace filter

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification)
{
  std::uint8_t label = ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  for (const auto & _class : classification) {
    if (highest_prob < _class.probability) {
      highest_prob = _class.probability;
      label = _class.label;
    }
  }

  return label;
}

bool isTargetObjectType(
  const PredictedObject & object, const ObjectTypesToCheck & target_object_types)
{
  const auto t = getHighestProbLabel(object.classification);

  return (
    (t == ObjectClassification::CAR && target_object_types.check_car) ||
    (t == ObjectClassification::TRUCK && target_object_types.check_truck) ||
    (t == ObjectClassification::BUS && target_object_types.check_bus) ||
    (t == ObjectClassification::TRAILER && target_object_types.check_trailer) ||
    (t == ObjectClassification::UNKNOWN && target_object_types.check_unknown) ||
    (t == ObjectClassification::BICYCLE && target_object_types.check_bicycle) ||
    (t == ObjectClassification::MOTORCYCLE && target_object_types.check_motorcycle) ||
    (t == ObjectClassification::PEDESTRIAN && target_object_types.check_pedestrian));
}

ClassObjectsMap separateObjectsByClass(const PredictedObjects & objects)
{
  ClassObjectsMap separated_objects;
  for (const auto & object : objects.objects) {
    const auto label = getHighestProbLabel(object.classification);
    separated_objects[label].objects.push_back(object);
    separated_objects[label].header = objects.header;
  }
  return separated_objects;
}

void filterObjectsByClass(
  PredictedObjects & objects, const ObjectTypesToCheck & target_object_types)
{
  const auto filter = [&](const auto & object) {
    return isTargetObjectType(object, target_object_types);
  };

  filterObjects(objects, filter);
}

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, const double velocity_threshold,
  const bool remove_above_threshold)
{
  if (remove_above_threshold) {
    return filterObjectsByVelocity(objects, -velocity_threshold, velocity_threshold);
  }
  return filterObjectsByVelocity(objects, velocity_threshold, std::numeric_limits<double>::max());
}

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, double velocity_threshold, double max_velocity)
{
  const auto filter = [&](const auto & object) {
    return filter::velocity_filter(object, velocity_threshold, max_velocity);
  };

  auto filtered = objects;
  filterObjects(filtered, filter);
  return filtered;
}
}  // namespace perception_diagnostics
