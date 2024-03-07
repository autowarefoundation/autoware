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

#ifndef PERCEPTION_ONLINE_EVALUATOR__UTILS__OBJECTS_FILTERING_HPP_
#define PERCEPTION_ONLINE_EVALUATOR__UTILS__OBJECTS_FILTERING_HPP_

#include "perception_online_evaluator/parameters.hpp"

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 *  most of this file is copied from objects_filtering.hpp in safety_check of behavior_path_planner
 */

namespace perception_diagnostics
{

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;

using ClassObjectsMap = std::unordered_map<uint8_t, PredictedObjects>;

std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification);

/**
 * @brief Specifies which object class should be checked.
 */
struct ObjectTypesToCheck
{
  bool check_car{true};         ///< Check for cars.
  bool check_truck{true};       ///< Check for trucks.
  bool check_bus{true};         ///< Check for buses.
  bool check_trailer{true};     ///< Check for trailers.
  bool check_unknown{true};     ///< Check for unknown object types.
  bool check_bicycle{true};     ///< Check for bicycles.
  bool check_motorcycle{true};  ///< Check for motorcycles.
  bool check_pedestrian{true};  ///< Check for pedestrians.
};

/**
 * @brief Determines whether the predicted object type matches any of the target object types
 * specified by the user.
 *
 * @param object The predicted object whose type is to be checked.
 * @param target_object_types A structure containing boolean flags for each object type that the
 * user is interested in checking.
 *
 * @return Returns true if the predicted object's highest probability label matches any of the
 * specified target object types.
 */
bool isTargetObjectType(
  const PredictedObject & object, const ObjectTypesToCheck & target_object_types);

/**
 * @brief Filters objects in the 'selected' container based on the provided filter function.
 *
 * This function partitions the 'selected' container based on the 'filter' function
 * and moves objects that satisfy the filter condition to the 'removed' container.
 *
 * @tparam Func The type of the filter function.
 * @param selected [in,out] The container of objects to be filtered.
 * @param removed [out] The container where objects not satisfying the filter condition are moved.
 * @param filter The filter function that determines whether an object should be removed.
 */
template <typename Func>
void filterObjects(PredictedObjects & selected, PredictedObjects & removed, Func filter)
{
  auto partitioned = std::partition(selected.objects.begin(), selected.objects.end(), filter);
  removed.objects.insert(removed.objects.end(), partitioned, selected.objects.end());
  selected.objects.erase(partitioned, selected.objects.end());
}

/**
 * @brief Filters objects in the 'objects' container based on the provided filter function.
 *
 * This function is an overload that simplifies filtering when you don't need to specify a
 * separate 'removed' container. It internally creates a 'removed_objects' container and calls the
 * main 'filterObjects' function.
 *
 * @tparam Func The type of the filter function.
 * @param objects [in,out] The container of objects to be filtered.
 * @param filter The filter function that determines whether an object should be removed.
 */
template <typename Func>
void filterObjects(PredictedObjects & objects, Func filter)
{
  [[maybe_unused]] PredictedObjects removed_objects{};
  filterObjects(objects, removed_objects, filter);
}

/**
 * @brief Separates the provided objects based on their classification.
 *
 * @param objects The predicted objects to be separated.
 * @return A map of objects separated by their classification.
 */
ClassObjectsMap separateObjectsByClass(const PredictedObjects & objects);

/**
 * @brief Filters the provided objects based on their classification.
 *
 * @param objects The predicted objects to be filtered.
 * @param target_object_types The types of objects to retain after filtering.
 */
void filterObjectsByClass(
  PredictedObjects & objects, const ObjectTypesToCheck & target_object_types);

/**
 * @brief Filters the provided objects based on their classification.
 *
 * @param objects The predicted objects to be filtered.
 * @param params The parameters for each object class.
 */
void filterDeviationCheckObjects(
  PredictedObjects & objects, const std::unordered_map<uint8_t, ObjectParameter> & params);

}  // namespace perception_diagnostics

#endif  // PERCEPTION_ONLINE_EVALUATOR__UTILS__OBJECTS_FILTERING_HPP_
