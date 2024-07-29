// Copyright 2024 The Autoware Contributors
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

#include "autoware/traffic_light_arbiter/signal_match_validator.hpp"

namespace util
{
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
using Element = autoware_perception_msgs::msg::TrafficLightElement;
using Time = builtin_interfaces::msg::Time;

// Finds a signal by its ID within a TrafficSignalArray
std::optional<TrafficSignal> find_signal_by_id(
  const std::unordered_map<lanelet::Id, TrafficSignal> & id_signal_map, int64_t signal_id)
{
  auto it = id_signal_map.find(signal_id);
  if (it != id_signal_map.end()) {
    return it->second;  // Return the found signal
  } else {
    return std::nullopt;  // Return an empty optional if not found
  }
}

// Creates a map from signal IDs to TrafficSignal objects.
std::unordered_map<lanelet::Id, TrafficSignal> create_id_signal_map(
  const TrafficSignalArray & traffic_signals)
{
  std::unordered_map<lanelet::Id, TrafficSignal> id_signal_map;
  for (const auto & traffic_signal : traffic_signals.traffic_light_groups) {
    id_signal_map[traffic_signal.traffic_light_group_id] = traffic_signal;
  }

  return id_signal_map;
}

// Creates a TrafficSignalElement with specified attributes
Element create_element(
  const Element::_color_type & color, const Element::_shape_type & shape,
  const Element::_status_type & status, const Element::_confidence_type & confidence)
{
  Element signal_element;
  signal_element.color = color;
  signal_element.shape = shape;
  signal_element.status = status;
  signal_element.confidence = confidence;

  return signal_element;
}

// Creates unknown elements for each unique shape from two element vectors
std::vector<Element> create_unknown_elements(
  const std::vector<Element> & elements1, const std::vector<Element> & elements2)
{
  std::unordered_set<Element::_shape_type> shape_set;
  for (const auto & element : elements1) {
    shape_set.emplace(element.shape);
  }
  for (const auto & element : elements2) {
    shape_set.emplace(element.shape);
  }

  std::vector<Element> unknown_elements;
  for (const auto & shape : shape_set) {
    // Confidence is set to a default value as it is not relevant for unknown signals
    unknown_elements.emplace_back(
      create_element(Element::UNKNOWN, shape, Element::UNKNOWN, /* confidence */ 1.0));
  }

  return unknown_elements;
}

// Creates a 'unknown' signal with elements matching the shapes of a given signal's elements
TrafficSignal create_unknown_signal(const TrafficSignal & traffic_signal)
{
  TrafficSignal unknown_signal;
  unknown_signal.traffic_light_group_id = traffic_signal.traffic_light_group_id;
  for (const auto & element : traffic_signal.elements) {
    // Confidence is set to a default value as it is not relevant for unknown signals
    const auto unknown_element =
      create_element(Element::UNKNOWN, element.shape, Element::UNKNOWN, /* confidence */ 1.0);
    unknown_signal.elements.emplace_back(unknown_element);
  }

  return unknown_signal;
}

// Creates an 'unknown' signal by combining unique shapes from two signals' elements
TrafficSignal create_unknown_signal(const TrafficSignal & signal1, const TrafficSignal & signal2)
{
  TrafficSignal unknown_signal;

  // Assumes that both signals have the same traffic_signal_id
  unknown_signal.traffic_light_group_id = signal1.traffic_light_group_id;

  const auto unknown_elements = create_unknown_elements(signal1.elements, signal2.elements);
  for (const auto & element : unknown_elements) {
    unknown_signal.elements.emplace_back(element);
  }

  return unknown_signal;
}

// Checks if all elements in two vectors are equivalent
bool are_all_elements_equivalent(
  const std::vector<Element> & signal1, const std::vector<Element> & signal2)
{
  // Returns false if vectors have different sizes
  if (signal1.size() != signal2.size()) {
    return false;
  }

  // Sorts copies of the vectors by shape for comparison
  std::vector<Element> sorted_signal1 = signal1;
  std::vector<Element> sorted_signal2 = signal2;
  auto compare_by_shape = [](const Element & a, const Element & b) { return a.shape < b.shape; };
  std::sort(sorted_signal1.begin(), sorted_signal1.end(), compare_by_shape);
  std::sort(sorted_signal2.begin(), sorted_signal2.end(), compare_by_shape);

  // Returns true if sorted vectors are equal
  return std::equal(
    sorted_signal1.begin(), sorted_signal1.end(), sorted_signal2.begin(), sorted_signal2.end(),
    [](const Element & a, const Element & b) { return a.color == b.color && a.shape == b.shape; });
}

// Creates a set of unique signal IDs from two vectors of TrafficSignals
std::unordered_set<lanelet::Id> create_signal_id_set(
  const std::vector<TrafficSignal> & signals1, const std::vector<TrafficSignal> & signals2)
{
  std::unordered_set<lanelet::Id> signal_id_set;
  for (const auto & traffic_signal : signals1) {
    signal_id_set.emplace(traffic_signal.traffic_light_group_id);
  }
  for (const auto & traffic_signal : signals2) {
    signal_id_set.emplace(traffic_signal.traffic_light_group_id);
  }

  return signal_id_set;
}

// Returns the signal with the highest confidence elements, considering a external priority
TrafficSignal get_highest_confidence_signal(
  const std::optional<TrafficSignal> & perception_signal,
  const std::optional<TrafficSignal> & external_signal, const bool external_priority)
{
  // Returns the existing signal if only one of them exists
  if (!perception_signal) {
    return *external_signal;
  }
  if (!external_signal) {
    return *perception_signal;
  }

  // Gives priority to the external signal if external_priority is true
  if (external_priority) {
    return *external_signal;
  }

  // Compiles elements into a map by shape, to compare their confidences
  using Key = Element::_shape_type;
  std::map<Key, std::vector<Element>> shape_element_map;
  for (const auto & element : perception_signal->elements) {
    shape_element_map[element.shape].emplace_back(element);
  }
  for (const auto & element : external_signal->elements) {
    shape_element_map[element.shape].emplace_back(element);
  }

  TrafficSignal highest_confidence_signal;

  // Assumes that both signals have the same traffic_signal_id
  highest_confidence_signal.traffic_light_group_id = perception_signal->traffic_light_group_id;

  // For each shape, finds the element with the highest confidence and adds it to the signal
  for (const auto & [shape, elements] : shape_element_map) {
    const auto highest_confidence_element = std::max_element(
      elements.begin(), elements.end(),
      [](const Element & a, const Element & b) { return a.confidence < b.confidence; });
    highest_confidence_signal.elements.emplace_back(*highest_confidence_element);
  }

  return highest_confidence_signal;
}

// Determines the newer of two Time stamps
Time get_newer_stamp(const Time & stamp1, const Time & stamp2)
{
  // Returns stamp1 if it is newer than stamp2, otherwise returns stamp2
  if (stamp1.sec > stamp2.sec || (stamp1.sec == stamp2.sec && stamp1.nanosec > stamp2.nanosec)) {
    return stamp1;
  } else {
    return stamp2;
  }
}

}  // namespace util

autoware_perception_msgs::msg::TrafficLightGroupArray SignalMatchValidator::validateSignals(
  const TrafficSignalArray & perception_signals, const TrafficSignalArray & external_signals)
{
  TrafficSignalArray validated_signals;

  // Set newer stamp
  validated_signals.stamp = util::get_newer_stamp(perception_signals.stamp, external_signals.stamp);

  // Create a map from signals to reduce the calculation cost
  const auto perception_id_signal_map = util::create_id_signal_map(perception_signals);
  const auto external_id_signal_map = util::create_id_signal_map(external_signals);

  // Create the unique set of the received id,
  // then compare the signal element for each received signal id
  const auto received_signal_id_set = util::create_signal_id_set(
    perception_signals.traffic_light_groups, external_signals.traffic_light_groups);

  for (const auto & signal_id : received_signal_id_set) {
    const auto perception_result = util::find_signal_by_id(perception_id_signal_map, signal_id);
    const auto external_result = util::find_signal_by_id(external_id_signal_map, signal_id);
    // Neither result exists
    if (!perception_result && !external_result) {
      continue;
    }

    // We don't validate the pedestrian signals
    // TODO(TomohitoAndo): Validate pedestrian signals
    if (isPedestrianSignal(signal_id)) {
      validated_signals.traffic_light_groups.emplace_back(util::get_highest_confidence_signal(
        perception_result, external_result, external_priority_));

      continue;
    }

    // If either of the signal is not received, treat as unknown signal
    if (!perception_result && external_result) {
      const auto unknown_signal = util::create_unknown_signal(*external_result);
      validated_signals.traffic_light_groups.emplace_back(unknown_signal);
      continue;
    }
    if (!external_result && perception_result) {
      const auto unknown_signal = util::create_unknown_signal(*perception_result);
      validated_signals.traffic_light_groups.emplace_back(unknown_signal);
      continue;
    }

    // Check if they have the same elements
    if (!util::are_all_elements_equivalent(
          perception_result->elements, external_result->elements)) {
      const auto unknown_signal = util::create_unknown_signal(*perception_result, *external_result);
      validated_signals.traffic_light_groups.emplace_back(unknown_signal);
      continue;
    }

    // Both results are same, then insert the received color
    validated_signals.traffic_light_groups.emplace_back(*perception_result);
  }

  return validated_signals;
}

void SignalMatchValidator::setPedestrianSignals(
  const std::vector<TrafficLightConstPtr> & pedestrian_signals)
{
  for (const auto & pedestrian_signal : pedestrian_signals) {
    map_pedestrian_signal_regulatory_elements_set_.emplace(pedestrian_signal->id());
  }
}

void SignalMatchValidator::setExternalPriority(const bool external_priority)
{
  external_priority_ = external_priority;
}

bool SignalMatchValidator::isPedestrianSignal(const lanelet::Id & signal_id)
{
  return map_pedestrian_signal_regulatory_elements_set_.find(signal_id) !=
         map_pedestrian_signal_regulatory_elements_set_.end();
}
