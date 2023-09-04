// Copyright 2023 The Autoware Contributors
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

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/time.hpp>
#include <traffic_light_arbiter/traffic_light_arbiter.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lanelet
{

using TrafficLightConstPtr = std::shared_ptr<const TrafficLight>;

std::vector<TrafficLightConstPtr> filter_traffic_signals(const LaneletMapConstPtr map)
{
  std::vector<TrafficLightConstPtr> signals;
  for (const auto & element : map->regulatoryElementLayer) {
    const auto signal = std::dynamic_pointer_cast<const TrafficLight>(element);
    if (signal) {
      signals.push_back(signal);
    }
  }
  return signals;
}

}  // namespace lanelet

TrafficLightArbiter::TrafficLightArbiter(const rclcpp::NodeOptions & options)
: Node("traffic_light_arbiter", options)
{
  external_time_tolerance_ = this->declare_parameter<double>("external_time_tolerance", 5.0);
  perception_time_tolerance_ = this->declare_parameter<double>("perception_time_tolerance", 1.0);
  external_priority_ = this->declare_parameter<bool>("external_priority", false);

  map_sub_ = create_subscription<LaneletMapBin>(
    "~/sub/vector_map", rclcpp::QoS(1).transient_local(),
    std::bind(&TrafficLightArbiter::onMap, this, std::placeholders::_1));

  perception_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/perception_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::onPerceptionMsg, this, std::placeholders::_1));

  external_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/external_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::onExternalMsg, this, std::placeholders::_1));

  pub_ = create_publisher<TrafficSignalArray>("~/pub/traffic_signals", rclcpp::QoS(1));
}

void TrafficLightArbiter::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  const auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);

  const auto signals = lanelet::filter_traffic_signals(map);
  map_regulatory_elements_set_ = std::make_unique<std::unordered_set<lanelet::Id>>();

  for (const auto & signal : signals) {
    map_regulatory_elements_set_->emplace(signal->id());
  }
}

void TrafficLightArbiter::onPerceptionMsg(const TrafficSignalArray::ConstSharedPtr msg)
{
  latest_perception_msg_ = *msg;

  if (
    (rclcpp::Time(msg->stamp) - rclcpp::Time(latest_external_msg_.stamp)).seconds() >
    external_time_tolerance_) {
    latest_external_msg_.signals.clear();
  }

  arbitrateAndPublish(msg->stamp);
}

void TrafficLightArbiter::onExternalMsg(const TrafficSignalArray::ConstSharedPtr msg)
{
  latest_external_msg_ = *msg;

  if (
    (rclcpp::Time(msg->stamp) - rclcpp::Time(latest_perception_msg_.stamp)).seconds() >
    perception_time_tolerance_) {
    latest_perception_msg_.signals.clear();
  }

  arbitrateAndPublish(msg->stamp);
}

void TrafficLightArbiter::arbitrateAndPublish(const builtin_interfaces::msg::Time & stamp)
{
  using ElementAndPriority = std::pair<Element, bool>;
  std::unordered_map<lanelet::Id, std::vector<ElementAndPriority>> regulatory_element_signals_map;

  if (map_regulatory_elements_set_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received traffic signal messages before a map");
    return;
  }

  TrafficSignalArray output_signals_msg;
  output_signals_msg.stamp = stamp;

  if (map_regulatory_elements_set_->empty()) {
    pub_->publish(output_signals_msg);
    return;
  }

  auto add_signal_function = [&](const auto & signal, bool priority) {
    const auto id = signal.traffic_signal_id;
    if (!map_regulatory_elements_set_->count(id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received a traffic signal not present in the current map (%lu)", id);
      return;
    }

    auto & elements_and_priority = regulatory_element_signals_map[id];
    for (const auto & element : signal.elements) {
      elements_and_priority.emplace_back(element, priority);
    }
  };

  for (const auto & signal : latest_perception_msg_.signals) {
    add_signal_function(signal, false);
  }

  for (const auto & signal : latest_external_msg_.signals) {
    add_signal_function(signal, external_priority_);
  }

  const auto get_highest_confidence_elements =
    [](const std::vector<ElementAndPriority> & elements_and_priority_vector) {
      using Key = Element::_shape_type;
      std::map<Key, ElementAndPriority> highest_score_element_and_priority_map;
      std::vector<Element> highest_score_elements_vector;

      for (const auto & elements_and_priority : elements_and_priority_vector) {
        const auto & element = elements_and_priority.first;
        const auto & element_priority = elements_and_priority.second;
        const auto key = element.shape;
        auto [iter, success] =
          highest_score_element_and_priority_map.try_emplace(key, elements_and_priority);
        const auto & iter_element = iter->second.first;
        const auto & iter_priority = iter->second.second;

        if (
          !success &&
          (iter_element.confidence < element.confidence || iter_priority < element_priority)) {
          iter->second = elements_and_priority;
        }
      }

      for (const auto & [k, v] : highest_score_element_and_priority_map) {
        highest_score_elements_vector.emplace_back(v.first);
      }

      return highest_score_elements_vector;
    };

  output_signals_msg.signals.reserve(regulatory_element_signals_map.size());

  for (const auto & [regulatory_element_id, elements] : regulatory_element_signals_map) {
    TrafficSignal signal_msg;
    signal_msg.traffic_signal_id = regulatory_element_id;
    signal_msg.elements = get_highest_confidence_elements(elements);
    output_signals_msg.signals.emplace_back(signal_msg);
  }

  pub_->publish(output_signals_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightArbiter)
