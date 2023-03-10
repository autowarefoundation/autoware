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

#include "traffic_light_selector.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <map>
#include <memory>
#include <tuple>
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

TrafficLightSelector::TrafficLightSelector(const rclcpp::NodeOptions & options)
: Node("traffic_light_selector", options)
{
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/sub/vector_map", rclcpp::QoS(1).transient_local(),
    std::bind(&TrafficLightSelector::on_map, this, std::placeholders::_1));

  sub_tlr_ = create_subscription<TrafficLightArray>(
    "~/sub/traffic_lights", rclcpp::QoS(1),
    std::bind(&TrafficLightSelector::on_msg, this, std::placeholders::_1));

  pub_ = create_publisher<TrafficSignalArray>("~/pub/traffic_signals", rclcpp::QoS(1));
}

void TrafficLightSelector::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  const auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);

  const auto signals = lanelet::filter_traffic_signals(map);
  mapping_.clear();
  for (const auto & signal : signals) {
    for (const auto & light : signal->trafficLights()) {
      mapping_[light.id()] = signal->id();
    }
  }
}

void TrafficLightSelector::on_msg(const TrafficLightArray::ConstSharedPtr msg)
{
  using TrafficSignal = autoware_perception_msgs::msg::TrafficSignal;
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  std::unordered_map<lanelet::Id, std::vector<Element>> intersections;

  // Use the most confident traffic light element in the same state.
  const auto get_highest_confidence_elements = [](const std::vector<Element> & elements) {
    using Key = std::tuple<Element::_color_type, Element::_shape_type, Element::_status_type>;
    std::map<Key, Element> highest_;

    for (const auto & element : elements) {
      const auto key = std::make_tuple(element.color, element.shape, element.status);
      auto [iter, success] = highest_.try_emplace(key, element);
      if (!success && iter->second.confidence < element.confidence) {
        iter->second = element;
      }
    }

    std::vector<Element> result;
    result.reserve(highest_.size());
    for (const auto & [k, v] : highest_) {
      result.push_back(v);
    }
    return result;
  };

  // Wait for vector map to create id mapping.
  if (mapping_.empty()) {
    return;
  }

  // Merge traffic lights in the same group.
  for (const auto & light : msg->lights) {
    const auto id = light.traffic_light_id;
    if (!mapping_.count(id)) {
      continue;
    }
    auto & elements = intersections[mapping_[id]];
    for (const auto & element : light.elements) {
      elements.push_back(element);
    }
  }

  TrafficSignalArray array;
  array.stamp = msg->stamp;
  for (const auto & [id, elements] : intersections) {
    TrafficSignal signal;
    signal.traffic_signal_id = id;
    signal.elements = get_highest_confidence_elements(elements);
    array.signals.push_back(signal);
  }
  pub_->publish(array);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightSelector)
