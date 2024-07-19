// Copyright 2022-2023 UCI SORA Lab, TIER IV, Inc.
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
#include "autoware_crosswalk_traffic_light_estimator/node.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::crosswalk_traffic_light_estimator
{
namespace
{

bool hasMergeLane(
  const lanelet::ConstLanelet & lanelet_1, const lanelet::ConstLanelet & lanelet_2,
  const lanelet::routing::RoutingGraphPtr & routing_graph_ptr)
{
  const auto next_lanelets_1 = routing_graph_ptr->following(lanelet_1);
  const auto next_lanelets_2 = routing_graph_ptr->following(lanelet_2);

  for (const auto & next_lanelet_1 : next_lanelets_1) {
    for (const auto & next_lanelet_2 : next_lanelets_2) {
      if (next_lanelet_1.id() == next_lanelet_2.id()) {
        return true;
      }
    }
  }

  return false;
}

bool hasMergeLane(
  const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphPtr & routing_graph_ptr)
{
  for (size_t i = 0; i < lanelets.size(); ++i) {
    for (size_t j = i + 1; j < lanelets.size(); ++j) {
      const auto lanelet_1 = lanelets.at(i);
      const auto lanelet_2 = lanelets.at(j);

      if (lanelet_1.id() == lanelet_2.id()) {
        continue;
      }

      const std::string turn_direction_1 = lanelet_1.attributeOr("turn_direction", "none");
      const std::string turn_direction_2 = lanelet_2.attributeOr("turn_direction", "none");
      if (turn_direction_1 == turn_direction_2) {
        continue;
      }

      if (!hasMergeLane(lanelet_1, lanelet_2, routing_graph_ptr)) {
        continue;
      }

      return true;
    }
  }

  return false;
}
}  // namespace

CrosswalkTrafficLightEstimatorNode::CrosswalkTrafficLightEstimatorNode(
  const rclcpp::NodeOptions & options)
: Node("crosswalk_traffic_light_estimator", options)
{
  using std::placeholders::_1;

  use_last_detect_color_ = declare_parameter<bool>("use_last_detect_color");
  last_detect_color_hold_time_ = declare_parameter<double>("last_detect_color_hold_time");
  last_colors_hold_time_ = declare_parameter<double>("last_colors_hold_time");

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CrosswalkTrafficLightEstimatorNode::onMap, this, _1));
  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&CrosswalkTrafficLightEstimatorNode::onRoute, this, _1));
  sub_traffic_light_array_ = create_subscription<TrafficSignalArray>(
    "~/input/classified/traffic_signals", rclcpp::QoS{1},
    std::bind(&CrosswalkTrafficLightEstimatorNode::onTrafficLightArray, this, _1));

  pub_traffic_light_array_ =
    this->create_publisher<TrafficSignalArray>("~/output/traffic_signals", rclcpp::QoS{1});
  pub_processing_time_ = std::make_shared<DebugPublisher>(this, "~/debug");
}

void CrosswalkTrafficLightEstimatorNode::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  RCLCPP_DEBUG(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Map is loaded");
}

void CrosswalkTrafficLightEstimatorNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  if (lanelet_map_ptr_ == nullptr) {
    RCLCPP_WARN(get_logger(), "cannot set traffic light in route because don't receive map");
    return;
  }

  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : msg->segments) {
    for (const auto & primitive : segment.primitives) {
      try {
        route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(primitive.id));
      } catch (const lanelet::NoSuchPrimitiveError & ex) {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
        return;
      }
    }
  }

  conflicting_crosswalks_.clear();

  for (const auto & route_lanelet : route_lanelets) {
    constexpr int PEDESTRIAN_GRAPH_ID = 1;
    const auto conflict_lls =
      overall_graphs_ptr_->conflictingInGraph(route_lanelet, PEDESTRIAN_GRAPH_ID);
    for (const auto & lanelet : conflict_lls) {
      conflicting_crosswalks_.push_back(lanelet);
    }
  }
}

void CrosswalkTrafficLightEstimatorNode::onTrafficLightArray(
  const TrafficSignalArray::ConstSharedPtr msg)
{
  StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");

  TrafficSignalArray output = *msg;

  TrafficLightIdMap traffic_light_id_map;
  for (const auto & traffic_signal : msg->traffic_light_groups) {
    traffic_light_id_map[traffic_signal.traffic_light_group_id] =
      std::pair<TrafficSignal, rclcpp::Time>(traffic_signal, get_clock()->now());
  }
  for (const auto & crosswalk : conflicting_crosswalks_) {
    constexpr int VEHICLE_GRAPH_ID = 0;
    const auto conflict_lls = overall_graphs_ptr_->conflictingInGraph(crosswalk, VEHICLE_GRAPH_ID);
    const auto non_red_lanelets = getNonRedLanelets(conflict_lls, traffic_light_id_map);

    const auto crosswalk_tl_color = estimateCrosswalkTrafficSignal(crosswalk, non_red_lanelets);
    setCrosswalkTrafficSignal(crosswalk, crosswalk_tl_color, *msg, output);
  }

  removeDuplicateIds(output);

  updateLastDetectedSignal(traffic_light_id_map);
  updateLastDetectedSignals(traffic_light_id_map);

  pub_traffic_light_array_->publish(output);
  pub_processing_time_->publish<Float64Stamped>("processing_time_ms", stop_watch.toc("Total"));

  return;
}

void CrosswalkTrafficLightEstimatorNode::updateLastDetectedSignal(
  const TrafficLightIdMap & traffic_light_id_map)
{
  for (const auto & input_traffic_signal : traffic_light_id_map) {
    const auto & elements = input_traffic_signal.second.first.elements;

    if (elements.empty()) {
      continue;
    }

    if (elements.front().color == TrafficSignalElement::UNKNOWN) {
      continue;
    }

    const auto & id = input_traffic_signal.second.first.traffic_light_group_id;

    if (last_detect_color_.count(id) == 0) {
      last_detect_color_.insert(std::make_pair(id, input_traffic_signal.second));
      continue;
    }

    last_detect_color_.at(id) = input_traffic_signal.second;
  }

  std::vector<int32_t> erase_id_list;
  for (const auto & last_traffic_signal : last_detect_color_) {
    const auto & id = last_traffic_signal.second.first.traffic_light_group_id;

    if (traffic_light_id_map.count(id) == 0) {
      // hold signal recognition results for [last_detect_color_hold_time_] seconds.
      const auto time_from_last_detected =
        (get_clock()->now() - last_traffic_signal.second.second).seconds();
      if (time_from_last_detected > last_detect_color_hold_time_) {
        erase_id_list.emplace_back(id);
      }
    }
  }
  for (const auto id : erase_id_list) {
    last_detect_color_.erase(id);
    is_flashing_.erase(id);
    current_color_state_.erase(id);
  }
}

void CrosswalkTrafficLightEstimatorNode::updateLastDetectedSignals(
  const TrafficLightIdMap & traffic_light_id_map)
{
  for (const auto & input_traffic_signal : traffic_light_id_map) {
    const auto & elements = input_traffic_signal.second.first.elements;

    if (elements.empty()) {
      continue;
    }

    if (
      elements.front().color == TrafficSignalElement::UNKNOWN && elements.front().confidence == 1) {
      continue;
    }

    const auto & id = input_traffic_signal.second.first.traffic_light_group_id;

    if (last_colors_.count(id) == 0) {
      std::vector<TrafficSignalAndTime> signal{input_traffic_signal.second};
      last_colors_.insert(std::make_pair(id, signal));
      continue;
    }

    last_colors_.at(id).push_back(input_traffic_signal.second);
  }

  std::vector<int32_t> erase_id_list;
  for (auto & last_traffic_signal : last_colors_) {
    const auto & id = last_traffic_signal.first;
    for (auto it = last_traffic_signal.second.begin(); it != last_traffic_signal.second.end();) {
      auto sig = (*it).first;
      rclcpp::Time t = (*it).second;

      // hold signal recognition results for [last_colors_hold_time_] seconds.
      const auto time_from_last_detected = (get_clock()->now() - t).seconds();
      if (time_from_last_detected > last_colors_hold_time_) {
        it = last_traffic_signal.second.erase(it);
      } else {
        ++it;
      }
    }
    if (last_traffic_signal.second.empty()) {
      erase_id_list.emplace_back(id);
    }
  }
  for (const auto id : erase_id_list) last_colors_.erase(id);
}

void CrosswalkTrafficLightEstimatorNode::setCrosswalkTrafficSignal(
  const lanelet::ConstLanelet & crosswalk, const uint8_t color, const TrafficSignalArray & msg,
  TrafficSignalArray & output)
{
  const auto tl_reg_elems = crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();

  std::unordered_map<lanelet::Id, size_t> valid_id2idx_map;  // detected traffic light

  for (size_t i = 0; i < msg.traffic_light_groups.size(); ++i) {
    auto signal = msg.traffic_light_groups[i];
    valid_id2idx_map[signal.traffic_light_group_id] = i;
  }

  for (const auto & tl_reg_elem : tl_reg_elems) {
    auto id = tl_reg_elem->id();
    // if valid prediction exists, overwrite the estimation; else, use the estimation
    if (valid_id2idx_map.count(id)) {
      size_t idx = valid_id2idx_map[id];
      auto signal = msg.traffic_light_groups[idx];
      updateFlashingState(signal);  // check if it is flashing
      // update output msg according to flashing and current state
      output.traffic_light_groups[idx].elements[0].color = updateAndGetColorState(signal);
    } else {
      TrafficSignal output_traffic_signal;
      TrafficSignalElement output_traffic_signal_element;
      output_traffic_signal_element.color = color;
      output_traffic_signal_element.shape = TrafficSignalElement::CIRCLE;
      output_traffic_signal_element.confidence = 1.0;
      output_traffic_signal.elements.push_back(output_traffic_signal_element);
      output_traffic_signal.traffic_light_group_id = id;
      output.traffic_light_groups.push_back(output_traffic_signal);
    }
  }
}

void CrosswalkTrafficLightEstimatorNode::updateFlashingState(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;

  // no record of detected color in last_detect_color_hold_time_
  if (is_flashing_.count(id) == 0) {
    is_flashing_.insert(std::make_pair(id, false));
    return;
  }

  // flashing green
  if (
    signal.elements.front().color == TrafficSignalElement::UNKNOWN &&
    signal.elements.front().confidence != 0 &&  // not due to occlusion
    current_color_state_.at(id) != TrafficSignalElement::UNKNOWN) {
    is_flashing_.at(id) = true;
    return;
  }

  // history exists
  if (last_colors_.count(id) > 0) {
    std::vector<TrafficSignalAndTime> history = last_colors_.at(id);
    for (const auto & h : history) {
      if (h.first.elements.front().color != signal.elements.front().color) {
        // keep the current value if not same with input signal
        return;
      }
    }
    // all history is same with input signal
    is_flashing_.at(id) = false;
  }

  // no record of detected color in last_color_hold_time_
  // keep the current value
  return;
}

uint8_t CrosswalkTrafficLightEstimatorNode::updateAndGetColorState(const TrafficSignal & signal)
{
  const auto id = signal.traffic_light_group_id;
  const auto color = signal.elements[0].color;

  if (current_color_state_.count(id) == 0) {
    current_color_state_.insert(std::make_pair(id, color));
  } else if (is_flashing_.at(id) == false) {
    current_color_state_.at(id) = color;
  } else if (is_flashing_.at(id) == true) {
    if (
      current_color_state_.at(id) == TrafficSignalElement::GREEN &&
      color == TrafficSignalElement::RED) {
      current_color_state_.at(id) = TrafficSignalElement::RED;
    } else if (
      current_color_state_.at(id) == TrafficSignalElement::RED &&
      color == TrafficSignalElement::GREEN) {
      current_color_state_.at(id) = TrafficSignalElement::GREEN;
    } else if (current_color_state_.at(id) == TrafficSignalElement::UNKNOWN) {
      if (color == TrafficSignalElement::GREEN || color == TrafficSignalElement::UNKNOWN)
        current_color_state_.at(id) = TrafficSignalElement::GREEN;
      if (color == TrafficSignalElement::RED)
        current_color_state_.at(id) = TrafficSignalElement::RED;
    }
  }

  return current_color_state_.at(id);
}

lanelet::ConstLanelets CrosswalkTrafficLightEstimatorNode::getNonRedLanelets(
  const lanelet::ConstLanelets & lanelets, const TrafficLightIdMap & traffic_light_id_map) const
{
  lanelet::ConstLanelets non_red_lanelets{};

  for (const auto & lanelet : lanelets) {
    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();

    if (tl_reg_elems.empty()) {
      continue;
    }

    const auto tl_reg_elem = tl_reg_elems.front();
    const auto current_detected_signal =
      getHighestConfidenceTrafficSignal(tl_reg_elem->id(), traffic_light_id_map);

    if (!current_detected_signal && !use_last_detect_color_) {
      continue;
    }

    const auto current_is_not_red =
      current_detected_signal ? current_detected_signal.get() == TrafficSignalElement::GREEN ||
                                  current_detected_signal.get() == TrafficSignalElement::AMBER
                              : true;

    const auto current_is_unknown_or_none =
      current_detected_signal ? current_detected_signal.get() == TrafficSignalElement::UNKNOWN
                              : true;

    const auto last_detected_signal =
      getHighestConfidenceTrafficSignal(tl_reg_elem->id(), last_detect_color_);

    if (!last_detected_signal) {
      continue;
    }

    const auto was_not_red = current_is_unknown_or_none &&
                             (last_detected_signal.get() == TrafficSignalElement::GREEN ||
                              last_detected_signal.get() == TrafficSignalElement::AMBER) &&
                             use_last_detect_color_;

    if (!current_is_not_red && !was_not_red) {
      continue;
    }

    non_red_lanelets.push_back(lanelet);
  }

  return non_red_lanelets;
}

uint8_t CrosswalkTrafficLightEstimatorNode::estimateCrosswalkTrafficSignal(
  const lanelet::ConstLanelet & crosswalk, const lanelet::ConstLanelets & non_red_lanelets) const
{
  bool has_left_non_red_lane = false;
  bool has_right_non_red_lane = false;
  bool has_straight_non_red_lane = false;
  bool has_related_non_red_tl = false;

  const std::string related_tl_id = crosswalk.attributeOr("related_traffic_light", "none");

  for (const auto & lanelet : non_red_lanelets) {
    const std::string turn_direction = lanelet.attributeOr("turn_direction", "none");

    if (turn_direction == "left") {
      has_left_non_red_lane = true;
    } else if (turn_direction == "right") {
      has_right_non_red_lane = true;
    } else {
      has_straight_non_red_lane = true;
    }

    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
    if (tl_reg_elems.front()->id() == std::atoi(related_tl_id.c_str())) {
      has_related_non_red_tl = true;
    }
  }

  if (has_straight_non_red_lane || has_related_non_red_tl) {
    return TrafficSignalElement::RED;
  }

  const auto has_merge_lane = hasMergeLane(non_red_lanelets, routing_graph_ptr_);
  return !has_merge_lane && has_left_non_red_lane && has_right_non_red_lane
           ? TrafficSignalElement::RED
           : TrafficSignalElement::UNKNOWN;
}

boost::optional<uint8_t> CrosswalkTrafficLightEstimatorNode::getHighestConfidenceTrafficSignal(
  const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
  const TrafficLightIdMap & traffic_light_id_map) const
{
  boost::optional<uint8_t> ret{boost::none};

  double highest_confidence = 0.0;
  for (const auto & traffic_light : traffic_lights) {
    if (!traffic_light.isLineString()) {
      continue;
    }

    const int id = static_cast<lanelet::ConstLineString3d>(traffic_light).id();
    if (traffic_light_id_map.count(id) == 0) {
      continue;
    }

    const auto & elements = traffic_light_id_map.at(id).first.elements;
    if (elements.empty()) {
      continue;
    }

    const auto & color = elements.front().color;
    const auto & confidence = elements.front().confidence;
    if (confidence < highest_confidence) {
      continue;
    }

    highest_confidence = confidence;
    ret = color;
  }

  return ret;
}

boost::optional<uint8_t> CrosswalkTrafficLightEstimatorNode::getHighestConfidenceTrafficSignal(
  const lanelet::Id & id, const TrafficLightIdMap & traffic_light_id_map) const
{
  boost::optional<uint8_t> ret{boost::none};

  double highest_confidence = 0.0;
  if (traffic_light_id_map.count(id) == 0) {
    return ret;
  }

  for (const auto & element : traffic_light_id_map.at(id).first.elements) {
    if (element.confidence < highest_confidence) {
      continue;
    }

    highest_confidence = element.confidence;
    ret = element.color;
  }

  return ret;
}

void CrosswalkTrafficLightEstimatorNode::removeDuplicateIds(TrafficSignalArray & signal_array) const
{
  auto & signals = signal_array.traffic_light_groups;
  std::sort(signals.begin(), signals.end(), [](const auto & s1, const auto & s2) {
    return s1.traffic_light_group_id < s2.traffic_light_group_id;
  });

  signals.erase(
    std::unique(
      signals.begin(), signals.end(),
      [](const auto & s1, const auto s2) {
        return s1.traffic_light_group_id == s2.traffic_light_group_id;
      }),
    signals.end());
}

}  // namespace autoware::crosswalk_traffic_light_estimator

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode)
