// Copyright 2022 TIER IV, Inc.
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
#include "crosswalk_traffic_light_estimator/node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace traffic_light
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

  use_last_detect_color_ = this->declare_parameter("use_last_detect_color", true);

  sub_map_ = create_subscription<HADMapBin>(
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

void CrosswalkTrafficLightEstimatorNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Start loading lanelet");
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
  RCLCPP_INFO(get_logger(), "[CrosswalkTrafficLightEstimatorNode]: Map is loaded");
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

  std::unordered_map<lanelet::Id, TrafficSignal> traffic_light_id_map;
  for (const auto & traffic_signal : msg->signals) {
    traffic_light_id_map[traffic_signal.map_primitive_id] = traffic_signal;
  }

  for (const auto & crosswalk : conflicting_crosswalks_) {
    constexpr int VEHICLE_GRAPH_ID = 0;
    const auto conflict_lls = overall_graphs_ptr_->conflictingInGraph(crosswalk, VEHICLE_GRAPH_ID);
    const auto green_lanelets = getGreenLanelets(conflict_lls, traffic_light_id_map);

    const auto crosswalk_tl_color = estimateCrosswalkTrafficSignal(crosswalk, green_lanelets);
    setCrosswalkTrafficSignal(crosswalk, crosswalk_tl_color, output);
  }

  updateLastDetectedSignal(traffic_light_id_map);

  pub_traffic_light_array_->publish(output);
  pub_processing_time_->publish<Float64Stamped>("processing_time_ms", stop_watch.toc("Total"));

  return;
}

void CrosswalkTrafficLightEstimatorNode::updateLastDetectedSignal(
  const TrafficLightIdMap & traffic_light_id_map)
{
  for (const auto & input_traffic_signal : traffic_light_id_map) {
    const auto & lights = input_traffic_signal.second.lights;

    if (lights.empty()) {
      continue;
    }

    if (lights.front().color == TrafficLight::UNKNOWN) {
      continue;
    }

    const auto & id = input_traffic_signal.second.map_primitive_id;

    if (last_detect_color_.count(id) == 0) {
      last_detect_color_.insert(std::make_pair(id, input_traffic_signal.second));
      continue;
    }

    last_detect_color_.at(id) = input_traffic_signal.second;
  }

  std::vector<int32_t> erase_id_list;
  for (auto & last_traffic_signal : last_detect_color_) {
    const auto & id = last_traffic_signal.second.map_primitive_id;

    if (traffic_light_id_map.count(id) == 0) {
      erase_id_list.emplace_back(id);
    }
  }
  for (const auto id : erase_id_list) last_detect_color_.erase(id);
}

void CrosswalkTrafficLightEstimatorNode::setCrosswalkTrafficSignal(
  const lanelet::ConstLanelet & crosswalk, const uint8_t color, TrafficSignalArray & msg) const
{
  const auto tl_reg_elems = crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();

  for (const auto & tl_reg_elem : tl_reg_elems) {
    const auto crosswalk_traffic_lights = tl_reg_elem->trafficLights();

    for (const auto & traffic_light : crosswalk_traffic_lights) {
      const auto ll_traffic_light = static_cast<lanelet::ConstLineString3d>(traffic_light);

      TrafficSignal output_traffic_signal;
      TrafficLight output_traffic_light;
      output_traffic_light.color = color;
      output_traffic_light.confidence = 1.0;
      output_traffic_signal.lights.push_back(output_traffic_light);
      output_traffic_signal.map_primitive_id = ll_traffic_light.id();
      msg.signals.push_back(output_traffic_signal);
    }
  }
}

lanelet::ConstLanelets CrosswalkTrafficLightEstimatorNode::getGreenLanelets(
  const lanelet::ConstLanelets & lanelets, const TrafficLightIdMap & traffic_light_id_map) const
{
  lanelet::ConstLanelets green_lanelets{};

  for (const auto & lanelet : lanelets) {
    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();

    if (tl_reg_elems.empty()) {
      continue;
    }

    const auto tl_reg_elem = tl_reg_elems.front();
    const auto traffic_lights_for_vehicle = tl_reg_elem->trafficLights();

    const auto current_detected_signal =
      getHighestConfidenceTrafficSignal(traffic_lights_for_vehicle, traffic_light_id_map);

    if (!current_detected_signal) {
      continue;
    }

    const auto is_green = current_detected_signal.get() == TrafficLight::GREEN;

    const auto last_detected_signal =
      getHighestConfidenceTrafficSignal(traffic_lights_for_vehicle, last_detect_color_);

    if (!last_detected_signal) {
      continue;
    }

    const auto was_green = current_detected_signal.get() == TrafficLight::UNKNOWN &&
                           last_detected_signal.get() == TrafficLight::GREEN &&
                           use_last_detect_color_;

    if (!is_green && !was_green) {
      continue;
    }

    green_lanelets.push_back(lanelet);
  }

  return green_lanelets;
}

uint8_t CrosswalkTrafficLightEstimatorNode::estimateCrosswalkTrafficSignal(
  const lanelet::ConstLanelet & crosswalk, const lanelet::ConstLanelets & green_lanelets) const
{
  bool has_left_green_lane = false;
  bool has_right_green_lane = false;
  bool has_straight_green_lane = false;
  bool has_related_green_tl = false;

  const std::string related_tl_id = crosswalk.attributeOr("related_traffic_light", "none");

  for (const auto & lanelet : green_lanelets) {
    const std::string turn_direction = lanelet.attributeOr("turn_direction", "none");

    if (turn_direction == "left") {
      has_left_green_lane = true;
    } else if (turn_direction == "right") {
      has_right_green_lane = true;
    } else {
      has_straight_green_lane = true;
    }

    const auto tl_reg_elems = lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
    if (tl_reg_elems.front()->id() == std::atoi(related_tl_id.c_str())) {
      has_related_green_tl = true;
    }
  }

  if (has_straight_green_lane || has_related_green_tl) {
    return TrafficLight::RED;
  }

  const auto has_merge_lane = hasMergeLane(green_lanelets, routing_graph_ptr_);
  return !has_merge_lane && has_left_green_lane && has_right_green_lane ? TrafficLight::RED
                                                                        : TrafficLight::UNKNOWN;
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

    const auto & lights = traffic_light_id_map.at(id).lights;
    if (lights.empty()) {
      continue;
    }

    const auto & color = lights.front().color;
    const auto & confidence = lights.front().confidence;
    if (confidence < highest_confidence) {
      continue;
    }

    highest_confidence = confidence;
    ret = color;
  }

  return ret;
}
}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::CrosswalkTrafficLightEstimatorNode)
