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

#ifndef AUTOWARE_CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR__NODE_HPP_
#define AUTOWARE_CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR__NODE_HPP_

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
namespace autoware::crosswalk_traffic_light_estimator
{

using autoware::universe_utils::DebugPublisher;
using autoware::universe_utils::StopWatch;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using tier4_debug_msgs::msg::Float64Stamped;
using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficSignalElement = autoware_perception_msgs::msg::TrafficLightElement;
using TrafficSignalAndTime = std::pair<TrafficSignal, rclcpp::Time>;
using TrafficLightIdMap = std::unordered_map<lanelet::Id, TrafficSignalAndTime>;

using TrafficLightIdArray = std::unordered_map<lanelet::Id, std::vector<TrafficSignalAndTime>>;

class CrosswalkTrafficLightEstimatorNode : public rclcpp::Node
{
public:
  explicit CrosswalkTrafficLightEstimatorNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr sub_traffic_light_array_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_traffic_light_array_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;

  lanelet::ConstLanelets conflicting_crosswalks_;

  void onMap(const LaneletMapBin::ConstSharedPtr msg);
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onTrafficLightArray(const TrafficSignalArray::ConstSharedPtr msg);

  void updateLastDetectedSignal(const TrafficLightIdMap & traffic_signals);
  void updateLastDetectedSignals(const TrafficLightIdMap & traffic_signals);
  void updateFlashingState(const TrafficSignal & signal);
  uint8_t updateAndGetColorState(const TrafficSignal & signal);
  void setCrosswalkTrafficSignal(
    const lanelet::ConstLanelet & crosswalk, const uint8_t color, const TrafficSignalArray & msg,
    TrafficSignalArray & output);

  lanelet::ConstLanelets getNonRedLanelets(
    const lanelet::ConstLanelets & lanelets, const TrafficLightIdMap & traffic_light_id_map) const;

  uint8_t estimateCrosswalkTrafficSignal(
    const lanelet::ConstLanelet & crosswalk, const lanelet::ConstLanelets & non_red_lanelets) const;

  boost::optional<uint8_t> getHighestConfidenceTrafficSignal(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    const TrafficLightIdMap & traffic_light_id_map) const;

  boost::optional<uint8_t> getHighestConfidenceTrafficSignal(
    const lanelet::Id & id, const TrafficLightIdMap & traffic_light_id_map) const;

  void removeDuplicateIds(TrafficSignalArray & signal_array) const;

  // Node param
  bool use_last_detect_color_;
  double last_detect_color_hold_time_;
  double last_colors_hold_time_;

  // Signal history
  TrafficLightIdMap last_detect_color_;
  TrafficLightIdArray last_colors_;

  // State
  std::map<lanelet::Id, bool> is_flashing_;
  std::map<lanelet::Id, uint8_t> current_color_state_;

  // Stop watch
  StopWatch<std::chrono::milliseconds> stop_watch_;

  // Debug
  std::shared_ptr<DebugPublisher> pub_processing_time_;
};

}  // namespace autoware::crosswalk_traffic_light_estimator

#endif  // AUTOWARE_CROSSWALK_TRAFFIC_LIGHT_ESTIMATOR__NODE_HPP_
