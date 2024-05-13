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

#include "remaining_distance_time_calculator_node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace autoware::remaining_distance_time_calculator
{

RemainingDistanceTimeCalculatorNode::RemainingDistanceTimeCalculatorNode(
  const rclcpp::NodeOptions & options)
: Node("remaining_distance_time_calculator", options),
  is_graph_ready_{false},
  has_received_route_{false},
  velocity_limit_{99.99},
  remaining_distance_{0.0},
  remaining_time_{0.0}
{
  using std::placeholders::_1;

  sub_odometry_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&RemainingDistanceTimeCalculatorNode::on_odometry, this, _1));

  const auto qos_transient_local = rclcpp::QoS{1}.transient_local();

  sub_map_ = create_subscription<HADMapBin>(
    "~/input/map", qos_transient_local,
    std::bind(&RemainingDistanceTimeCalculatorNode::on_map, this, _1));
  sub_route_ = create_subscription<LaneletRoute>(
    "~/input/route", qos_transient_local,
    std::bind(&RemainingDistanceTimeCalculatorNode::on_route, this, _1));
  sub_planning_velocity_ = create_subscription<tier4_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/current_max_velocity", qos_transient_local,
    std::bind(&RemainingDistanceTimeCalculatorNode::on_velocity_limit, this, _1));

  pub_mission_remaining_distance_time_ = create_publisher<MissionRemainingDistanceTime>(
    "~/output/mission_remaining_distance_time",
    rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable());

  node_param_.update_rate = declare_parameter<double>("update_rate");

  const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&RemainingDistanceTimeCalculatorNode::on_timer, this));
}

void RemainingDistanceTimeCalculatorNode::on_map(const HADMapBin::ConstSharedPtr & msg)
{
  route_handler_.setMap(*msg);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  is_graph_ready_ = true;
}

void RemainingDistanceTimeCalculatorNode::on_odometry(const Odometry::ConstSharedPtr & msg)
{
  current_vehicle_pose_ = msg->pose.pose;
  current_vehicle_velocity_ = msg->twist.twist.linear;
}

void RemainingDistanceTimeCalculatorNode::on_route(const LaneletRoute::ConstSharedPtr & msg)
{
  goal_pose_ = msg->goal_pose;
  has_received_route_ = true;
}

void RemainingDistanceTimeCalculatorNode::on_velocity_limit(
  const VelocityLimit::ConstSharedPtr & msg)
{
  if (msg->max_velocity > 1e-5) {
    velocity_limit_ = msg->max_velocity;
  }
}

void RemainingDistanceTimeCalculatorNode::on_timer()
{
  if (is_graph_ready_ && has_received_route_) {
    calculate_remaining_distance();
    calculate_remaining_time();
    publish_mission_remaining_distance_time();
  }
}

void RemainingDistanceTimeCalculatorNode::calculate_remaining_distance()
{
  size_t index = 0;

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        road_lanelets_, current_vehicle_pose_, &current_lanelet)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find current lanelet.");

    return;
  }

  lanelet::ConstLanelet goal_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, goal_pose_, &goal_lanelet)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find goal lanelet.");

    return;
  }

  const lanelet::Optional<lanelet::routing::Route> optional_route =
    routing_graph_ptr_->getRoute(current_lanelet, goal_lanelet, 0);
  if (!optional_route) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to find proper route.");

    return;
  }

  lanelet::routing::LaneletPath remaining_shortest_path;
  remaining_shortest_path = optional_route->shortestPath();

  remaining_distance_ = 0.0;

  for (auto & llt : remaining_shortest_path) {
    if (remaining_shortest_path.size() == 1) {
      remaining_distance_ +=
        tier4_autoware_utils::calcDistance2d(current_vehicle_pose_.position, goal_pose_.position);
      break;
    }

    if (index == 0) {
      lanelet::ArcCoordinates arc_coord =
        lanelet::utils::getArcCoordinates({llt}, current_vehicle_pose_);
      double this_lanelet_length = lanelet::utils::getLaneletLength2d(llt);

      remaining_distance_ += this_lanelet_length - arc_coord.length;
    } else if (index == (remaining_shortest_path.size() - 1)) {
      lanelet::ArcCoordinates arc_coord = lanelet::utils::getArcCoordinates({llt}, goal_pose_);
      remaining_distance_ += arc_coord.length;
    } else {
      remaining_distance_ += lanelet::utils::getLaneletLength2d(llt);
    }

    index++;
  }
}

void RemainingDistanceTimeCalculatorNode::calculate_remaining_time()
{
  if (velocity_limit_ > 0.0) {
    remaining_time_ = remaining_distance_ / velocity_limit_;
  }
}

void RemainingDistanceTimeCalculatorNode::publish_mission_remaining_distance_time()
{
  MissionRemainingDistanceTime mission_remaining_distance_time;

  mission_remaining_distance_time.remaining_distance = remaining_distance_;
  mission_remaining_distance_time.remaining_time = remaining_time_;
  pub_mission_remaining_distance_time_->publish(mission_remaining_distance_time);
}

}  // namespace autoware::remaining_distance_time_calculator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::remaining_distance_time_calculator::RemainingDistanceTimeCalculatorNode)
