// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__SCENARIO_SELECTOR__NODE_HPP_
#define AUTOWARE__SCENARIO_SELECTOR__NODE_HPP_

#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>

namespace autoware::scenario_selector
{
class ScenarioSelectorNode : public rclcpp::Node
{
public:
  explicit ScenarioSelectorNode(const rclcpp::NodeOptions & node_options);

  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  bool isDataReady();
  void onTimer();
  void onMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void onRoute(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void onLaneDrivingTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onParkingTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void publishTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  void updateCurrentScenario();
  std::string selectScenarioByPosition();
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr getScenarioTrajectory(
    const std::string & scenario);

  void updateData();

private:
  bool isAutonomous() const;
  bool isEmptyParkingTrajectory() const;
  bool isSwitchToLaneDriving();
  bool isSwitchToParking(const bool is_stopped);

  inline bool isCurrentLaneDriving() const
  {
    return current_scenario_ == tier4_planning_msgs::msg::Scenario::LANEDRIVING;
  }

  inline bool isCurrentParking() const
  {
    return current_scenario_ == tier4_planning_msgs::msg::Scenario::PARKING;
  }

  rclcpp::TimerBase::SharedPtr timer_;

  // subscribers
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_lanelet_map_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_lane_driving_trajectory_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_parking_trajectory_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr pub_scenario_;

  // polling subscribers
  universe_utils::InterProcessPollingSubscriber<
    nav_msgs::msg::Odometry, autoware::universe_utils::polling_policy::All>::SharedPtr sub_odom_;
  universe_utils::InterProcessPollingSubscriber<std_msgs::msg::Bool>::SharedPtr sub_parking_state_;
  universe_utils::InterProcessPollingSubscriber<
    autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_operation_mode_state_;

  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_state_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr lane_driving_trajectory_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr parking_trajectory_;
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  std::string current_scenario_;
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer_;

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  // Parameters
  double update_rate_;
  double th_max_message_delay_sec_;
  double th_arrived_distance_m_;
  double th_stopped_time_sec_;
  double th_stopped_velocity_mps_;
  bool enable_mode_switching_;
  bool is_parking_completed_;

  boost::optional<rclcpp::Time> lane_driving_stop_time_;
  boost::optional<rclcpp::Time> empty_parking_trajectory_time_;

  static constexpr double lane_stopping_timeout_s = 5.0;
  static constexpr double empty_parking_trajectory_timeout_s = 3.0;
};
}  // namespace autoware::scenario_selector
#endif  // AUTOWARE__SCENARIO_SELECTOR__NODE_HPP_
