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

#ifndef ROUTE_SELECTOR_PANEL_HPP_
#define ROUTE_SELECTOR_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/route_state.hpp>
#include <tier4_planning_msgs/srv/clear_route.hpp>
#include <tier4_planning_msgs/srv/set_waypoint_route.hpp>

namespace rviz_plugins
{

using geometry_msgs::msg::PoseStamped;
using tier4_planning_msgs::msg::RouteState;
using tier4_planning_msgs::srv::ClearRoute;
using tier4_planning_msgs::srv::SetWaypointRoute;

class RouteSelectorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RouteSelectorPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  QLabel * main_state_;
  QLabel * mrm_state_;
  QLabel * planner_state_;
  QPushButton * main_clear_;
  QPushButton * mrm_clear_;

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_main_goal_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_mrm_goal_;
  void onMainGoal(const PoseStamped::ConstSharedPtr msg);
  void onMrmGoal(const PoseStamped::ConstSharedPtr msg);

  rclcpp::Subscription<RouteState>::SharedPtr sub_main_state_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_mrm_state_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_planner_state_;
  void onMainState(RouteState::ConstSharedPtr msg);
  void onMrmState(RouteState::ConstSharedPtr msg);
  void onPlannerState(RouteState::ConstSharedPtr msg);

  rclcpp::Client<ClearRoute>::SharedPtr cli_main_clear_;
  rclcpp::Client<ClearRoute>::SharedPtr cli_mrm_clear_;
  rclcpp::Client<SetWaypointRoute>::SharedPtr cli_main_set_waypoint_;
  rclcpp::Client<SetWaypointRoute>::SharedPtr cli_mrm_set_waypoint_;

private Q_SLOTS:
  void onMainClear();
  void onMrmClear();
};

}  // namespace rviz_plugins

#endif  // ROUTE_SELECTOR_PANEL_HPP_
