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

#include "route_selector_panel.hpp"

#include <QGridLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace rviz_plugins
{

QString to_string_state(const RouteState & state)
{
  // clang-format off
  switch (state.state) {
    case RouteState::UNKNOWN:      return "unknown";
    case RouteState::INITIALIZING: return "initializing";
    case RouteState::UNSET:        return "unset";
    case RouteState::ROUTING:      return "routing";
    case RouteState::SET:          return "set";
    case RouteState::REROUTING:    return "rerouting";
    case RouteState::ARRIVED:      return "arrived";
    case RouteState::ABORTED:      return "aborted";
    case RouteState::INTERRUPTED:  return "interrupted";
    default:                       return "-----";
  }
  // clang-format on
}

RouteSelectorPanel::RouteSelectorPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  main_state_ = new QLabel("unknown");
  main_clear_ = new QPushButton("clear");
  mrm_state_ = new QLabel("unknown");
  mrm_clear_ = new QPushButton("clear");
  planner_state_ = new QLabel("unknown");

  connect(main_clear_, &QPushButton::clicked, this, &RouteSelectorPanel::onMainClear);
  connect(mrm_clear_, &QPushButton::clicked, this, &RouteSelectorPanel::onMrmClear);

  const auto layout = new QGridLayout();
  setLayout(layout);
  layout->addWidget(new QLabel("main"), 0, 0);
  layout->addWidget(main_state_, 0, 1);
  layout->addWidget(main_clear_, 0, 2);
  layout->addWidget(new QLabel("mrm"), 1, 0);
  layout->addWidget(mrm_state_, 1, 1);
  layout->addWidget(mrm_clear_, 1, 2);
  layout->addWidget(new QLabel("planner"), 2, 0);
  layout->addWidget(planner_state_, 2, 1);
}

void RouteSelectorPanel::onInitialize()
{
  auto lock = getDisplayContext()->getRosNodeAbstraction().lock();
  auto node = lock->get_raw_node();

  const auto durable_qos = rclcpp::QoS(1).transient_local();

  sub_main_goal_ = node->create_subscription<PoseStamped>(
    "/rviz/route_selector/main/goal", rclcpp::QoS(1),
    std::bind(&RouteSelectorPanel::onMainGoal, this, std::placeholders::_1));
  sub_mrm_goal_ = node->create_subscription<PoseStamped>(
    "/rviz/route_selector/mrm/goal", rclcpp::QoS(1),
    std::bind(&RouteSelectorPanel::onMrmGoal, this, std::placeholders::_1));
  sub_main_state_ = node->create_subscription<RouteState>(
    "/planning/mission_planning/route_selector/main/state", durable_qos,
    std::bind(&RouteSelectorPanel::onMainState, this, std::placeholders::_1));
  sub_mrm_state_ = node->create_subscription<RouteState>(
    "/planning/mission_planning/route_selector/mrm/state", durable_qos,
    std::bind(&RouteSelectorPanel::onMrmState, this, std::placeholders::_1));
  sub_planner_state_ = node->create_subscription<RouteState>(
    "/planning/mission_planning/state", durable_qos,
    std::bind(&RouteSelectorPanel::onPlannerState, this, std::placeholders::_1));

  cli_main_clear_ =
    node->create_client<ClearRoute>("/planning/mission_planning/route_selector/main/clear_route");
  cli_mrm_clear_ =
    node->create_client<ClearRoute>("/planning/mission_planning/route_selector/mrm/clear_route");
  cli_main_set_waypoint_ = node->create_client<SetWaypointRoute>(
    "/planning/mission_planning/route_selector/main/set_waypoint_route");
  cli_mrm_set_waypoint_ = node->create_client<SetWaypointRoute>(
    "/planning/mission_planning/route_selector/mrm/set_waypoint_route");
}

void RouteSelectorPanel::onMainGoal(const PoseStamped::ConstSharedPtr msg)
{
  const auto req = std::make_shared<SetWaypointRoute::Request>();
  req->header = msg->header;
  req->goal_pose = msg->pose;
  req->allow_modification = true;
  cli_main_set_waypoint_->async_send_request(req);
}

void RouteSelectorPanel::onMrmGoal(const PoseStamped::ConstSharedPtr msg)
{
  const auto req = std::make_shared<SetWaypointRoute::Request>();
  req->header = msg->header;
  req->goal_pose = msg->pose;
  req->allow_modification = true;
  cli_mrm_set_waypoint_->async_send_request(req);
}

void RouteSelectorPanel::onMainState(RouteState::ConstSharedPtr msg)
{
  main_state_->setText(to_string_state(*msg));
}

void RouteSelectorPanel::onMrmState(RouteState::ConstSharedPtr msg)
{
  mrm_state_->setText(to_string_state(*msg));
}

void RouteSelectorPanel::onPlannerState(RouteState::ConstSharedPtr msg)
{
  planner_state_->setText(to_string_state(*msg));
}

void RouteSelectorPanel::onMainClear()
{
  const auto req = std::make_shared<ClearRoute::Request>();
  cli_main_clear_->async_send_request(req);
}

void RouteSelectorPanel::onMrmClear()
{
  const auto req = std::make_shared<ClearRoute::Request>();
  cli_mrm_clear_->async_send_request(req);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::RouteSelectorPanel, rviz_common::Panel)
