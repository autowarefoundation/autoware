//  Copyright 2023 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "route_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace tier4_adapi_rviz_plugins
{

RoutePanel::RoutePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  waypoints_mode_ = new QPushButton("mode");
  waypoints_reset_ = new QPushButton("reset");
  waypoints_apply_ = new QPushButton("apply");
  adapi_clear_ = new QPushButton("clear");
  adapi_set_ = new QPushButton("set");
  adapi_change_ = new QPushButton("change");
  adapi_response_ = new QLabel("the response will be displayed here");
  adapi_auto_clear_ = new QCheckBox("auto clear");
  allow_goal_modification_ = new QCheckBox("allow goal modification");

  waypoints_mode_->setCheckable(true);
  waypoints_reset_->setDisabled(true);
  waypoints_apply_->setDisabled(true);
  connect(waypoints_mode_, &QPushButton::clicked, this, &RoutePanel::onWaypointsMode);
  connect(waypoints_reset_, &QPushButton::clicked, this, &RoutePanel::onWaypointsReset);
  connect(waypoints_apply_, &QPushButton::clicked, this, &RoutePanel::onWaypointsApply);

  const auto buttons = new QButtonGroup(this);
  buttons->addButton(adapi_set_);
  buttons->addButton(adapi_change_);
  buttons->setExclusive(true);
  adapi_set_->setCheckable(true);
  adapi_change_->setCheckable(true);
  adapi_response_->setAlignment(Qt::AlignCenter);

  connect(adapi_clear_, &QPushButton::clicked, this, &RoutePanel::clearRoute);
  connect(adapi_set_, &QPushButton::clicked, [this] { adapi_mode_ = AdapiMode::Set; });
  connect(adapi_change_, &QPushButton::clicked, [this] { adapi_mode_ = AdapiMode::Change; });

  adapi_auto_clear_->setChecked(true);
  adapi_set_->setChecked(true);
  adapi_mode_ = AdapiMode::Set;

  const auto layout = new QVBoxLayout();
  setLayout(layout);

  // waypoints group
  {
    const auto group = new QGroupBox("waypoints");
    const auto local = new QHBoxLayout();
    local->addWidget(waypoints_mode_);
    local->addWidget(waypoints_reset_);
    local->addWidget(waypoints_apply_);
    group->setLayout(local);
    layout->addWidget(group);
    waypoints_group_ = group;
  }

  // adapi group
  {
    const auto group = new QGroupBox("adapi");
    const auto local = new QGridLayout();
    local->addWidget(adapi_clear_, 0, 0);
    local->addWidget(adapi_set_, 0, 1);
    local->addWidget(adapi_change_, 0, 2);
    local->addWidget(adapi_auto_clear_, 1, 0);
    local->addWidget(adapi_response_, 1, 1, 1, 2);
    group->setLayout(local);
    layout->addWidget(group);
  }

  // options group
  {
    const auto group = new QGroupBox("options");
    const auto local = new QHBoxLayout();
    local->addWidget(allow_goal_modification_);
    group->setLayout(local);
    layout->addWidget(group);
  }
}

void RoutePanel::onInitialize()
{
  auto lock = getDisplayContext()->getRosNodeAbstraction().lock();
  auto node = lock->get_raw_node();

  sub_pose_ = node->create_subscription<PoseStamped>(
    "/rviz/routing/pose", rclcpp::QoS(1),
    std::bind(&RoutePanel::onPose, this, std::placeholders::_1));

  const auto adaptor = component_interface_utils::NodeAdaptor(node.get());
  adaptor.init_cli(cli_clear_);
  adaptor.init_cli(cli_set_);
  adaptor.init_cli(cli_change_);
}

void RoutePanel::onPose(const PoseStamped::ConstSharedPtr msg)
{
  if (!waypoints_mode_->isChecked()) {
    requestRoute(*msg);
  } else {
    waypoints_.push_back(*msg);
    waypoints_group_->setTitle(QString("waypoints (count: %1)").arg(waypoints_.size()));
  }
}

void RoutePanel::onWaypointsMode(bool clicked)
{
  waypoints_reset_->setEnabled(clicked);
  waypoints_apply_->setEnabled(clicked);

  if (clicked) {
    onWaypointsReset();
  } else {
    waypoints_group_->setTitle("waypoints");
  }
}

void RoutePanel::onWaypointsReset()
{
  waypoints_.clear();
  waypoints_group_->setTitle(QString("waypoints (count: %1)").arg(waypoints_.size()));
}

void RoutePanel::onWaypointsApply()
{
  if (waypoints_.empty()) return;

  const auto call = [this] {
    const auto req = std::make_shared<SetRoutePoints::Service::Request>();
    req->header = waypoints_.back().header;
    req->goal = waypoints_.back().pose;
    for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
      req->waypoints.push_back(waypoints_[i].pose);
    }
    req->option.allow_goal_modification = allow_goal_modification_->isChecked();
    asyncSendRequest(req);
    onWaypointsReset();
  };

  if (adapi_mode_ == AdapiMode::Set && adapi_auto_clear_->isChecked()) {
    const auto req = std::make_shared<ClearRoute::Service::Request>();
    cli_clear_->async_send_request(req, [call](auto) { call(); });
  } else {
    call();
  }
}

void RoutePanel::requestRoute(const PoseStamped & pose)
{
  const auto call = [this, pose] {
    const auto req = std::make_shared<SetRoutePoints::Service::Request>();
    req->header = pose.header;
    req->goal = pose.pose;
    req->option.allow_goal_modification = allow_goal_modification_->isChecked();
    asyncSendRequest(req);
  };

  if (adapi_mode_ == AdapiMode::Set && adapi_auto_clear_->isChecked()) {
    const auto req = std::make_shared<ClearRoute::Service::Request>();
    cli_clear_->async_send_request(req, [call](auto) { call(); });
  } else {
    call();
  }
}

void RoutePanel::clearRoute()
{
  const auto callback = [this](auto future) {
    const auto status = future.get()->status;
    std::string text = status.success ? "OK" : "Error";
    text += "[" + std::to_string(status.code) + "]";
    text += " " + status.message;
    adapi_response_->setText(QString::fromStdString(text));
  };

  const auto req = std::make_shared<ClearRoute::Service::Request>();
  cli_clear_->async_send_request(req, callback);
}

void RoutePanel::asyncSendRequest(SetRoutePoints::Service::Request::SharedPtr req)
{
  const auto callback = [this](auto future) {
    const auto status = future.get()->status;
    std::string text = status.success ? "OK" : "Error";
    text += "[" + std::to_string(status.code) + "]";
    text += " " + status.message;
    adapi_response_->setText(QString::fromStdString(text));
  };

  if (adapi_mode_ == AdapiMode::Set) {
    cli_set_->async_send_request(req, callback);
  }
  if (adapi_mode_ == AdapiMode::Change) {
    cli_change_->async_send_request(req, callback);
  }
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::RoutePanel, rviz_common::Panel)
