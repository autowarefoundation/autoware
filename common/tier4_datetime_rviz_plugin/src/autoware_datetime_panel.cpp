// Copyright 2021 Tier IV, Inc.
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

#include "autoware_datetime_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>

#include <ctime>

void set_format_time(QLineEdit * line, double time)
{
  char buffer[128];
  auto seconds = static_cast<time_t>(time);
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&seconds));
  line->setText(QString(buffer) + QString::number((time - seconds), 'f', 3).rightRef(4));
}

AutowareDateTimePanel::AutowareDateTimePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  ros_time_label_ = new QLineEdit;
  ros_time_label_->setReadOnly(true);

  wall_time_label_ = new QLineEdit;
  wall_time_label_->setReadOnly(true);

  auto * layout = new QHBoxLayout(this);
  layout->addWidget(new QLabel("ROS Time:"));
  layout->addWidget(ros_time_label_);
  layout->addWidget(new QLabel("Wall Time:"));
  layout->addWidget(wall_time_label_);
  setLayout(layout);

  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &AutowareDateTimePanel::update);
  timer->start(60);
}

void AutowareDateTimePanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
}

void AutowareDateTimePanel::update()
{
  set_format_time(
    ros_time_label_, rviz_ros_node_.lock()->get_raw_node()->get_clock()->now().seconds());
  set_format_time(wall_time_label_, rclcpp::Clock().now().seconds());
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AutowareDateTimePanel, rviz_common::Panel)
