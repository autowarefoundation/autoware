/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <QPainter>
#include <QVBoxLayout>

#include "state_panel.h"

namespace autoware_rviz_plugins
{
StatePanel::StatePanel(QWidget *parent) : rviz::Panel(parent)
{
  sub_ = nh_.subscribe("state",10, &StatePanel::processMessage, this);

  current_state_ = new QLabel;

  QVBoxLayout *layout = new QVBoxLayout;
  current_state_->setText("INITIAL_STATE");
  layout->addWidget(current_state_);
  setLayout( layout );
}

void StatePanel::processMessage(const std_msgs::String::ConstPtr &msg)
{
  QString qstr = QString::fromStdString(msg->data);
  current_state_->setText(qstr);
}

}  // autoware_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::StatePanel, rviz::Panel)