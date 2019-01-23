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

#ifndef STATE_PANEL_H
#define STATE_PANEL_H

#include <memory>

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#include <QLabel>
#endif

#include <std_msgs/String.h>

namespace autoware_rviz_plugins
{
class StatePanel : public rviz::Panel
{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  StatePanel(QWidget *parent = 0);

  // Then we finish up with protected member variables.
private:
  void processMessage(const std_msgs::String::ConstPtr &msg);

  QLabel *current_state_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  // END_TUTORIAL
};

}  // autoware_rviz_plugins

#endif  // STATE_PANEL_H
