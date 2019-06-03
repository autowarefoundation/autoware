/*
 * Copyright (c) 2018, TierIV Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSignalMapper>
#include <QTimer>
#include <QVBoxLayout>
#include <QMap>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "autoware_msgs/RemoteCmd.h"
#include "decision_maker_panel.h"

namespace autoware_rviz_debug
{
// BEGIN_TUTORIAL
// Here is the implementation of the DecisionMakerPanel class.  DecisionMakerPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.

class StateInfo
{
private:
public:
  uint64_t state_num_;
  uint8_t state_category_;
  std::string state_num_name_;
  std::string state_category_name_;
};

DecisionMakerPanel::DecisionMakerPanel(QWidget* parent) : rviz::Panel(parent)
{
  statecmd_publisher_ = nh_.advertise<std_msgs::String>("/state_cmd", 1);
  emergency_publisher_ = nh_.advertise<autoware_msgs::RemoteCmd>("/remote_cmd", 1);

  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QGridLayout* label_layout = new QGridLayout;
  label_layout->addWidget(new QLabel("DecisionMaker QuickOrder"), 0, 0);

  QSignalMapper* signalMapper = new QSignalMapper(this);
  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(sendTopic(QString)));

  QMap<QString, QPushButton*> buttons_def;
  // main
  buttons_def["engage"] = new QPushButton("\nEngage\n");
  signalMapper->setMapping(buttons_def["engage"], QString("engage"));
  buttons_def["stop"] = new QPushButton("\nStop\n");
  signalMapper->setMapping(buttons_def["stop"], QString("wait"));
  buttons_def["go"] = new QPushButton("\nGo\n");
  signalMapper->setMapping(buttons_def["go"], QString("clear"));
  // execute
  buttons_def["execute_lane_change"] = new QPushButton("Execute \nLaneChange");
  signalMapper->setMapping(buttons_def["execute_lane_change"], QString("execute_lane_change"));
  // mission
  buttons_def["request_mission_change"] = new QPushButton("Request \nmission change");
  signalMapper->setMapping(buttons_def["request_mission_change"], QString("request_mission_change"));
  buttons_def["return_to_driving"] = new QPushButton("Return \nto driving");
  signalMapper->setMapping(buttons_def["return_to_driving"], QString("return_to_driving"));
  buttons_def["mission_canceled"] = new QPushButton("Mission \ncancel");
  signalMapper->setMapping(buttons_def["mission_canceled"], QString("mission_canceled"));
  // emergency
  buttons_def["emergency"] = new QPushButton("Emergency");
  signalMapper->setMapping(buttons_def["emergency"], QString("emergency"));
  buttons_def["return_from_emergency"] = new QPushButton("Return \nfrom emergency");
  signalMapper->setMapping(buttons_def["return_from_emergency"], QString("return_from_emergency"));
  buttons_def["return_to_ready"] = new QPushButton("Return \nto ready");
  signalMapper->setMapping(buttons_def["return_to_ready"], QString("return_to_ready"));
  // set options
  QMap<QString, QPushButton*>::const_iterator i = buttons_def.constBegin();
  QFont* q_font = new QFont("Arial", 11);
  QSizePolicy* q_size_policy = new QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  while (i != buttons_def.constEnd())
  {
    i.value()->setSizePolicy(*q_size_policy);
    i.value()->setFont(*q_font);
    connect(i.value(), SIGNAL(clicked()), signalMapper, SLOT(map()));
    ++i;
  }

  QGridLayout* button_layout_main = new QGridLayout;
  button_layout_main->addWidget(new QLabel("Main"), 0, 0);
  button_layout_main->addWidget(buttons_def["engage"], 1, 0);
  button_layout_main->addWidget(buttons_def["stop"], 1, 1);
  button_layout_main->addWidget(buttons_def["go"], 1, 2);

  QGridLayout* button_layout_execute = new QGridLayout;
  button_layout_execute->addWidget(new QLabel("Execute"), 0, 0); button_layout_execute->addWidget(buttons_def["execute_lane_change"], 1, 0);

  QGridLayout* button_layout_mission = new QGridLayout;
  button_layout_mission->addWidget(new QLabel("Mission"), 0, 0);
  button_layout_mission->addWidget(buttons_def["request_mission_change"], 1, 0);
  button_layout_mission->addWidget(buttons_def["return_to_driving"], 1, 1);
  button_layout_mission->addWidget(buttons_def["mission_canceled"], 1, 2);

  QGridLayout* button_layout_emergency = new QGridLayout;
  button_layout_emergency->addWidget(new QLabel("Emergency"), 0, 0);
  button_layout_emergency->addWidget(buttons_def["emergency"], 1, 0);
  button_layout_emergency->addWidget(buttons_def["return_from_emergency"], 1, 1);
  button_layout_emergency->addWidget(buttons_def["return_to_ready"], 1, 2);


  QMap<QString, QPushButton*> buttons_ext;
  // state vehicle
  buttons_ext["sensor_is_ready"] = new QPushButton("Sensor is ready");
  signalMapper->setMapping(buttons_ext["sensor_is_ready"], QString("sensor_is_ready"));
  buttons_ext["map_is_ready"] = new QPushButton("Map is ready");
  signalMapper->setMapping(buttons_ext["map_is_ready"], QString("map_is_ready"));
  buttons_ext["localization_is_ready"] = new QPushButton("Localization \nis ready");
  signalMapper->setMapping(buttons_ext["localization_is_ready"], QString("localization_is_ready"));
  buttons_ext["planning_is_ready"] = new QPushButton("Planning is ready");
  signalMapper->setMapping(buttons_ext["planning_is_ready"], QString("planning_is_ready"));
  buttons_ext["vehicle_is_ready"] = new QPushButton("Vehicle is ready");
  signalMapper->setMapping(buttons_ext["vehicle_is_ready"], QString("vehicle_is_ready"));
  // state mission
  buttons_ext["received_mission_order"] = new QPushButton("Received mission order");
  signalMapper->setMapping(buttons_ext["received_mission_order"], QString("received_mission_order"));
  buttons_ext["mission_is_compatible"] = new QPushButton("Mission is compatible");
  signalMapper->setMapping(buttons_ext["mission_is_compatible"], QString("mission_is_compatible"));
  buttons_ext["mission_aborted"] = new QPushButton("Mission aborted");
  signalMapper->setMapping(buttons_ext["mission_aborted"], QString("mission_aborted"));
  buttons_ext["arrived_goal"] = new QPushButton("Arrived goal");
  signalMapper->setMapping(buttons_ext["arrived_goal"], QString("arrived_goal"));
  buttons_ext["request_mission_change"] = new QPushButton("Request mission change");
  signalMapper->setMapping(buttons_ext["request_mission_change"], QString("request_mission_change"));
  buttons_ext["mission_is_conflicting"] = new QPushButton("Mission is conflicting");
  signalMapper->setMapping(buttons_ext["mission_is_conflicting"], QString("mission_is_conflicting"));
  buttons_ext["return_to_driving"] = new QPushButton("Return to driving");
  signalMapper->setMapping(buttons_ext["return_to_driving"], QString("return_to_driving"));
  buttons_ext["goto_wait_order"] = new QPushButton("Go to WaitOrder");
  signalMapper->setMapping(buttons_ext["goto_wait_order"], QString("goto_wait_order"));
  // state drive
  buttons_ext["operation_start"] = new QPushButton("Operation start");
  signalMapper->setMapping(buttons_ext["operation_start"], QString("operation_start"));
  buttons_ext["operation_end"] = new QPushButton("Operation end");
  signalMapper->setMapping(buttons_ext["operation_end"], QString("operation_end"));
  buttons_ext["on_lane_area"] = new QPushButton("Lane area");
  signalMapper->setMapping(buttons_ext["on_lane_area"], QString("on_lane_area"));
  buttons_ext["on_free_area"] = new QPushButton("Free area");
  signalMapper->setMapping(buttons_ext["on_free_area"], QString("on_free_area"));
  buttons_ext["on_cruise"] = new QPushButton("Cruise");
  signalMapper->setMapping(buttons_ext["on_cruise"], QString("on_cruise"));
  buttons_ext["on_bus_stop"] = new QPushButton("Bus stop");
  signalMapper->setMapping(buttons_ext["on_bus_stop"], QString("on_bus_stop"));
  buttons_ext["on_parking"] = new QPushButton("Parking");
  signalMapper->setMapping(buttons_ext["on_parking"], QString("on_parking"));
  buttons_ext["on_straight"] = new QPushButton("Straight");
  signalMapper->setMapping(buttons_ext["on_straight"], QString("on_straight"));
  buttons_ext["on_right_turn"] = new QPushButton("Right turn");
  signalMapper->setMapping(buttons_ext["on_right_turn"], QString("on_right_turn"));
  buttons_ext["on_left_turn"] = new QPushButton("Left turn");
  signalMapper->setMapping(buttons_ext["on_left_turn"], QString("on_left_turn"));
  buttons_ext["on_back"] = new QPushButton("Back");
  signalMapper->setMapping(buttons_ext["on_back"], QString("on_back"));
  buttons_ext["lane_change_right"] = new QPushButton("Lanechange Right");
  signalMapper->setMapping(buttons_ext["lane_change_right"], QString("lane_change_right"));
  buttons_ext["lane_change_left"] = new QPushButton("Lanechange Left");
  signalMapper->setMapping(buttons_ext["lane_change_left"], QString("lane_change_left"));
  buttons_ext["on_pull_over"] = new QPushButton("Pull over");
  signalMapper->setMapping(buttons_ext["on_pull_over"], QString("on_pull_over"));
  buttons_ext["on_pull_out"] = new QPushButton("Pull out");
  signalMapper->setMapping(buttons_ext["on_pull_out"], QString("on_pull_out"));
  // set options
  QMap<QString, QPushButton*>::const_iterator j = buttons_ext.constBegin();
  while (j != buttons_ext.constEnd())
  {
    j.value()->setSizePolicy(*q_size_policy);
    j.value()->setFont(*q_font);
    connect(j.value(), SIGNAL(clicked()), signalMapper, SLOT(map()));
    ++j;
  }

  // state vehicle
  QGridLayout* extra_layout_vehicle = new QGridLayout;
  extra_layout_vehicle->addWidget(new QLabel("state Vehicle"), 0, 0);
  extra_layout_vehicle->addWidget(buttons_ext["sensor_is_ready"], 1, 0);
  extra_layout_vehicle->addWidget(buttons_ext["map_is_ready"], 1, 1);
  extra_layout_vehicle->addWidget(buttons_ext["localization_is_ready"], 1, 2);
  extra_layout_vehicle->addWidget(buttons_ext["planning_is_ready"], 2, 0);
  extra_layout_vehicle->addWidget(buttons_ext["vehicle_is_ready"], 2, 1);
  // state mission
  QGridLayout* extra_layout_mission = new QGridLayout;
  extra_layout_mission->addWidget(new QLabel("state Mission"), 0, 0);
  extra_layout_mission->addWidget(buttons_ext["received_mission_order"],1, 0);
  extra_layout_mission->addWidget(buttons_ext["mission_is_compatible"], 1, 1);
  extra_layout_mission->addWidget(buttons_ext["mission_aborted"], 1, 2);
  extra_layout_mission->addWidget(buttons_ext["arrived_goal"], 2, 0);
  extra_layout_mission->addWidget(buttons_ext["request_mission_change"], 2, 1);
  extra_layout_mission->addWidget(buttons_ext["mission_is_conflicting"], 2, 2);
  extra_layout_mission->addWidget(buttons_ext["return_to_driving"], 3, 0);
  extra_layout_mission->addWidget(buttons_ext["goto_wait_order"], 3, 1);
  // state behavior & motion
  QGridLayout* extra_layout_drive = new QGridLayout;
  extra_layout_drive->addWidget(new QLabel("state Behavior & Motion"), 0, 0);
  extra_layout_drive->addWidget(buttons_ext["operation_start"], 1, 0);
  extra_layout_drive->addWidget(buttons_ext["operation_end"], 1, 1);
  extra_layout_drive->addWidget(buttons_ext["on_lane_area"], 2, 0);
  extra_layout_drive->addWidget(buttons_ext["on_free_area"], 2, 1);
  extra_layout_drive->addWidget(buttons_ext["on_cruise"], 3, 0);
  extra_layout_drive->addWidget(buttons_ext["on_bus_stop"], 3, 1);
  extra_layout_drive->addWidget(buttons_ext["on_parking"], 3, 2);
  extra_layout_drive->addWidget(buttons_ext["on_straight"], 4, 0);
  extra_layout_drive->addWidget(buttons_ext["on_right_turn"],4, 1);
  extra_layout_drive->addWidget(buttons_ext["on_left_turn"], 4, 2);
  extra_layout_drive->addWidget(buttons_ext["on_back"], 4, 3);
  extra_layout_drive->addWidget(buttons_ext["lane_change_right"], 5, 0);
  extra_layout_drive->addWidget(buttons_ext["lane_change_left"], 5, 1);
  extra_layout_drive->addWidget(buttons_ext["on_pull_over"], 6, 0);
  extra_layout_drive->addWidget(buttons_ext["on_pull_out"], 6, 1);
  // set layout to window
  window_vehicle = new QWidget;
  window_vehicle->setLayout(extra_layout_vehicle);
  window_vehicle->hide();
  window_mission = new QWidget;
  window_mission->setLayout(extra_layout_mission);
  window_mission->hide();
  window_drive = new QWidget;
  window_drive->setLayout(extra_layout_drive);
  window_drive->hide();


  QPushButton* button_more = new QPushButton("&More...");
  button_more->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  button_more->setFont(*q_font);
  button_more->setCheckable(true);
  connect(button_more, &QAbstractButton::toggled, window_vehicle, &QWidget::setVisible);
  connect(button_more, &QAbstractButton::toggled, window_mission, &QWidget::setVisible);
  connect(button_more, &QAbstractButton::toggled, window_drive, &QWidget::setVisible);

  QHBoxLayout* button_toggle_layout = new QHBoxLayout;
  button_toggle_layout->addWidget(button_more);
  button_toggle_layout->addWidget(new QLabel("(Normally unused commands)"));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(label_layout);
  layout->addLayout(button_layout_main);
  layout->addLayout(button_layout_execute);
  layout->addLayout(button_layout_mission);
  layout->addLayout(button_layout_emergency);
  layout->addLayout(button_toggle_layout);
  layout->addWidget(window_vehicle);
  layout->addWidget(window_mission);
  layout->addWidget(window_drive);
  setLayout(layout);
}

void DecisionMakerPanel::sendTopic(const QString &text)
{
  if (text == "emergency")
  {
    sendEmergency();
  }
  else if (statecmd_publisher_)
  {
    std_msgs::String msg;
    msg.data = text.toStdString();
    statecmd_publisher_.publish(msg);
  }
}

void DecisionMakerPanel::sendEmergency()
{
  if (emergency_publisher_)
  {
    autoware_msgs::RemoteCmd remote_emergency;
    remote_emergency.header.stamp = ros::Time::now();
    remote_emergency.vehicle_cmd.emergency = 1;
    emergency_publisher_.publish(remote_emergency);
  }
}

void DecisionMakerPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void DecisionMakerPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}  // end namespace autoware_rviz_debug

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_debug::DecisionMakerPanel, rviz::Panel)
// END_TUTORIAL
