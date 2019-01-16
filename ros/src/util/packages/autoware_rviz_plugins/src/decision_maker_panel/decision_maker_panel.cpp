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
  QHBoxLayout* label_layout = new QHBoxLayout;
  label_layout->addWidget(new QLabel("DecisionMaker QuickOrder"));

  QSignalMapper* signalMapper = new QSignalMapper(this);
  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(sendTopic(QString)));

  QHBoxLayout* button_layout = new QHBoxLayout;
  QPushButton* button_engage = new QPushButton("Engage");
  signalMapper->setMapping(button_engage, QString("engage"));
  connect(button_engage, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_stop = new QPushButton("Stop");
  signalMapper->setMapping(button_stop, QString("wait"));
  connect(button_stop, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_go = new QPushButton("Go");
  signalMapper->setMapping(button_go, QString("clear"));
  connect(button_go, SIGNAL(clicked()), signalMapper, SLOT(map()));

  button_layout->addWidget(button_engage);
  button_layout->addWidget(button_stop);
  button_layout->addWidget(button_go);

  QHBoxLayout* button_second_layout = new QHBoxLayout;
  QPushButton* button_execute_avoidance = new QPushButton("Execute Avoidance");
  signalMapper->setMapping(button_execute_avoidance, QString("execute_avoidance"));
  connect(button_execute_avoidance, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_execute_lane_change = new QPushButton("Execute LaneChange");
  signalMapper->setMapping(button_execute_lane_change, QString("execute_lane_change"));
  connect(button_execute_lane_change, SIGNAL(clicked()), signalMapper, SLOT(map()));

  button_second_layout->addWidget(button_execute_avoidance);
  button_second_layout->addWidget(button_execute_lane_change);

  QHBoxLayout* button_middle_layout = new QHBoxLayout;
  QPushButton* button_request_mission_change = new QPushButton("Request mission change");
  signalMapper->setMapping(button_request_mission_change, QString("request_mission_change"));
  connect(button_request_mission_change, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_return_to_driving = new QPushButton("Return to driving");
  signalMapper->setMapping(button_return_to_driving, QString("return_to_driving"));
  connect(button_return_to_driving, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_mission_canceled = new QPushButton("Mission cancel");
  signalMapper->setMapping(button_mission_canceled, QString("mission_canceled"));
  connect(button_mission_canceled, SIGNAL(clicked()), signalMapper, SLOT(map()));

  button_middle_layout->addWidget(button_request_mission_change);
  button_middle_layout->addWidget(button_return_to_driving);
  button_middle_layout->addWidget(button_mission_canceled);

  QHBoxLayout* button_low_layout = new QHBoxLayout;
  QPushButton* button_emergency = new QPushButton("Emergency");
  signalMapper->setMapping(button_emergency, QString("emergency"));
  connect(button_emergency, SIGNAL(clicked()), signalMapper, SLOT(map()));

  QPushButton* button_return_from_emergency = new QPushButton("Return from emergency");
  signalMapper->setMapping(button_return_from_emergency, QString("return_from_emergency"));
  connect(button_return_from_emergency, SIGNAL(clicked()), signalMapper, SLOT(map()));

  button_low_layout->addWidget(button_emergency);
  button_low_layout->addWidget(button_return_from_emergency);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(label_layout);
  layout->addLayout(button_layout);
  layout->addLayout(button_second_layout);
  layout->addLayout(button_middle_layout);
  layout->addLayout(button_low_layout);
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
