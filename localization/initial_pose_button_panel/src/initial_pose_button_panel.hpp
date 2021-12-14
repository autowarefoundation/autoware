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

#pragma once

#ifndef INITIAL_POSE_BUTTON_PANEL_HPP_
#define INITIAL_POSE_BUTTON_PANEL_HPP_

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>

#include <string>
#ifndef Q_MOC_RUN

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#endif
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>

namespace tier4_localization_rviz_plugin
{
class InitialPoseButtonPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit InitialPoseButtonPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void callbackPoseCov(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

public Q_SLOTS:
  void editTopic();
  void pushInitializeButton();

protected:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;

  rclcpp::Client<tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr client_;

  QLabel * topic_label_;
  QLineEdit * topic_edit_;
  QPushButton * initialize_button_;
  QLabel * status_label_;

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg_;
};

}  // end namespace tier4_localization_rviz_plugin

#endif  // INITIAL_POSE_BUTTON_PANEL_HPP_
