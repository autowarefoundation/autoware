//
// Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#ifndef ACCEL_BRAKE_MAP_CALIBRATOR_BUTTON_PANEL_HPP_
#define ACCEL_BRAKE_MAP_CALIBRATOR_BUTTON_PANEL_HPP_

#include "QLabel"
#include "QLineEdit"
#include "QPushButton"
#include "QSettings"

#include <string>
#ifndef Q_MOC_RUN

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#endif

#include "std_msgs/msg/bool.hpp"
#include "tier4_vehicle_msgs/srv/update_accel_brake_map.hpp"

namespace tier4_calibration_rviz_plugin
{
class AccelBrakeMapCalibratorButtonPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AccelBrakeMapCalibratorButtonPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void callbackUpdateSuggest(const std_msgs::msg::Bool::ConstSharedPtr msg);

public Q_SLOTS:  // NOLINT for Qt
  void editTopic();
  void pushCalibrationButton();

protected:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr update_suggest_sub_;
  rclcpp::Client<tier4_vehicle_msgs::srv::UpdateAccelBrakeMap>::SharedPtr client_;

  bool after_calib_ = false;

  QLabel * topic_label_;
  QLineEdit * topic_edit_;
  QPushButton * calibration_button_;
  QLabel * status_label_;
};

}  // end namespace tier4_calibration_rviz_plugin

#endif  // ACCEL_BRAKE_MAP_CALIBRATOR_BUTTON_PANEL_HPP_
