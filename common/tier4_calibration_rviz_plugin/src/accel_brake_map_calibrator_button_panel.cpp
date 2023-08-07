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

#include "accel_brake_map_calibrator_button_panel.hpp"

#include "QFileDialog"
#include "QHBoxLayout"
#include "QLineEdit"
#include "QPainter"
#include "QPushButton"
#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace tier4_calibration_rviz_plugin
{
AccelBrakeMapCalibratorButtonPanel::AccelBrakeMapCalibratorButtonPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  topic_label_ = new QLabel("Topic name of update suggest ");
  topic_label_->setAlignment(Qt::AlignCenter);

  topic_edit_ =
    new QLineEdit("/vehicle/calibration/accel_brake_map_calibrator/output/update_suggest");
  connect(topic_edit_, SIGNAL(textEdited(QString)), SLOT(editTopic()));

  calibration_button_ = new QPushButton("Wait for subscribe topic");
  calibration_button_->setEnabled(false);
  connect(calibration_button_, SIGNAL(clicked(bool)), SLOT(pushCalibrationButton()));

  status_label_ = new QLabel("Not Ready");
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("QLabel { background-color : darkgray;}");

  QSizePolicy q_size_policy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  calibration_button_->setSizePolicy(q_size_policy);

  auto * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(topic_label_);
  topic_layout->addWidget(topic_edit_);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(topic_layout);
  v_layout->addWidget(calibration_button_);
  v_layout->addWidget(status_label_);

  setLayout(v_layout);
}

void AccelBrakeMapCalibratorButtonPanel::onInitialize()
{
  rclcpp::Node::SharedPtr raw_node =
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  update_suggest_sub_ = raw_node->create_subscription<std_msgs::msg::Bool>(
    topic_edit_->text().toStdString(), 10,
    std::bind(
      &AccelBrakeMapCalibratorButtonPanel::callbackUpdateSuggest, this, std::placeholders::_1));

  client_ = raw_node->create_client<tier4_vehicle_msgs::srv::UpdateAccelBrakeMap>(
    "/vehicle/calibration/accel_brake_map_calibrator/update_map_dir");
}

void AccelBrakeMapCalibratorButtonPanel::callbackUpdateSuggest(
  const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  if (after_calib_) {
    return;
  }

  if (msg->data) {
    status_label_->setText("Ready");
    status_label_->setStyleSheet("QLabel { background-color : white;}");
  } else {
    status_label_->setText("Ready (not recommended)");
    status_label_->setStyleSheet("QLabel { background-color : darkgray;}");
  }
  calibration_button_->setText("push: start to accel/brake map calibration");
  calibration_button_->setEnabled(true);
}

void AccelBrakeMapCalibratorButtonPanel::editTopic()
{
  update_suggest_sub_.reset();
  rclcpp::Node::SharedPtr raw_node =
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  update_suggest_sub_ = raw_node->create_subscription<std_msgs::msg::Bool>(
    topic_edit_->text().toStdString(), 10,
    std::bind(
      &AccelBrakeMapCalibratorButtonPanel::callbackUpdateSuggest, this, std::placeholders::_1));
  calibration_button_->setText("Wait for subscribe topic");
  calibration_button_->setEnabled(false);
}

void AccelBrakeMapCalibratorButtonPanel::pushCalibrationButton()
{
  // lock button
  calibration_button_->setEnabled(false);

  status_label_->setStyleSheet("QLabel { background-color : blue;}");
  status_label_->setText("executing calibration...");

  std::thread thread([this] {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      status_label_->setStyleSheet("QLabel { background-color : red;}");
      status_label_->setText("service server not found");

    } else {
      auto req = std::make_shared<tier4_vehicle_msgs::srv::UpdateAccelBrakeMap::Request>();
      req->path = "";
      client_->async_send_request(
        req, [this]([[maybe_unused]] rclcpp::Client<
                    tier4_vehicle_msgs::srv::UpdateAccelBrakeMap>::SharedFuture result) {});

      status_label_->setStyleSheet("QLabel { background-color : lightgreen;}");
      status_label_->setText("OK!!!");
    }

    // wait 3 second
    after_calib_ = true;
    rclcpp::Rate(1.0 / 3.0).sleep();
    after_calib_ = false;

    // unlock button
    calibration_button_->setEnabled(true);
  });

  thread.detach();
}

}  // namespace tier4_calibration_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  tier4_calibration_rviz_plugin::AccelBrakeMapCalibratorButtonPanel, rviz_common::Panel)
