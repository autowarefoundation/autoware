//
//  Copyright 2022 Tier IV, Inc. All rights reserved.
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
//

#include "bag_time_manager_panel.hpp"

#include <qt5/QtWidgets/QHBoxLayout>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QWidget>
#include <rviz_common/display_context.hpp>

namespace rviz_plugins
{
BagTimeManagerPanel::BagTimeManagerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // pause / resume
  {
    pause_button_ = new QPushButton("Pause");
    pause_button_->setToolTip("Pause/Resume ROS time.");
    pause_button_->setStyleSheet("background-color: #00FF00;");
    pause_button_->setCheckable(true);
  }

  // apply
  {
    apply_rate_button_ = new QPushButton("ApplyRate");
    apply_rate_button_->setToolTip("control ROS time rate.");
  }

  // combo
  {
    rate_label_ = new QLabel("Rate:");
    rate_label_->setAlignment(Qt::AlignCenter);
    rate_combo_ = new QComboBox();
    rate_combo_->addItems({"0.01", "0.1", "0.5", "1.0", "2.0", "5.0", "10.0"});
    rate_combo_->setCurrentText(QString("1.0"));
    time_label_ = new QLabel("X  real time ");
    rate_label_->setAlignment(Qt::AlignCenter);
  }

  auto * layout = new QHBoxLayout();
  layout->addWidget(pause_button_);
  layout->addWidget(apply_rate_button_);
  layout->addWidget(rate_label_);
  layout->addWidget(rate_combo_);
  layout->addWidget(time_label_);
  setLayout(layout);

  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPauseClicked()));
  connect(apply_rate_button_, SIGNAL(clicked()), this, SLOT(onApplyRateClicked()));
  connect(rate_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(onRateChanged()));
}

void BagTimeManagerPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  client_pause_ =
    raw_node_->create_client<Pause>("/rosbag2_player/pause", rmw_qos_profile_services_default);
  client_resume_ =
    raw_node_->create_client<Resume>("/rosbag2_player/resume", rmw_qos_profile_services_default);
  client_set_rate_ =
    raw_node_->create_client<SetRate>("/rosbag2_player/set_rate", rmw_qos_profile_services_default);
}

void BagTimeManagerPanel::onPauseClicked()
{
  if (current_state_ == STATE::PAUSE) {
    // do resume
    current_state_ = STATE::RESUME;
    pause_button_->setText(QString::fromStdString("Resume"));
    // green
    pause_button_->setStyleSheet("background-color: #00FF00;");
    auto req = std::make_shared<Resume::Request>();
    client_resume_->async_send_request(
      req, [this]([[maybe_unused]] rclcpp::Client<Resume>::SharedFuture result) {});
  } else {
    // do pause
    current_state_ = STATE::PAUSE;
    pause_button_->setText(QString::fromStdString("Pause"));
    // red
    pause_button_->setStyleSheet("background-color: #FF0000;");
    auto req = std::make_shared<Pause::Request>();
    client_pause_->async_send_request(
      req, [this]([[maybe_unused]] rclcpp::Client<Pause>::SharedFuture result) {});
  }
}

void BagTimeManagerPanel::onApplyRateClicked()
{
  auto request = std::make_shared<SetRate::Request>();
  request->rate = std::stod(rate_combo_->currentText().toStdString());
  client_set_rate_->async_send_request(
    request, [this, request](rclcpp::Client<SetRate>::SharedFuture result) {
      const auto & response = result.get();
      if (response->success) {
        RCLCPP_INFO(raw_node_->get_logger(), "set ros bag rate %f x real time", request->rate);
      } else {
        RCLCPP_WARN(raw_node_->get_logger(), "service failed");
      }
    });
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::BagTimeManagerPanel, rviz_common::Panel)
