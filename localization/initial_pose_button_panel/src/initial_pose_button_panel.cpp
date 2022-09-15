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

#include "initial_pose_button_panel.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace tier4_localization_rviz_plugin
{
InitialPoseButtonPanel::InitialPoseButtonPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  topic_label_ = new QLabel("PoseWithCovarianceStamped ");
  topic_label_->setAlignment(Qt::AlignCenter);

  topic_edit_ = new QLineEdit("/sensing/gnss/pose_with_covariance");
  connect(topic_edit_, SIGNAL(textEdited(QString)), SLOT(editTopic()));

  initialize_button_ = new QPushButton("Wait for subscribe topic");
  initialize_button_->setEnabled(false);
  connect(initialize_button_, SIGNAL(clicked(bool)), SLOT(pushInitializeButton()));

  status_label_ = new QLabel("Not Initialize");
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("QLabel { background-color : gray;}");

  QSizePolicy q_size_policy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  initialize_button_->setSizePolicy(q_size_policy);

  auto * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(topic_label_);
  topic_layout->addWidget(topic_edit_);

  auto * v_layout = new QVBoxLayout;
  v_layout->addLayout(topic_layout);
  v_layout->addWidget(initialize_button_);
  v_layout->addWidget(status_label_);

  setLayout(v_layout);
}
void InitialPoseButtonPanel::onInitialize()
{
  rclcpp::Node::SharedPtr raw_node =
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pose_cov_sub_ = raw_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_edit_->text().toStdString(), 10,
    std::bind(&InitialPoseButtonPanel::callbackPoseCov, this, std::placeholders::_1));

  client_ = raw_node->create_client<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "/localization/initialize");
}

void InitialPoseButtonPanel::callbackPoseCov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  pose_cov_msg_ = *msg;
  initialize_button_->setText("Pose Initializer Let's GO!");
  initialize_button_->setEnabled(true);
}

void InitialPoseButtonPanel::editTopic()
{
  pose_cov_sub_.reset();
  rclcpp::Node::SharedPtr raw_node =
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pose_cov_sub_ = raw_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_edit_->text().toStdString(), 10,
    std::bind(&InitialPoseButtonPanel::callbackPoseCov, this, std::placeholders::_1));
  initialize_button_->setText("Wait for subscribe topic");
  initialize_button_->setEnabled(false);
}

void InitialPoseButtonPanel::pushInitializeButton()
{
  // lock button
  initialize_button_->setEnabled(false);

  status_label_->setStyleSheet("QLabel { background-color : dodgerblue;}");
  status_label_->setText("Initializing...");

  std::thread thread([this] {
    auto req = std::make_shared<tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request>();
    req->pose_with_covariance = pose_cov_msg_;

    client_->async_send_request(
      req, [this]([[maybe_unused]] rclcpp::Client<
                  tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedFuture result) {
        status_label_->setStyleSheet("QLabel { background-color : lightgreen;}");
        status_label_->setText("OK!!!");

        // unlock button
        initialize_button_->setEnabled(true);
      });
  });

  thread.detach();
}

}  // end namespace tier4_localization_rviz_plugin

PLUGINLIB_EXPORT_CLASS(tier4_localization_rviz_plugin::InitialPoseButtonPanel, rviz_common::Panel)
