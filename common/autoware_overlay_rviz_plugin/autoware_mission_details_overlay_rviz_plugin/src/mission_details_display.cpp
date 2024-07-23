// Copyright 2024 The Autoware Contributors
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

#include "mission_details_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/render_system.hpp>

#include <algorithm>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>

namespace autoware::mission_details_overlay_rviz_plugin
{

MissionDetailsDisplay::MissionDetailsDisplay()
{
  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 170, "Width of the overlay", this, SLOT(update_size()));
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 100, "Height of the overlay", this, SLOT(update_size()));
  property_right_ = new rviz_common::properties::IntProperty(
    "Right", 10, "Margin from the right border", this, SLOT(update_size()));
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 10, "Margin from the top border", this, SLOT(update_size()));
  property_bg_alpha_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", 0.3, "Background Alpha", this, SLOT(update_size()));
  property_bg_color_ = new rviz_common::properties::ColorProperty(
    "Background Color", QColor(0, 0, 0), "Background Color", this, SLOT(update_size()));
  property_text_color_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(194, 194, 194), "Text Color", this, SLOT(update_size()));

  // Initialize the component displays
  remaining_distance_time_display_ = std::make_unique<RemainingDistanceTimeDisplay>();
}

void MissionDetailsDisplay::onInitialize()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int count = 0;
  std::stringstream ss;
  ss << "MissionDetailsDisplay" << count++;
  overlay_ =
    std::make_shared<autoware::mission_details_overlay_rviz_plugin::OverlayObject>(ss.str());
  overlay_->show();
  update_size();

  auto rviz_ros_node = context_->getRosNodeAbstraction();

  remaining_distance_time_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
      "Remaining Distance and Time Topic", "/planning/mission_remaining_distance_time",
      "autoware_internal_msgs/msg/MissionRemainingDistanceTime",
      "Topic for Mission Remaining Distance and Time Data", this,
      SLOT(topic_updated_remaining_distance_time()));
  remaining_distance_time_topic_property_->initialize(rviz_ros_node);
}

void MissionDetailsDisplay::setupRosSubscriptions()
{
  // Don't create a node, just use the one from the parent class
  auto rviz_node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  remaining_distance_time_sub_ =
    rviz_node_->create_subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>(
      remaining_distance_time_topic_property_->getTopicStd(),
      rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
      [this](const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
        cb_remaining_distance_time(msg);
      });
}

MissionDetailsDisplay::~MissionDetailsDisplay()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  overlay_.reset();

  remaining_distance_time_sub_.reset();
  remaining_distance_time_display_.reset();
  remaining_distance_time_topic_property_.reset();
}

// mark maybe unused
void MissionDetailsDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;  // Mark as unused
  (void)ros_dt;   // Mark as unused

  if (!overlay_) {
    return;
  }
  autoware::mission_details_overlay_rviz_plugin::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(Qt::transparent);
  draw_widget(hud);
}

void MissionDetailsDisplay::onEnable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);
  if (overlay_) {
    overlay_->show();
  }
  setupRosSubscriptions();
}

void MissionDetailsDisplay::onDisable()
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  remaining_distance_time_sub_.reset();

  if (overlay_) {
    overlay_->hide();
  }
}

void MissionDetailsDisplay::cb_remaining_distance_time(
  const autoware_internal_msgs::msg::MissionRemainingDistanceTime::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (remaining_distance_time_display_) {
    remaining_distance_time_display_->updateRemainingDistanceTimeData(msg);
    queueRender();
  }
}

void MissionDetailsDisplay::draw_widget(QImage & hud)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (!overlay_->isVisible()) {
    return;
  }

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QRectF backgroundRect(0, 0, qreal(property_width_->getInt()), qreal(property_height_->getInt()));
  draw_rounded_rect(painter, backgroundRect);

  if (remaining_distance_time_display_) {
    remaining_distance_time_display_->drawRemainingDistanceTimeDisplay(
      painter, backgroundRect, property_text_color_->getColor());
  }

  painter.end();
}

void MissionDetailsDisplay::draw_rounded_rect(QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(
    property_bg_color_->getColor().hue(), property_bg_color_->getColor().saturation(),
    property_bg_color_->getColor().value());
  colorFromHSV.setAlphaF(property_bg_alpha_->getFloat());

  painter.setBrush(colorFromHSV);

  painter.setPen(Qt::NoPen);
  painter.drawRoundedRect(backgroundRect, backgroundRect.height() / 2, backgroundRect.height() / 2);
}

void MissionDetailsDisplay::reset()
{
  rviz_common::Display::reset();
  overlay_->hide();
}

void MissionDetailsDisplay::update_size()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  overlay_->setPosition(
    property_right_->getInt(), property_top_->getInt(), HorizontalAlignment::RIGHT,
    VerticalAlignment::TOP);
  queueRender();
}

void MissionDetailsDisplay::topic_updated_remaining_distance_time()
{
  // resubscribe to the topic
  remaining_distance_time_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  remaining_distance_time_sub_ =
    rviz_ros_node->get_raw_node()
      ->create_subscription<autoware_internal_msgs::msg::MissionRemainingDistanceTime>(
        remaining_distance_time_topic_property_->getTopicStd(),
        rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable(),
        [this](const autoware_internal_msgs::msg::MissionRemainingDistanceTime::SharedPtr msg) {
          cb_remaining_distance_time(msg);
        });
}

}  // namespace autoware::mission_details_overlay_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::mission_details_overlay_rviz_plugin::MissionDetailsDisplay, rviz_common::Display)
