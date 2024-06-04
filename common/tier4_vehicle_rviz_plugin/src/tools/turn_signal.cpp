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

#include "turn_signal.hpp"

#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/uniform_string_stream.hpp>

#include <X11/Xlib.h>

namespace rviz_plugins
{
TurnSignalDisplay::TurnSignalDisplay()
{
  const Screen * screen_info = DefaultScreenOfDisplay(XOpenDisplay(NULL));

  constexpr float hight_4k = 2160.0;
  const float scale = static_cast<float>(screen_info->height) / hight_4k;
  const auto left = static_cast<int>(std::round(196 * scale));
  const auto top = static_cast<int>(std::round(350 * scale));
  const auto width = static_cast<int>(std::round(512 * scale));
  const auto height = static_cast<int>(std::round(256 * scale));

  property_left_ = new rviz_common::properties::IntProperty(
    "Left", left, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", top, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_width_ = new rviz_common::properties::IntProperty(
    "Width", width, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_width_->setMin(10);
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", height, "Height of the plotter window", this, SLOT(updateVisualization()), this);
  property_height_->setMin(10);
}

TurnSignalDisplay::~TurnSignalDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void TurnSignalDisplay::onInitialize()
{
  RTDClass::onInitialize();
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "TurnSignalDisplayObject" << count++;
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

void TurnSignalDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void TurnSignalDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void TurnSignalDisplay::processMessage(
  const autoware_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) {
    return;
  }

  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    last_msg_ptr_ = msg_ptr;
  }

  queueRender();
}

void TurnSignalDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  unsigned int signal_type = 0;
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    if (last_msg_ptr_) {
      signal_type = last_msg_ptr_->report;
    }
  }

  QColor background_color;
  background_color.setAlpha(0);
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(background_color);

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  // turn signal color
  QColor white_color(Qt::white);
  white_color.setAlpha(255);
  if (signal_type == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT) {
    painter.setPen(QPen(white_color, static_cast<int>(2), Qt::DotLine));
    painter.drawPolygon(left_arrow_polygon_, 7);
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.setPen(QPen(white_color, static_cast<int>(2), Qt::SolidLine));
    painter.drawPolygon(right_arrow_polygon_, 7);
  } else if (signal_type == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT) {
    painter.setPen(QPen(white_color, static_cast<int>(2), Qt::DotLine));
    painter.drawPolygon(right_arrow_polygon_, 7);
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.setPen(QPen(white_color, static_cast<int>(2), Qt::SolidLine));
    painter.drawPolygon(left_arrow_polygon_, 7);
  } else if (signal_type == autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE) {
    painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
    painter.setPen(QPen(white_color, static_cast<int>(2), Qt::SolidLine));
    painter.drawPolygon(right_arrow_polygon_, 7);
    painter.drawPolygon(left_arrow_polygon_, 7);
  } else {
    painter.setPen(QPen(white_color, static_cast<int>(2), Qt::DotLine));
    painter.drawPolygon(right_arrow_polygon_, 7);
    painter.drawPolygon(left_arrow_polygon_, 7);
  }
  painter.end();
  updateVisualization();
}

void TurnSignalDisplay::updateVisualization()
{
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  const int w = overlay_->getTextureWidth();
  const int h = overlay_->getTextureHeight();

  right_arrow_polygon_[0].setX(static_cast<float>(w) * 5.0 / 5.0);
  right_arrow_polygon_[0].setY(static_cast<float>(h) * 1.0 / 2.0);
  right_arrow_polygon_[1].setX(static_cast<float>(w) * 4.0 / 5.0);
  right_arrow_polygon_[1].setY(static_cast<float>(h) * 1.0 / 5.0);
  right_arrow_polygon_[2].setX(static_cast<float>(w) * 4.0 / 5.0);
  right_arrow_polygon_[2].setY(static_cast<float>(h) * 2.0 / 5.0);
  right_arrow_polygon_[3].setX(static_cast<float>(w) * 3.0 / 5.0);
  right_arrow_polygon_[3].setY(static_cast<float>(h) * 2.0 / 5.0);
  right_arrow_polygon_[4].setX(static_cast<float>(w) * 3.0 / 5.0);
  right_arrow_polygon_[4].setY(static_cast<float>(h) * 3.0 / 5.0);
  right_arrow_polygon_[5].setX(static_cast<float>(w) * 4.0 / 5.0);
  right_arrow_polygon_[5].setY(static_cast<float>(h) * 3.0 / 5.0);
  right_arrow_polygon_[6].setX(static_cast<float>(w) * 4.0 / 5.0);
  right_arrow_polygon_[6].setY(static_cast<float>(h) * 4.0 / 5.0);

  left_arrow_polygon_[0].setX(static_cast<float>(w) * 0.0 / 5.0);
  left_arrow_polygon_[0].setY(static_cast<float>(h) * 1.0 / 2.0);
  left_arrow_polygon_[1].setX(static_cast<float>(w) * 1.0 / 5.0);
  left_arrow_polygon_[1].setY(static_cast<float>(h) * 1.0 / 5.0);
  left_arrow_polygon_[2].setX(static_cast<float>(w) * 1.0 / 5.0);
  left_arrow_polygon_[2].setY(static_cast<float>(h) * 2.0 / 5.0);
  left_arrow_polygon_[3].setX(static_cast<float>(w) * 2.0 / 5.0);
  left_arrow_polygon_[3].setY(static_cast<float>(h) * 2.0 / 5.0);
  left_arrow_polygon_[4].setX(static_cast<float>(w) * 2.0 / 5.0);
  left_arrow_polygon_[4].setY(static_cast<float>(h) * 3.0 / 5.0);
  left_arrow_polygon_[5].setX(static_cast<float>(w) * 1.0 / 5.0);
  left_arrow_polygon_[5].setY(static_cast<float>(h) * 3.0 / 5.0);
  left_arrow_polygon_[6].setX(static_cast<float>(w) * 1.0 / 5.0);
  left_arrow_polygon_[6].setY(static_cast<float>(h) * 4.0 / 5.0);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TurnSignalDisplay, rviz_common::Display)
