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

#include "turn_signals_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_rendering/render_system.hpp>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

namespace autoware_overlay_rviz_plugin
{

TurnSignalsDisplay::TurnSignalsDisplay() : current_turn_signal_(0)
{
  last_toggle_time_ = std::chrono::steady_clock::now();

  // Load the arrow image
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_overlay_rviz_plugin");
  std::string image_path = package_path + "/assets/images/arrow.png";
  arrowImage.load(image_path.c_str());
}

void TurnSignalsDisplay::updateTurnSignalsData(
  const autoware_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr & msg)
{
  try {
    // Assuming msg->report is the field you're interested in
    current_turn_signal_ = msg->report;
  } catch (const std::exception & e) {
    // Log the error
    std::cerr << "Error in processMessage: " << e.what() << std::endl;
  }
}

void TurnSignalsDisplay::updateHazardLightsData(
  const autoware_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr & msg)
{
  try {
    // Assuming msg->report is the field you're interested in
    current_hazard_lights_ = msg->report;
  } catch (const std::exception & e) {
    // Log the error
    std::cerr << "Error in processMessage: " << e.what() << std::endl;
  }
}

void TurnSignalsDisplay::drawArrows(
  QPainter & painter, const QRectF & backgroundRect, const QColor & color)
{
  QImage scaledLeftArrow = arrowImage.scaled(50, 32, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  scaledLeftArrow = coloredImage(scaledLeftArrow, gray);
  QImage scaledRightArrow = scaledLeftArrow.mirrored(true, false);
  int arrowYPos = (backgroundRect.height() / 2 - scaledLeftArrow.height() / 2 - 4);
  int leftArrowXPos = backgroundRect.left() + scaledLeftArrow.width() * 2 + 180;
  int rightArrowXPos = backgroundRect.right() - scaledRightArrow.width() * 3 - 175;

  bool leftActive =
    (current_turn_signal_ == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT ||
     current_hazard_lights_ == autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE);
  bool rightActive =
    (current_turn_signal_ == autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT ||
     current_hazard_lights_ == autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE);

  // Color the arrows based on the state of the turn signals and hazard lights by having them blink
  // on and off
  if (this->blink_on_) {
    if (leftActive) {
      scaledLeftArrow = coloredImage(scaledLeftArrow, color);
    }
    if (rightActive) {
      scaledRightArrow = coloredImage(scaledRightArrow, color);
    }
  }

  // Draw the arrows
  painter.drawImage(QPointF(leftArrowXPos, arrowYPos), scaledLeftArrow);
  painter.drawImage(QPointF(rightArrowXPos, arrowYPos), scaledRightArrow);

  auto now = std::chrono::steady_clock::now();
  if (
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last_toggle_time_) >=
    blink_interval_) {
    blink_on_ = !blink_on_;  // Toggle the blink state
    last_toggle_time_ = now;
  }
}

QImage TurnSignalsDisplay::coloredImage(const QImage & source, const QColor & color)
{
  QImage result = source;
  QPainter p(&result);
  p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
  p.fillRect(result.rect(), color);
  p.end();
  return result;
}

}  // namespace autoware_overlay_rviz_plugin
