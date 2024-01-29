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

#include "speed_limit_display.hpp"

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

namespace awf_2d_overlay_vehicle
{

SpeedLimitDisplay::SpeedLimitDisplay() : current_limit(0.0)
{
  std::string package_path = ament_index_cpp::get_package_share_directory("awf_2d_overlay_vehicle");
  std::string font_path = package_path + "/assets/font/Quicksand/static/Quicksand-Regular.ttf";
  std::string font_path2 = package_path + "/assets/font/Quicksand/static/Quicksand-Bold.ttf";
  int fontId = QFontDatabase::addApplicationFont(
    font_path.c_str());  // returns -1 on failure (see docs for more info)
  int fontId2 = QFontDatabase::addApplicationFont(
    font_path2.c_str());  // returns -1 on failure (see docs for more info)
  if (fontId == -1 || fontId2 == -1) {
    std::cout << "Failed to load the Quicksand font.";
  }
}

void SpeedLimitDisplay::updateSpeedLimitData(
  const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg)
{
  current_limit = msg->max_velocity;
}

void SpeedLimitDisplay::drawSpeedLimitIndicator(QPainter & painter, const QRectF & backgroundRect)
{
  // Enable Antialiasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  // #C2C2C2
  painter.setPen(QPen(gray, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.setBrush(QBrush(gray, Qt::SolidPattern));

  // Define the area for the outer circle
  QRectF outerCircleRect = backgroundRect;
  outerCircleRect.setWidth(backgroundRect.width() / 2 - 20);
  outerCircleRect.setHeight(backgroundRect.height() - 20);
  outerCircleRect.moveTopLeft(QPointF(backgroundRect.left() + 10, backgroundRect.top() + 10));

  // Define the area for the inner circle
  QRectF innerCircleRect = outerCircleRect;
  innerCircleRect.setWidth(outerCircleRect.width() / 1.375);
  innerCircleRect.setHeight(outerCircleRect.height() / 1.375);
  innerCircleRect.moveCenter(outerCircleRect.center());

  // Draw the outer circle
  painter.drawEllipse(outerCircleRect);

  // Change the composition mode and draw the inner circle
  painter.setCompositionMode(QPainter::CompositionMode_Clear);
  painter.drawEllipse(innerCircleRect);

  // Reset the composition mode
  painter.setCompositionMode(QPainter::CompositionMode_SourceOver);

  int current_limit_int = std::round(current_limit * 3.6);

  // Define the text to be drawn
  QString text = QString::number(current_limit_int);

  // Set the font and color for the text
  QFont font = QFont("Quicksand", 14, QFont::Bold);

  painter.setFont(font);
  // #C2C2C2
  painter.setPen(QPen(gray, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  // Draw the text in the center of the circle
  painter.drawText(innerCircleRect, Qt::AlignCenter, text);
}

}  // namespace awf_2d_overlay_vehicle
