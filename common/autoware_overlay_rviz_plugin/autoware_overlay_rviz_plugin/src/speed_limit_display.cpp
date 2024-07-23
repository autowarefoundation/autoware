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
#include <qobject.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <string>

namespace autoware_overlay_rviz_plugin
{

SpeedLimitDisplay::SpeedLimitDisplay() : current_limit(0.0), current_speed_(0.0)
{
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_overlay_rviz_plugin");
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

void SpeedLimitDisplay::updateSpeedData(
  const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg)
{
  try {
    float speed = msg->longitudinal_velocity;
    current_speed_ = speed;
  } catch (const std::exception & e) {
    // Log the error
    std::cerr << "Error in processMessage: " << e.what() << std::endl;
  }
}

void SpeedLimitDisplay::drawSpeedLimitIndicator(
  QPainter & painter, const QRectF & backgroundRect, const QColor & color,
  const QColor & light_color, const QColor & dark_color, const QColor & bg_color,
  const float bg_alpha)
{
  // Enable Antialiasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  QColor borderColor = light_color;

  if (current_limit > 0.0) {
    double speed_to_limit_ratio = current_speed_ / current_limit;
    const double speed_to_limit_ratio_min = 0.6;
    const double speed_to_limit_ratio_max = 0.9;

    if (speed_to_limit_ratio >= speed_to_limit_ratio_max) {
      borderColor = dark_color;
    } else if (speed_to_limit_ratio > speed_to_limit_ratio_min) {
      double interpolation_factor = (speed_to_limit_ratio - speed_to_limit_ratio_min) /
                                    (speed_to_limit_ratio_max - speed_to_limit_ratio_min);
      // Interpolate between light_color and dark_color
      int red = light_color.red() + (dark_color.red() - light_color.red()) * interpolation_factor;
      int green =
        light_color.green() + (dark_color.green() - light_color.green()) * interpolation_factor;
      int blue =
        light_color.blue() + (dark_color.blue() - light_color.blue()) * interpolation_factor;
      borderColor = QColor(red, green, blue);
    }
  }

  // Define the area for the outer circle
  QRectF outerCircleRect = QRectF(45, 45, 45, 45);
  outerCircleRect.moveTopRight(QPointF(
    backgroundRect.right() - 44, backgroundRect.height() / 2 - outerCircleRect.height() / 2));

  // Now use borderColor for drawing
  painter.setPen(QPen(borderColor, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.setBrush(borderColor);
  // Draw the outer circle for the speed limit indicator
  painter.drawEllipse(outerCircleRect);

  // Define the area for the inner circle
  QRectF innerCircleRect = outerCircleRect;
  innerCircleRect.setWidth(outerCircleRect.width() / 1.09);
  innerCircleRect.setHeight(outerCircleRect.height() / 1.09);
  innerCircleRect.moveCenter(outerCircleRect.center());

  QRectF innerCircleRect2 = innerCircleRect;

  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(bg_color.hue(), bg_color.saturation(), bg_color.value());
  colorFromHSV.setAlphaF(bg_alpha);
  painter.setBrush(colorFromHSV);
  painter.drawEllipse(innerCircleRect);

  // Add a second inner circle as a mask to make the speed limit indicator look like a ring
  // and follow the rest of the background color as close as possible
  painter.drawEllipse(innerCircleRect2);

  int current_limit_int = std::round(current_limit * 3.6);

  // Define the text to be drawn
  QString text = QString::number(current_limit_int);

  // Set the font and color for the text
  QFont font = QFont("Quicksand", 16, QFont::Bold);

  painter.setFont(font);
  painter.setPen(QPen(color, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  // Draw the text in the center of the circle
  painter.drawText(innerCircleRect, Qt::AlignCenter, text);
}

}  // namespace autoware_overlay_rviz_plugin
