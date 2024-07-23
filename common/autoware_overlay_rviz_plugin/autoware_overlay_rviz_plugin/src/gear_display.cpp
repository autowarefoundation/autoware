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

#include "gear_display.hpp"

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

GearDisplay::GearDisplay() : current_gear_(0)
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

void GearDisplay::updateGearData(const autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr & msg)
{
  current_gear_ = msg->report;  // Assuming msg->report contains the gear information
}

void GearDisplay::drawGearIndicator(
  QPainter & painter, const QRectF & backgroundRect, const QColor & color, const QColor & bg_color)
{
  // we deal with the different gears here
  std::string gearString;
  switch (current_gear_) {
    case autoware_vehicle_msgs::msg::GearReport::NEUTRAL:
      gearString = "N";
      break;
    case autoware_vehicle_msgs::msg::GearReport::LOW:
    case autoware_vehicle_msgs::msg::GearReport::LOW_2:
      gearString = "L";
      break;
    case autoware_vehicle_msgs::msg::GearReport::NONE:
    case autoware_vehicle_msgs::msg::GearReport::PARK:
      gearString = "P";
      break;
    case autoware_vehicle_msgs::msg::GearReport::REVERSE:
    case autoware_vehicle_msgs::msg::GearReport::REVERSE_2:
      gearString = "R";
      break;
    // all the drive gears from DRIVE to DRIVE_16
    default:
      gearString = "D";
      break;
  }

  QFont gearFont("Quicksand", 12, QFont::Bold);
  painter.setFont(gearFont);
  QPen borderPen(gray);
  borderPen.setWidth(1);
  painter.setPen(borderPen);

  double gearBoxSize = 37.5;
  double gearX = backgroundRect.left() + 54;
  double gearY = backgroundRect.height() / 2 - gearBoxSize / 2;
  QRect gearRect(gearX, gearY, gearBoxSize, gearBoxSize);
  painter.setBrush(color);
  painter.drawRoundedRect(gearRect, 10, 10);
  painter.setPen(bg_color);
  painter.drawText(gearRect, Qt::AlignCenter, QString::fromStdString(gearString));
}

}  // namespace autoware_overlay_rviz_plugin
