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

#include "traffic_display.hpp"

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

TrafficDisplay::TrafficDisplay()
: tl_red_(QString("#cc3d3d")),
  tl_yellow_(QString("#ccb43d")),
  tl_green_(QString("#3dcc55")),
  tl_gray_(QString("#4f4f4f"))
{
  // Load the traffic light image
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_overlay_rviz_plugin");
  std::string image_path = package_path + "/assets/images/traffic.png";
  traffic_light_image_.load(image_path.c_str());
}

void TrafficDisplay::updateTrafficLightData(
  const autoware_perception_msgs::msg::TrafficLightGroup::ConstSharedPtr & msg)
{
  current_traffic_ = *msg;
}

void TrafficDisplay::drawTrafficLightIndicator(QPainter & painter, const QRectF & backgroundRect)
{
  // Enable Antialiasing for smoother drawing
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

  painter.setBrush(QBrush(tl_gray_, Qt::SolidPattern));
  painter.setPen(Qt::NoPen);
  // Define the area for the circle (background)
  QRectF circleRect = QRectF(50, 50, 50, 50);
  circleRect.moveTopRight(QPointF(
    backgroundRect.right() - circleRect.width() - 75,
    backgroundRect.height() / 2 - circleRect.height() / 2));
  painter.drawEllipse(circleRect);

  if (!current_traffic_.elements.empty()) {
    switch (current_traffic_.elements[0].color) {
      case 1:
        painter.setBrush(QBrush(tl_red_));
        painter.drawEllipse(circleRect);
        break;
      case 2:
        painter.setBrush(QBrush(tl_yellow_));
        painter.drawEllipse(circleRect);
        break;
      case 3:
        painter.setBrush(QBrush(tl_green_));
        painter.drawEllipse(circleRect);
        break;
      case 4:
        painter.setBrush(tl_gray_);
        painter.drawEllipse(circleRect);
        break;
      default:
        painter.setBrush(tl_gray_);
        painter.drawEllipse(circleRect);
        break;
    }
  }

  // Scaling factor (e.g., 1.5 for 150% size)
  float scaleFactor = 0.75;

  // Calculate the scaled size
  QSize scaledSize = traffic_light_image_.size() * scaleFactor;

  // Ensure the scaled image is still within the circle bounds or adjust scaleFactor accordingly

  // Calculate the centered rectangle for the scaled image
  QRectF scaledImageRect(0, 0, scaledSize.width(), scaledSize.height());
  scaledImageRect.moveCenter(circleRect.center());

  // Scale the image to the new size
  QImage scaledTrafficImage =
    traffic_light_image_.scaled(scaledSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);

  // Draw the scaled and centered image
  painter.drawImage(scaledImageRect.topLeft(), scaledTrafficImage);
}

QImage TrafficDisplay::coloredImage(const QImage & source, const QColor & color)
{
  QImage result = source;
  QPainter p(&result);
  p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
  p.fillRect(result.rect(), color);
  p.end();
  return result;
}

}  // namespace autoware_overlay_rviz_plugin
